"""CANopen frame decoder and live/candump sniffer runtime."""

from __future__ import annotations

from collections.abc import Callable
from collections.abc import Iterator
import datetime
import pathlib
import re

import can
import canopen
import canopen.objectdictionary
import click
from loguru import logger

from canopener import capture
from canopener import decode
from canopener import log_writer
from canopener import pdo_mapping

_NMT_CMDS = {
    0x01: "start",
    0x02: "stop",
    0x80: "pre-op",
    0x81: "reset",
    0x82: "reset-comm",
}

_HEARTBEAT_STATES = {
    0x00: "boot",
    0x04: "stopped",
    0x05: "operational",
    0x7F: "pre-operational",
}


class CanOpenDecoder:
    """Stateful CANopen frame decoder."""

    def __init__(
        self,
        eds_path: str | None = None,
        pdo_config_path: pathlib.Path | None = None,
        on_sdo_write: Callable[[int, int, int, int], None] | None = None,
    ) -> None:
        """Instantiates a CanOpenDecoder.

        Args:
            eds_path: Optional path to an EDS file used to resolve names.
            pdo_config_path: Optional YAML file used to persist discovered PDO
                mappings across runs.
            on_sdo_write: Optional callback invoked for every expedited SDO
                download (write) seen on the bus, with arguments
                ``(node_id, index, subindex, value)``.  Called *before* the
                decoder's own PDO-mapping bookkeeping runs.
        """
        self._eds_path: str | None = eds_path
        self._on_sdo_write = on_sdo_write
        # od[node_id] loaded lazily from _eds_path on first contact with that node
        self.od: dict[int, canopen.ObjectDictionary] = {}
        self.pdo = pdo_mapping.PdoMappingTracker(
            config_path=pdo_config_path,
            od_provider=self._get_od,
        )

    def decode(self, msg: can.Message) -> str:
        """Decode a raw CAN frame as a CANopen message.

        Args:
            msg: Received CAN message.

        Returns:
            Human-readable description of the CANopen message.
        """
        cob = msg.arbitration_id
        node_id = cob & 0x7F
        # Ignore all, or host.
        if node_id > 1:
            self._get_od(node_id)

        if cob == 0x000:
            return self._decode_nmt(msg)
        if cob == 0x080:
            return "SYNC"
        if 0x081 <= cob <= 0x0FF:
            return f"EMCY node={node_id}  {decode.decode_emcy(msg.data)}"
        if cob == 0x100:
            return f"TIME data={msg.data.hex()}"
        if 0x581 <= cob <= 0x5FF:
            return self._decode_sdo(msg, node_id, is_response=True)
        if 0x601 <= cob <= 0x67F:
            return self._decode_sdo(msg, node_id, is_response=False)
        if 0x701 <= cob <= 0x77F:
            return self._decode_heartbeat(msg, node_id)

        if 0x181 <= cob <= 0x57F:
            return self._decode_pdo(msg, cob, self.pdo.name_for(cob))

        return f"UNKNOWN cob=0x{cob:03X} data={msg.data.hex()}"

    def _get_od(self, node_id: int) -> canopen.ObjectDictionary | None:
        """Return the OD for node_id, loading it from EDS if this is the first contact."""
        if self._eds_path is None:
            return None
        if node_id not in self.od:
            self.od[node_id] = canopen.import_od(self._eds_path, node_id)
            self.pdo.apply_eds_defaults(node_id, self.od[node_id])
        return self.od[node_id]

    def _handle_sdo_write(
        self, node_id: int, index: int, subindex: int, value: int
    ) -> None:
        """Forward an expedited SDO write to the user callback and PDO tracker.

        The user callback runs *before* the tracker's bookkeeping.
        """
        if self._on_sdo_write is not None:
            self._on_sdo_write(node_id, index, subindex, value)
        self.pdo.handle_sdo_write(node_id, index, subindex, value)

    def print_msg(self, msg: can.Message, label: str) -> None:
        """Decode and print a CAN message to stdout."""
        decoded = self.decode(msg)
        raw = " ".join(f"{b:02X}" for b in msg.data)
        raw_padded = f"[{raw:<{8 * 3 - 1}}]"
        click.echo(f"{label}  0x{msg.arbitration_id:03X}  {raw_padded}  {decoded}")

    def _decode_pdo(self, msg: can.Message, cob_id: int, pdo_name: str) -> str:
        # Prefer the tracker's owning-node ID over the default cob -> node
        # mapping, since PDOs may be remapped to arbitrary COB-IDs.
        node_id = self.pdo.node_for(cob_id)
        if node_id is None:
            node_id = cob_id & 0x7F
        signals = self.pdo.mapping_for(cob_id)
        if signals is None:
            return f"{pdo_name} node={node_id} data={msg.data.hex()}"

        parts: list[str] = []
        bit_offset = 0
        raw = int.from_bytes(msg.data, "little")
        for name, bits in signals:
            mask = (1 << bits) - 1
            value = (raw >> bit_offset) & mask
            parts.append(name + "=" + decode.annotate_word(name, value))
            bit_offset += bits
        return f"{pdo_name} node={node_id} [{' '.join(parts)}]"

    @staticmethod
    def _decode_nmt(msg: can.Message) -> str:
        if len(msg.data) >= 2:
            cmd_name = _NMT_CMDS.get(msg.data[0], f"0x{msg.data[0]:02X}")
            return f"NMT cmd={cmd_name} node={msg.data[1]}"
        return f"NMT data={msg.data.hex()}"

    @staticmethod
    def _decode_heartbeat(msg: can.Message, node_id: int) -> str:
        state = msg.data[0] if msg.data else 0
        state_name = _HEARTBEAT_STATES.get(state, f"0x{state:02X}")
        return f"HEARTBEAT node={node_id} state={state_name}"

    def _sdo_name(self, index: int, subindex: int, node_id: int) -> str:
        """Resolve an SDO object index/subindex to a human-readable name with raw location."""
        raw = f"0x{index:04X}:{subindex:02X}"
        od = self._get_od(node_id)
        if od is None:
            return raw
        try:
            obj = od[index]
            obj_name: str = obj.name
            if isinstance(obj, canopen.objectdictionary.ODRecord):
                sub = obj[subindex]
                return f'"{obj_name}"."{sub.name}" ({raw})'
            return f'"{obj_name}" ({raw})'
        except (KeyError, AttributeError):
            return raw

    def _sdo_leaf_name(self, index: int, subindex: int, node_id: int) -> str | None:
        """Return the leaf parameter name for word-decoder lookup."""
        od = self._get_od(node_id)
        if od is None:
            return None
        try:
            obj = od[index]
            if isinstance(
                obj,
                (canopen.objectdictionary.ODRecord, canopen.objectdictionary.ODArray),
            ):
                return obj[subindex].name
            return obj.name
        except (KeyError, AttributeError):
            return None

    def _decode_sdo(self, msg: can.Message, node_id: int, *, is_response: bool) -> str:
        direction = "RES" if is_response else "REQ"
        if len(msg.data) < 4:
            return f"SDO-{direction} node={node_id} data={msg.data.hex()}"

        cmd = msg.data[0]
        cs = (cmd >> 5) & 0x7

        # Abort transfer (0x80, cs=4) — either direction, bytes 1-3 are index/subindex
        if cs == 4:
            index = int.from_bytes(msg.data[1:3], "little")
            subindex = msg.data[3]
            abort_code = int.from_bytes(msg.data[4:], "little")
            return f"SDO-ABORT node={node_id} {self._sdo_name(index, subindex, node_id)} code=0x{abort_code:08X}"

        if is_response:
            if cs == 2:
                # Initiate upload response (0x40-0x5F): read data from node
                index = int.from_bytes(msg.data[1:3], "little")
                subindex = msg.data[3]
                loc = self._sdo_name(index, subindex, node_id)
                e = (cmd >> 1) & 1  # expedited
                s = cmd & 1  # size indicated
                if e:
                    n = (cmd >> 2) & 3  # bytes NOT containing data
                    size = 4 - n
                    data_bytes = bytes(msg.data[4 : 4 + size])
                    leaf = self._sdo_leaf_name(index, subindex, node_id)
                    return f"SDO-READ node={node_id} {loc} data={decode.annotate_sdo_data(leaf, data_bytes)}"
                size_str = (
                    f" size={int.from_bytes(msg.data[4:], 'little')}" if s else ""
                )
                return f"SDO-READ-INIT node={node_id} {loc}{size_str}"
            if cs == 3:
                # Initiate download response (0x60): write acknowledged by node
                index = int.from_bytes(msg.data[1:3], "little")
                subindex = msg.data[3]
                return f"SDO-WRITE-ACK node={node_id} {self._sdo_name(index, subindex, node_id)}"
            if cs == 0:
                # Upload segment response (0x00-0x1F): segmented read data from node
                t = (cmd >> 4) & 1  # toggle bit
                n = (cmd >> 1) & 7  # bytes NOT containing data
                c = cmd & 1  # c=1 means last segment
                seg_data = bytes(msg.data[1 : 8 - n])
                suffix = " last" if c else ""
                return (
                    f"SDO-READ-SEG node={node_id} t={t} data={seg_data.hex()}{suffix}"
                )
            if cs == 1:
                # Download segment response (0x20-0x3F): write segment acknowledged
                t = (cmd >> 4) & 1
                return f"SDO-WRITE-SEG-ACK node={node_id} t={t}"
        else:
            if cs == 2:
                # Initiate upload request (0x40): request read from node
                index = int.from_bytes(msg.data[1:3], "little")
                subindex = msg.data[3]
                return f"SDO-READ-REQ node={node_id} {self._sdo_name(index, subindex, node_id)}"
            if cs == 3:
                # Upload segment request (0x60/0x70): request next segment
                t = (cmd >> 4) & 1
                return f"SDO-READ-SEG-REQ node={node_id} t={t}"
            if cs == 1:
                # Initiate download request (0x20-0x3F): write to node
                index = int.from_bytes(msg.data[1:3], "little")
                subindex = msg.data[3]
                loc = self._sdo_name(index, subindex, node_id)
                e = (cmd >> 1) & 1
                s = cmd & 1
                if e and s:
                    n = (cmd >> 2) & 3
                    size = 4 - n
                    data_bytes = bytes(msg.data[4 : 4 + size])
                    self._handle_sdo_write(
                        node_id,
                        index,
                        subindex,
                        int.from_bytes(data_bytes, "little"),
                    )
                    leaf = self._sdo_leaf_name(index, subindex, node_id)
                    return f"SDO-WRITE node={node_id} {loc} data={decode.annotate_sdo_data(leaf, data_bytes)}"
                return (
                    f"SDO-WRITE node={node_id} {loc} data={bytes(msg.data[4:]).hex()}"
                )
            if cs == 0:
                # Download segment request (0x00-0x1F): send next write segment
                t = (cmd >> 4) & 1
                n = (cmd >> 1) & 7
                c = cmd & 1
                seg_data = bytes(msg.data[1 : 8 - n])
                suffix = " last" if c else ""
                return (
                    f"SDO-WRITE-SEG node={node_id} t={t} data={seg_data.hex()}{suffix}"
                )

        return f"SDO-{direction} node={node_id} cmd=0x{cmd:02X}"


_CANDUMP_RE = re.compile(
    r"^\s*"
    r"(?:\((?P<ts>[0-9]+\.[0-9]+)\)\s+)?"  # optional (timestamp)
    r"\S+\s+"  # interface
    r"(?P<id>[0-9A-Fa-f]+)\s+"  # CAN ID
    r"\[(?P<dlc>[0-9]+)\]"  # [DLC]
    r"(?P<data>(?:\s+[0-9A-Fa-f]{2})*)"  # data bytes
)


def _parse_candump(path: str) -> Iterator[tuple[can.Message, float | None]]:
    """Yield (message, timestamp) pairs parsed from a candump log file.

    Supports both timestamped (``-t a`` / ``-t d``) and plain formats.
    ``timestamp`` is ``None`` for files without timestamps.

    Args:
        path: Path to the candump log file.

    Yields:
        Tuples of (CAN message, timestamp or None).
    """
    with open(path) as f:
        for line in f:
            m = _CANDUMP_RE.match(line)
            if m is None:
                continue
            ts: float | None = float(m.group("ts")) if m.group("ts") else None
            arb_id = int(m.group("id"), 16)
            data_str = m.group("data").strip()
            data = bytes(int(b, 16) for b in data_str.split()) if data_str else b""
            yield (
                can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    timestamp=ts if ts is not None else 0.0,
                    is_extended_id=arb_id > 0x7FF,
                ),
                ts,
            )


def format_timestamp(t: float) -> str:
    """Format a Unix timestamp as ``HH:MM:SS.mmm``."""
    return (
        datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.")
        + f"{t % 1 * 1000:03.0f}"
    )


def run(
    interface: str,
    eds: str | None,
    save: bool,
    save_dir: str,
    rotate: float,
    compress: bool,
) -> None:
    """Run the live sniffer on a SocketCAN interface.

    Args:
        interface: SocketCAN interface name.
        eds: Optional path to an EDS file for signal decoding.
        save: When True, persist live capture frames to rotating log files.
        save_dir: Directory for saved log files and ``pdo.yaml``.
        rotate: Seconds before rotating the log file.
        compress: When True, bzip2-compress log files after rotation.
    """
    pdo_config_path = pathlib.Path(save_dir) / "pdo.yaml"
    decoder = CanOpenDecoder(eds, pdo_config_path=pdo_config_path)
    decoder.pdo.load_config()

    logger.info(f"Listening on {interface} — press Ctrl+C to stop")
    with capture.LiveCapture(interface) as cap:
        if save:
            with log_writer.LogWriter(save_dir, interface, rotate, compress) as lw:
                for msg in cap:
                    lw.write(msg)
                    decoder.print_msg(msg, format_timestamp(msg.timestamp))
        else:
            for msg in cap:
                decoder.print_msg(msg, format_timestamp(msg.timestamp))


def decode_candump(
    candump_path: str,
    eds: str | None,
    pdo_config: str | None,
) -> None:
    """Decode a candump log file offline.

    Args:
        candump_path: Path to the candump log file to replay.
        eds: Optional path to an EDS file for signal decoding.
        pdo_config: Optional path to a ``pdo.yaml`` file with discovered PDO
          mappings; when None, no persisted PDO config is loaded.
    """
    pdo_config_path = pathlib.Path(pdo_config) if pdo_config else None
    decoder = CanOpenDecoder(eds, pdo_config_path=pdo_config_path)
    decoder.pdo.load_config()

    for i, (msg, ts) in enumerate(_parse_candump(candump_path)):
        label = format_timestamp(ts) if ts is not None else f"#{i + 1:>11}"
        decoder.print_msg(msg, label)
