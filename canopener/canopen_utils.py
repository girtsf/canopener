"""CANopen utility runners called from cli.py.

Provides ping, SDO read/write, NMT reset, PDO dump, homing, and limp
commands plus shared low-level helpers (SDO access, status polling,
CiA 402 enable sequence) used by both these utilities and
``pvt_demo``.
"""

from __future__ import annotations

import threading
import time

import canopen
import canopen.objectdictionary
from canopen.sdo.base import SdoArray
from canopen.sdo.base import SdoRecord
from canopen.sdo.base import SdoVariable
from canopen.sdo.exceptions import SdoAbortedError
import click
from loguru import logger

from canopener import decode


# ---------------------------------------------------------------------------
# Built-in CiA 402 object dictionary (fallback when no EDS is supplied)
# ---------------------------------------------------------------------------

# The drive/motion helpers below (enable_drive, home, limp) address a small,
# fixed set of standard CiA 402 objects by index.  When no EDS file is given we
# synthesize a minimal object dictionary covering just these objects, so typed
# SDO access (correct byte width and signedness) keeps working without an EDS.
_CIA402_FALLBACK_OBJECTS = (
    (0x6040, "Controlword", canopen.objectdictionary.UNSIGNED16),
    (0x6041, "Statusword", canopen.objectdictionary.UNSIGNED16),
    (0x6060, "Modes of Operation", canopen.objectdictionary.INTEGER8),
    (0x6061, "Modes of Operation Display", canopen.objectdictionary.INTEGER8),
    (0x6064, "Position Actual Value", canopen.objectdictionary.INTEGER32),
    (0x607C, "Home Offset", canopen.objectdictionary.INTEGER32),
    (0x6098, "Homing Method", canopen.objectdictionary.INTEGER8),
)


def _cia402_fallback_od() -> canopen.ObjectDictionary:
    """Build a minimal CiA 402 object dictionary for the drive/motion helpers.

    Used as the object dictionary when no EDS file is supplied so that
    ``enable_drive``, ``home``, and ``limp`` can address the standard
    controlword/statusword/position objects with correct datatypes.
    """
    od = canopen.ObjectDictionary()
    for index, name, data_type in _CIA402_FALLBACK_OBJECTS:
        var = canopen.objectdictionary.ODVariable(name, index, 0)
        var.data_type = data_type
        var.access_type = "rw"
        od.add_object(var)
    return od


# ---------------------------------------------------------------------------
# Low-level SDO helpers (shared with pvt_demo)
# ---------------------------------------------------------------------------


def _sdo(
    node: canopen.RemoteNode, index: int, subindex: int | None = None
) -> SdoVariable:
    """Return an SdoVariable for the given index (and optional subindex).

    Args:
        node: Remote CANopen node.
        index: Object dictionary index.
        subindex: Sub-index for record/array objects; omit for plain variables.

    Raises:
        TypeError: If the object type does not match the subindex usage.
    """
    obj = node.sdo[index]
    if subindex is not None:
        if not isinstance(obj, (SdoArray, SdoRecord)):
            raise TypeError(
                f"Object 0x{index:04X} is not an array/record (got {type(obj).__name__})"
            )
        return obj[subindex]
    if not isinstance(obj, SdoVariable):
        raise TypeError(f"Object 0x{index:04X} has sub-indexes; provide a subindex")
    return obj


def _sdo_read(node: canopen.RemoteNode, index: int, subindex: int | None = None) -> int:
    """Read an integer SDO value.

    Args:
        node: Remote CANopen node.
        index: Object dictionary index.
        subindex: Sub-index for record/array objects.

    Returns:
        Raw value as int.

    Raises:
        TypeError: If the raw value is not an integer type.
    """
    raw = _sdo(node, index, subindex).raw
    if not isinstance(raw, (int, bool)):
        raise TypeError(
            f"SDO 0x{index:04X} raw value is {type(raw).__name__}, expected int"
        )
    return int(raw)


def _wait_status(
    node: canopen.RemoteNode,
    mask: int,
    expected: int,
    timeout: float = 5.0,
    index: int = 0x6041,
    subindex: None | int = None,
) -> None:
    """Poll an object dictionary word until (value & mask) == expected.

    Args:
        node: Remote CANopen node.
        mask: Bit mask applied to the value before comparison.
        expected: Target value after masking.
        timeout: Seconds before raising TimeoutError.
        index: Object index to poll (default 0x6041, status word).
        subindex: Object subindex to poll.

    Raises:
        TimeoutError: If the condition is not met within timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sw = _sdo_read(node, index, subindex)
        if (sw & mask) == expected:
            return
        time.sleep(0.05)
    sw = _sdo_read(node, index, subindex)
    addr = f"0x{index:04X}" if subindex is None else f"0x{index:04X}.{subindex:02X}"
    raise TimeoutError(
        f"{addr} = 0x{sw:04X}: (& 0x{mask:04X}) != 0x{expected:04X} after {timeout}s"
    )


def enable_drive(node: canopen.RemoteNode) -> None:
    """Bring the drive to Operation Enabled via the CiA 402 state machine.

    Sequence: Shutdown → Ready to Switch On → Enable Operation → Operation Enabled.
    Checks for Fault bit and aborts if set.

    Args:
        node: Remote CANopen node.

    Raises:
        RuntimeError: If the drive is in Fault state.
        TimeoutError: If a state transition times out.
    """
    sw = _sdo_read(node, 0x6041)
    if sw & 0x0008:  # bit 3 = Fault
        raise RuntimeError(
            f"Drive is in Fault state (6041h = 0x{sw:04X}). "
            "Clear fault before proceeding."
        )

    logger.info("6040h = 0x0006 (Shutdown → Ready to Switch On)")
    _sdo(node, 0x6040).raw = 0x0006
    # CiA 402 "Ready to Switch On": bits [6:0] of status word = 0x21
    # mask 0x006F covers the state bits (0,1,2,3,5,6); expected = 0x0021
    _wait_status(node, mask=0x006F, expected=0x0021)

    logger.info("6040h = 0x000F (Enable Operation → Operation Enabled)")
    _sdo(node, 0x6040).raw = 0x000F
    # "Operation Enabled": bits [6:0] = 0x27 (bits 0,1,2,5 set)
    _wait_status(node, mask=0x006F, expected=0x0027)
    logger.info("Drive is Operation Enabled")


# ---------------------------------------------------------------------------
# Index resolution helpers (used by sdo-read/sdo-write)
# ---------------------------------------------------------------------------


def _search_od(od: canopen.ObjectDictionary, name: str) -> tuple[int, int] | None:
    """Search an ObjectDictionary for a variable whose name matches case-insensitively.

    Checks top-level ODVariable objects and sub-variables inside ODRecord/ODArray
    objects.  Record/array names themselves are not matched.

    Args:
        od: Object dictionary to search.
        name: Name to look for (case-insensitive).

    Returns:
        ``(index, subindex)`` of the first match, or ``None``.
    """
    name_lower = name.lower()
    for obj in od.values():
        if isinstance(obj, canopen.objectdictionary.ODVariable):
            if obj.name.lower() == name_lower:
                return (obj.index, 0)
        elif isinstance(
            obj, (canopen.objectdictionary.ODRecord, canopen.objectdictionary.ODArray)
        ):
            for sub in obj.values():
                if (
                    isinstance(sub, canopen.objectdictionary.ODVariable)
                    and sub.name.lower() == name_lower
                ):
                    return (obj.index, sub.subindex)
    return None


def _resolve_index(
    index_str: str,
    subindex: int,
    eds_path: str | None = None,
    node_id: int = 0,
) -> tuple[int, int]:
    """Resolve an SDO index from a numeric string or a parameter name.

    Resolution order:
    1. Decimal/hex integer string (e.g. ``'0x6064'``).
    2. Hardcoded CiA 402 name from :data:`decode.NAME_TO_INDEX`.
    3. Name search in the EDS file (if *eds_path* is given).

    For named lookups the returned subindex comes from the name; *subindex* is
    only used for numeric lookups.

    Args:
        index_str: Integer string or parameter name.
        subindex: Subindex to use for numeric lookups.
        eds_path: Optional path to an EDS file for extended name lookup.
        node_id: Node ID passed to :func:`canopen.import_od` when loading EDS.

    Raises:
        click.BadParameter: If ``index_str`` is neither a valid integer nor a
            known name.
    """
    try:
        return int(index_str, 0), subindex
    except ValueError:
        pass

    resolved = decode.NAME_TO_INDEX.get(index_str.lower())
    if resolved is not None:
        return resolved

    if eds_path is not None:
        od = canopen.import_od(eds_path, node_id)
        resolved = _search_od(od, index_str)
        if resolved is not None:
            return resolved

    known = ", ".join(sorted(decode.NAME_TO_INDEX))
    eds_hint = "" if eds_path is not None else " (use --eds for EDS name lookup)"
    raise click.BadParameter(
        f"unknown name {index_str!r}{eds_hint}. Built-in names: {known}",
        param_hint="INDEX",
    )


# ---------------------------------------------------------------------------
# Runners
# ---------------------------------------------------------------------------


def ping(
    interface: str, node_id: int, count: int, interval: float, timeout: float
) -> None:
    """Ping a CANopen node via SDO read of Device Type (0x1000).

    Sends an SDO upload request for object 0x1000 (Device Type) and
    measures round-trip time.  Prints per-ping results and a summary
    on exit, similar to ICMP ping.  ``count=0`` pings until Ctrl+C.
    """
    req_cob = 0x600 + node_id
    rsp_cob = 0x580 + node_id
    # SDO upload initiate: read 0x1000 sub 0
    request = bytes([0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00])

    network = canopen.Network()
    network.connect(channel=interface, bustype="socketcan")

    received = threading.Event()
    rsp_buf: list[bytes] = []

    def _on_sdo(_cob_id: int, data: bytearray, _ts: float) -> None:
        rsp_buf.clear()
        rsp_buf.append(bytes(data))
        received.set()

    network.subscribe(rsp_cob, _on_sdo)
    click.echo(f"PING node {node_id} on {interface} (SDO read 0x1000)")

    rtts: list[float] = []
    sent = 0
    try:
        i = 0
        while count == 0 or i < count:
            received.clear()
            t0 = time.monotonic()
            network.send_message(req_cob, request)
            sent += 1

            if received.wait(timeout=timeout):
                rtt_ms = (time.monotonic() - t0) * 1000.0
                rtts.append(rtt_ms)
                data = rsp_buf[0]
                if data[0] == 0x80:
                    abort_code = int.from_bytes(data[4:8], "little")
                    click.echo(
                        f"node {node_id}: SDO abort 0x{abort_code:08X}  time={rtt_ms:.2f} ms"
                    )
                else:
                    device_type = int.from_bytes(data[4:8], "little")
                    click.echo(
                        f"node {node_id}: device_type=0x{device_type:08X}  time={rtt_ms:.2f} ms"
                    )
            else:
                click.echo(f"node {node_id}: timeout")

            i += 1
            if count == 0 or i < count:
                time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        network.unsubscribe(rsp_cob, _on_sdo)
        network.disconnect()

    click.echo(f"\n--- node {node_id} ping statistics ---")
    loss = sent - len(rtts)
    loss_pct = loss * 100 // sent if sent else 0
    click.echo(f"{sent} sent, {len(rtts)} received, {loss_pct}% loss")
    if rtts:
        avg = sum(rtts) / len(rtts)
        click.echo(f"rtt min/avg/max = {min(rtts):.2f}/{avg:.2f}/{max(rtts):.2f} ms")


def sdo_read(
    interface: str,
    node_id: int,
    index: str,
    subindex: int,
    timeout: float,
    eds_path: str | None,
) -> None:
    """Read an SDO object from the node (expedited transfer only).

    ``index`` may be a decimal/hex integer string, a built-in CiA 402 name,
    or any object/sub-object name from the EDS file.  ``subindex`` is
    ignored for named lookups.
    """
    index_int, subindex = _resolve_index(index, subindex, eds_path, node_id)

    network = canopen.Network()
    network.connect(channel=interface, bustype="socketcan")
    try:
        node: canopen.RemoteNode = network.add_node(node_id, eds_path)
        node.sdo.RESPONSE_TIMEOUT = timeout
        value_bytes = bytes(node.sdo.upload(index_int, subindex))
        n_bytes = len(value_bytes)
        value_u = int.from_bytes(value_bytes, "little", signed=False)
        value_s = int.from_bytes(value_bytes, "little", signed=True)
        annotation = decode.annotate_by_index(index_int, subindex, value_bytes)
        annotation_str = f"  [{annotation}]" if annotation else ""
        bin_str = f"{value_u:0{n_bytes * 8}b}"
        bin_grouped = "_".join(bin_str[i : i + 8] for i in range(0, len(bin_str), 8))
        click.echo(
            f"0x{index_int:04X}.{subindex:02X}  "
            f"hex=0x{value_u:0{n_bytes * 2}X}  uint={value_u}  int={value_s}  "
            f"bin=0b{bin_grouped}  raw={value_bytes.hex()}{annotation_str}"
        )
    finally:
        network.disconnect()


def sdo_write(
    interface: str,
    node_id: int,
    index: str,
    subindex: int,
    value: int,
    size: str,
    timeout: float,
    eds_path: str | None,
) -> None:
    """Write an SDO object on the node.

    ``index`` may be a decimal/hex integer string, a built-in CiA 402 name,
    or any object/sub-object name from the EDS file.  ``subindex`` is
    ignored for named lookups.  ``size`` is one of ``"1"``, ``"2"``,
    ``"4"``, ``"6"``, ``"8"`` bytes.
    """
    index_int, subindex = _resolve_index(index, subindex, eds_path, node_id)
    n_bytes = int(size)
    mask = (1 << (n_bytes * 8)) - 1
    value_bytes = (value & mask).to_bytes(n_bytes, "little")

    network = canopen.Network()
    network.connect(channel=interface, bustype="socketcan")
    try:
        node: canopen.RemoteNode = network.add_node(node_id, eds_path)
        node.sdo.RESPONSE_TIMEOUT = timeout
        node.sdo.download(index_int, subindex, value_bytes)

        annotation = decode.annotate_by_index(index_int, subindex, value_bytes)
        annotation_str = f"  [{annotation}]" if annotation else ""
        click.echo(
            f"0x{index_int:04X}.{subindex:02X} ← "
            f"0x{value & mask:0{n_bytes * 2}X}{annotation_str}"
        )
    finally:
        network.disconnect()


def nmt_reset(interface: str, node_id: int) -> None:
    """Send an NMT Reset Node command to the configured node."""
    network = canopen.Network()
    network.connect(channel=interface, bustype="socketcan")
    try:
        network.send_message(0x000, bytes([0x81, node_id]))
        click.echo(f"NMT Reset Node sent to node {node_id}")
    finally:
        network.disconnect()


def _fmt_pdo_tx_type(t: int) -> str:
    """Format a PDO transmission type byte as a short tag.

    Args:
        t: Transmission type byte from comm parameter sub-index 02h.
    """
    if t == 0x00:
        return "sync-acyclic"
    if 0x01 <= t <= 0xF0:
        return f"sync-every-{t}"
    if t == 0xFC:
        return "rtr-sync"
    if t == 0xFD:
        return "rtr-async"
    if t == 0xFE:
        return "async-mfr"
    if t == 0xFF:
        return "async"
    return f"type-0x{t:02X}"


def _dump_single_pdo(
    node: canopen.RemoteNode, pdo_num: int, is_tx: bool
) -> tuple[str, str, str] | None:
    """Collect comm + mapping params for one PDO.

    Returns ``None`` if the comm parameter object does not exist on the
    device (SDO abort 0x06020000), so callers can probe a range without
    knowing how many PDOs the device supports.

    Args:
        node: Remote CANopen node.
        pdo_num: 1-based PDO number.
        is_tx: True for TPDO, False for RPDO.

    Returns:
        ``(label, left, right)`` where ``left`` is the comm summary
        ``[hex...] (annotations)`` and ``right`` is the mapping summary.
    """
    if is_tx:
        comm_idx = 0x1800 + (pdo_num - 1)
        map_idx = 0x1A00 + (pdo_num - 1)
        label = f"TPDO {pdo_num:2d}"
    else:
        comm_idx = 0x1400 + (pdo_num - 1)
        map_idx = 0x1600 + (pdo_num - 1)
        label = f"RPDO {pdo_num:2d}"

    try:
        comm_count_raw = node.sdo.upload(comm_idx, 0)
    except SdoAbortedError as e:
        if e.code == 0x06020000:
            return None
        raise

    comm_count = int.from_bytes(comm_count_raw, "little")
    comm_parts: list[str] = []
    cob_id: int | None = None
    tx_type: int | None = None
    for sub in range(1, comm_count + 1):
        try:
            data = node.sdo.upload(comm_idx, sub)
        except SdoAbortedError as e:
            if e.code == 0x06090011:  # sub-index does not exist
                continue
            raise
        val = int.from_bytes(data, "little")
        comm_parts.append(f"{sub:02X}={val:0{len(data) * 2}X}")
        if sub == 1:
            cob_id = val
        elif sub == 2:
            tx_type = val

    annot: list[str] = []
    if cob_id is not None:
        enabled = (cob_id & 0x80000000) == 0
        cid = cob_id & 0x7FF
        annot.append(f"COB-ID 0x{cid:03X} {'enabled' if enabled else 'disabled'}")
    if tx_type is not None:
        annot.append(_fmt_pdo_tx_type(tx_type))
    annot_str = f" ({', '.join(annot)})" if annot else ""

    map_count_raw = node.sdo.upload(map_idx, 0)
    map_count = int.from_bytes(map_count_raw, "little")
    map_parts: list[str] = [f"00={map_count:0{len(map_count_raw) * 2}X}"]
    for sub in range(1, map_count + 1):
        data = node.sdo.upload(map_idx, sub)
        val = int.from_bytes(data, "little")
        map_parts.append(f"{sub:02X}={val:0{len(data) * 2}X}")

    left = f"[{' '.join(comm_parts)}]{annot_str}"
    right = f"[{' '.join(map_parts)}]"
    return (label, left, right)


def dump_pdo(
    interface: str, node_id: int, max_rpdo: int, max_tpdo: int, eds_path: str | None
) -> None:
    """Dump RPDO/TPDO communication and mapping parameters via SDO.

    Probes RPDOs 1..max_rpdo and TPDOs 1..max_tpdo.  PDOs whose comm
    parameter object is absent (SDO abort 0x06020000) are skipped.  For
    each PDO that exists a one-line summary is printed with raw hex of
    every comm sub-index, a decoded annotation (COB-ID state + transmission
    type), and raw hex of every mapping sub-index.
    """
    network = canopen.Network()
    network.connect(channel=interface, bustype="socketcan")
    try:
        node: canopen.RemoteNode = network.add_node(node_id, eds_path)
        rows: list[tuple[str, str, str]] = []
        for n in range(1, max_rpdo + 1):
            row = _dump_single_pdo(node, n, is_tx=False)
            if row is not None:
                rows.append(row)
        for n in range(1, max_tpdo + 1):
            row = _dump_single_pdo(node, n, is_tx=True)
            if row is not None:
                rows.append(row)
    finally:
        network.disconnect()

    left_width = max((len(left) for _, left, _ in rows), default=0)
    for label, left, right in rows:
        click.echo(f"{label}: {left:<{left_width}} | {right}")


def home(interface: str, node_id: int, eds_path: str | None) -> None:
    """Zero the position using homing method 35 (current position as home).

    Enables the drive, switches to Homing mode (6), sets method 35, triggers
    homing, and waits for Homing Attained.  Prints Position Actual Value
    (6064h) and Home Offset (607Ch) before and after.
    """
    network = canopen.Network()
    network.connect(channel=interface, bustype="socketcan")
    try:
        node: canopen.RemoteNode = network.add_node(
            node_id, eds_path or _cia402_fallback_od()
        )

        enable_drive(node)

        logger.info("6060h = 6 (Homing mode)")
        _sdo(node, 0x6060).raw = 6
        time.sleep(0.1)
        display = _sdo_read(node, 0x6061)
        if display != 6:
            raise RuntimeError(f"Mode display (6061h) = {display}, expected 6")

        logger.info("6098h = 35 (current position as home)")
        _sdo(node, 0x6098).raw = 35

        pos_before = _sdo_read(node, 0x6064)
        offset_before = _sdo_read(node, 0x607C)
        click.echo("\nBefore homing:")
        click.echo(f"  Position Measured (6064h): {pos_before}")
        click.echo(f"  Home Offset       (607Ch): {offset_before}")

        # Rising edge on bit 4 starts homing
        _sdo(node, 0x6040).raw = 0x000F
        time.sleep(0.05)
        logger.info("6040h = 0x001F (Start Homing)")
        _sdo(node, 0x6040).raw = 0x001F

        logger.info("Waiting for Homing Attained (6041h bit 12)...")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            sw = _sdo_read(node, 0x6041)
            if sw & (1 << 13):
                raise RuntimeError(f"Homing Error set (6041h = 0x{sw:04X})")
            if sw & (1 << 12):
                break
            time.sleep(0.05)
        else:
            sw = _sdo_read(node, 0x6041)
            raise TimeoutError(
                f"Homing Attained not set within 5s (6041h = 0x{sw:04X})"
            )

        _sdo(node, 0x6040).raw = 0x000F  # clear homing start bit

        pos_after = _sdo_read(node, 0x6064)
        offset_after = _sdo_read(node, 0x607C)
        click.echo("\nAfter homing:")
        click.echo(f"  Position Measured (6064h): {pos_after}")
        click.echo(f"  Home Offset       (607Ch): {offset_after}")

    finally:
        network.disconnect()


def limp(interface: str, node_id: int, eds_path: str | None) -> None:
    """Disable the drive so the motor turns freely.

    Writes ControlWord 0x0000 (Disable Voltage) and waits for the drive to
    reach Switch On Disabled state (no holding torque).
    """
    network = canopen.Network()
    network.connect(channel=interface, bustype="socketcan")
    try:
        node: canopen.RemoteNode = network.add_node(
            node_id, eds_path or _cia402_fallback_od()
        )
        logger.info("6040h = 0x0000 (Disable Voltage)")
        _sdo(node, 0x6040).raw = 0x0000
        # Switch On Disabled: bit 6 set, state bits 0-3 clear; ignore bit 5 (Quick Stop)
        _wait_status(node, mask=0x004F, expected=0x0040)
        click.echo("Motor is free (Switch On Disabled)")
    finally:
        network.disconnect()
