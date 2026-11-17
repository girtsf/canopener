"""Convert a pcap capture of SocketCAN traffic to candump -ta format.

Supports classic little-endian pcap files (not pcapng) with either of the two
link types tcpdump produces for CAN traffic:

- ``LINKTYPE_LINUX_SLL`` (113): ``tcpdump -i any``.  Each frame sent by the
  capturing host appears twice — once as the tx-queue copy (packet type
  ``PACKET_OUTGOING``) and once as the looped-back rx copy — so the tx-queue
  copies are dropped to match what ``candump`` would have shown.
- ``LINKTYPE_CAN_SOCKETCAN`` (227): ``tcpdump -i can0``.
"""

from __future__ import annotations

from collections.abc import Iterator
import pathlib
import struct

import can
from loguru import logger

from canopener import log_writer

_MAGIC_USEC = 0xA1B2C3D4
_MAGIC_NSEC = 0xA1B23C4D
_LINKTYPE_LINUX_SLL = 113
_LINKTYPE_CAN_SOCKETCAN = 227
_SLL_HEADER_LEN = 16
_SLL_PACKET_OUTGOING = 4
_SLL_PROTO_CAN = 0x000C
_SLL_PROTO_CANFD = 0x000D
_CAN_EFF_FLAG = 0x80000000
_CAN_RTR_FLAG = 0x40000000
_CAN_ERR_FLAG = 0x20000000


def read_pcap(pcap_path: str) -> Iterator[can.Message]:
    """Yield timestamped CAN messages from a pcap file.

    Args:
        pcap_path: Path to a classic little-endian pcap file with SLL v1 or
            raw SocketCAN link type.

    Yields:
        CAN messages in capture order, one per bus frame.
    """
    with open(pcap_path, "rb") as f:
        header = f.read(24)
        assert len(header) == 24, "truncated pcap file header"
        magic = struct.unpack("<I", header[:4])[0]
        if magic == _MAGIC_USEC:
            ts_divisor = 1e6
        elif magic == _MAGIC_NSEC:
            ts_divisor = 1e9
        else:
            raise ValueError(
                f"unsupported pcap magic 0x{magic:08X}; "
                "only classic little-endian pcap (not pcapng) is supported"
            )
        linktype = struct.unpack("<I", header[20:24])[0]
        if linktype not in (_LINKTYPE_LINUX_SLL, _LINKTYPE_CAN_SOCKETCAN):
            raise ValueError(
                f"unsupported pcap link type {linktype}; "
                "expected Linux SLL (113) or SocketCAN (227)"
            )

        while rec := f.read(16):
            assert len(rec) == 16, "truncated packet record header"
            ts_sec, ts_frac, incl_len, _orig_len = struct.unpack("<IIII", rec)
            pkt = f.read(incl_len)
            assert len(pkt) == incl_len, "truncated packet data"

            if linktype == _LINKTYPE_LINUX_SLL:
                packet_type = struct.unpack(">H", pkt[0:2])[0]
                proto = struct.unpack(">H", pkt[14:16])[0]
                if packet_type == _SLL_PACKET_OUTGOING:
                    continue  # tx-queue copy; the looped-back rx copy follows
                if proto == _SLL_PROTO_CANFD:
                    raise ValueError("CAN FD frames are not supported")
                if proto != _SLL_PROTO_CAN:
                    continue  # non-CAN traffic in an "-i any" capture
                frame = pkt[_SLL_HEADER_LEN:]
                # struct can_frame in the capturing host's byte order; the
                # little-endian pcap magic implies a little-endian host.
                can_id = struct.unpack("<I", frame[0:4])[0]
            else:
                frame = pkt
                # LINKTYPE_CAN_SOCKETCAN mandates big-endian CAN IDs.
                can_id = struct.unpack(">I", frame[0:4])[0]

            assert not can_id & _CAN_ERR_FLAG, "error frame in capture"
            assert not can_id & _CAN_RTR_FLAG, "RTR frame in capture"
            dlc = frame[4]
            assert dlc <= 8, f"bad DLC {dlc}"
            assert len(frame) >= 8 + dlc, "truncated CAN frame"
            is_extended = bool(can_id & _CAN_EFF_FLAG)
            yield can.Message(
                timestamp=ts_sec + ts_frac / ts_divisor,
                arbitration_id=can_id & (0x1FFFFFFF if is_extended else 0x7FF),
                is_extended_id=is_extended,
                data=frame[8 : 8 + dlc],
            )


def convert(pcap_path: str, output_path: str | None, interface: str) -> None:
    """Convert a pcap file to a candump -ta log file.

    Args:
        pcap_path: Input pcap file.
        output_path: Output log file; defaults to the input path with a
            ``.log`` suffix.
        interface: CAN interface name written into each line.
    """
    if output_path is None:
        output_path = str(pathlib.Path(pcap_path).with_suffix(".log"))
    count = 0
    with open(output_path, "w") as out:
        for msg in read_pcap(pcap_path):
            out.write(log_writer.format_line(msg, interface))
            count += 1
    logger.info(f"Wrote {count} frames to {output_path}")
