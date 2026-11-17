"""Tests for canopener.pcap2candump."""

from __future__ import annotations

import pathlib
import struct

import pytest

from canopener import pcap2candump

_SLL_PACKET_BROADCAST = 1
_SLL_PACKET_OUTGOING = 4
_SLL_PROTO_CAN = 0x000C


def _pcap_header(linktype: int) -> bytes:
    return struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 262144, linktype)


def _record(ts_sec: int, ts_usec: int, payload: bytes) -> bytes:
    return struct.pack("<IIII", ts_sec, ts_usec, len(payload), len(payload)) + payload


def _can_frame_le(can_id: int, data: bytes) -> bytes:
    return struct.pack("<IB3x", can_id, len(data)) + data.ljust(8, b"\x00")


def _sll_packet(packet_type: int, can_id: int, data: bytes) -> bytes:
    sll = struct.pack(">HHH8sH", packet_type, 280, 0, b"", _SLL_PROTO_CAN)
    return sll + _can_frame_le(can_id, data)


def test_sll_capture_drops_tx_queue_copies(tmp_path: pathlib.Path) -> None:
    pcap = tmp_path / "cap.pcap"
    pcap.write_bytes(
        _pcap_header(113)
        # Host-sent frame: tx-queue copy followed by looped-back rx copy.
        + _record(100, 1, _sll_packet(_SLL_PACKET_OUTGOING, 0x602, b"\x40\x00\x10\x00"))
        + _record(
            100, 2, _sll_packet(_SLL_PACKET_BROADCAST, 0x602, b"\x40\x00\x10\x00")
        )
        # Frame from another node: rx copy only.
        + _record(100, 500000, _sll_packet(_SLL_PACKET_BROADCAST, 0x582, b"\x43\x00"))
    )
    msgs = list(pcap2candump.read_pcap(str(pcap)))
    assert [(m.arbitration_id, bytes(m.data), m.timestamp) for m in msgs] == [
        (0x602, b"\x40\x00\x10\x00", 100.000002),
        (0x582, b"\x43\x00", 100.5),
    ]


def test_socketcan_linktype_big_endian_id(tmp_path: pathlib.Path) -> None:
    pcap = tmp_path / "cap.pcap"
    frame = (
        struct.pack(">I", 0x181)
        + struct.pack("<B3x", 2)
        + b"\x01\x02".ljust(8, b"\x00")
    )
    pcap.write_bytes(_pcap_header(227) + _record(7, 0, frame))
    (msg,) = pcap2candump.read_pcap(str(pcap))
    assert msg.arbitration_id == 0x181
    assert bytes(msg.data) == b"\x01\x02"
    assert not msg.is_extended_id


def test_extended_id(tmp_path: pathlib.Path) -> None:
    pcap = tmp_path / "cap.pcap"
    eff_id = 0x80000000 | 0x1ABCDEF0
    pcap.write_bytes(
        _pcap_header(113)
        + _record(1, 0, _sll_packet(_SLL_PACKET_BROADCAST, eff_id, b"\xaa"))
    )
    (msg,) = pcap2candump.read_pcap(str(pcap))
    assert msg.arbitration_id == 0x1ABCDEF0
    assert msg.is_extended_id


def test_convert_writes_candump_lines(tmp_path: pathlib.Path) -> None:
    pcap = tmp_path / "cap.pcap"
    pcap.write_bytes(
        _pcap_header(113)
        + _record(100, 2, _sll_packet(_SLL_PACKET_BROADCAST, 0x80, b""))
        + _record(100, 500000, _sll_packet(_SLL_PACKET_BROADCAST, 0x182, b"\x37\x02"))
    )
    pcap2candump.convert(str(pcap), None, "can1")
    assert (tmp_path / "cap.log").read_text() == (
        " (100.000002)  can1  080   [0]  \n (100.500000)  can1  182   [2]  37 02\n"
    )


def test_rejects_pcapng(tmp_path: pathlib.Path) -> None:
    pcap = tmp_path / "cap.pcapng"
    pcap.write_bytes(struct.pack("<I", 0x0A0D0D0A).ljust(24, b"\x00"))
    with pytest.raises(ValueError, match="magic"):
        list(pcap2candump.read_pcap(str(pcap)))


def test_rejects_error_frame(tmp_path: pathlib.Path) -> None:
    pcap = tmp_path / "cap.pcap"
    pcap.write_bytes(
        _pcap_header(113)
        + _record(1, 0, _sll_packet(_SLL_PACKET_BROADCAST, 0x20000001, b""))
    )
    with pytest.raises(AssertionError, match="error frame"):
        list(pcap2candump.read_pcap(str(pcap)))
