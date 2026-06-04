"""Tests for canopener.decode.expand_to_fields."""

from __future__ import annotations

import math

import pytest

from canopener import decode


def test_pvt_status_executing_and_buf_empty() -> None:
    # bit 1 (buf_empty) + bit 6 (executing) = 0x42
    out = decode.expand_to_fields("PVT Status", 0x42)
    assert out == {
        "pvt_status.raw": 0x42,
        "pvt_status.buf_full": 0,
        "pvt_status.buf_empty": 1,
        "pvt_status.buf_threshold": 0,
        "pvt_status.buf_failure": 0,
        "pvt_status.buf_empty_stop": 0,
        "pvt_status.seq_error": 0,
        "pvt_status.executing": 1,
    }


def test_pvt_quick_status_pts_and_flags() -> None:
    # pts=8 (bits 0-4 = 0b01000), zero_speed (bit 8), pvt_executing (bit 15) = 0x8108
    out = decode.expand_to_fields("PVT Quick Status", 0x8108)
    assert out == {
        "pvt_quick_status.raw": 0x8108,
        "pvt_quick_status.pts": 8,
        "pvt_quick_status.zero_speed": 1,
        "pvt_quick_status.at_command": 0,
        "pvt_quick_status.homing_active": 0,
        "pvt_quick_status.homing_complete": 0,
        "pvt_quick_status.bridge_enabled": 0,
        "pvt_quick_status.brake_enabled": 0,
        "pvt_quick_status.stop": 0,
        "pvt_quick_status.pvt_executing": 1,
    }


def test_statusword_operation_enabled_and_target_reached() -> None:
    # 0x0637: state bits = 0x27 (Operation enabled), bit 4 (VoltageEnabled),
    # bit 10 (TargetReached), bits 0,1,2,5 set.
    out = decode.expand_to_fields("StatusWord", 0x0637)
    assert out["statusword.raw"] == 0x0637
    assert out["statusword.state"] == "Operation enabled"
    assert out["statusword.voltageenabled"] == 1
    assert out["statusword.targetreached"] == 1
    assert out["statusword.warning"] == 0
    assert out["statusword.internallimitactive"] == 0
    assert out["statusword.homingcomplete"] == 0


def test_statusword_fault_state() -> None:
    # 0x0008: bit 3 (Fault) set, no other state bits.
    out = decode.expand_to_fields("StatusWord", 0x0008)
    assert out["statusword.state"] == "Fault"


def test_controlword_enable_operation() -> None:
    out = decode.expand_to_fields("ControlWord", 0x000F)
    assert out == {
        "controlword.raw": 0x000F,
        "controlword.cmd": "Enable operation",
    }


def test_actual_position_negative() -> None:
    # 0xFFFFFFFF == -1 as signed INT32
    out = decode.expand_to_fields("Actual Position", 0xFFFFFFFF)
    assert out == {"actual_position": -1}


def test_actual_position_positive() -> None:
    out = decode.expand_to_fields("Actual Position", 12345)
    assert out == {"actual_position": 12345}


def test_actual_velocity_scaled() -> None:
    # value = -131072 -> -131072 / (2^17 / 20000) = -20000.0 counts/s
    raw = 0x100000000 - 131072  # 0xFFFE0000
    out = decode.expand_to_fields("Actual Velocity", raw)
    assert isinstance(out["actual_velocity"], float)
    velocity_value = out["actual_velocity"]
    assert isinstance(velocity_value, float)
    assert math.isclose(velocity_value, -20000.0)


def test_actual_current_scaled() -> None:
    # value = -1 -> -1 / (2^14 / 12) ≈ -0.000732
    out = decode.expand_to_fields("Actual Current", 0xFFFF)
    velocity_value = out["actual_current"]
    assert isinstance(velocity_value, float)
    assert math.isclose(velocity_value, -1.0 / (2**14 / 12))


def test_modes_of_operation_signed() -> None:
    out = decode.expand_to_fields("Modes of Operation", 0xFF)
    assert out == {"modes_of_operation": -1}


def test_modes_of_operation_display_positive() -> None:
    out = decode.expand_to_fields("Modes of Operation Display", 7)
    assert out == {"modes_of_operation_display": 7}


def test_target_position_signed() -> None:
    out = decode.expand_to_fields("Target Position", 0xFFFFFFFE)
    assert out == {"target_position": -2}


def test_dotted_record_uses_leaf() -> None:
    # Record-prefixed names should use only the leaf for channel base.
    out = decode.expand_to_fields("Foo.Actual Position", 12345)
    assert out == {"actual_position": 12345}


def test_unknown_signal_passthrough() -> None:
    out = decode.expand_to_fields("Some Unknown Signal", 42)
    assert out == {"some_unknown_signal": 42}


def test_unknown_signal_zero() -> None:
    out = decode.expand_to_fields("MyField", 0)
    assert out == {"myfield": 0}


@pytest.mark.parametrize(
    "raw,buf_threshold",
    [
        (0x00, 0),
        (0x04, 1),
    ],
)
def test_pvt_status_buf_threshold_bit(raw: int, buf_threshold: int) -> None:
    out = decode.expand_to_fields("PVT Status", raw)
    assert out["pvt_status.buf_threshold"] == buf_threshold


def test_expand_emcy_amc_pvt_buffer_underflow() -> None:
    # AMC layout: bytes 0-2 = 0, byte 3 = AMC code (0x02 = PVT Buffer Underflow)
    out = decode.expand_emcy(bytes([0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00]))
    assert out["emcy.error_code"] == 0
    assert out["emcy.error_register"] == 0
    assert out["emcy.amc_code"] == 0x02
    text = out["emcy.text"]
    assert isinstance(text, str)
    assert "PVT Buffer Underflow" in text


def test_expand_emcy_cia_overcurrent() -> None:
    # CiA: error_code = 0x2100 (Current device input), reg = 0x02 (Current)
    out = decode.expand_emcy(bytes([0x00, 0x21, 0x02, 0xAB, 0xCD, 0x00, 0x00, 0x00]))
    assert out["emcy.error_code"] == 0x2100
    assert out["emcy.error_register"] == 0x02
    assert out["emcy.amc_code"] == 0xAB
    text = out["emcy.text"]
    assert isinstance(text, str)
    assert "Current" in text


def test_expand_emcy_short_frame() -> None:
    out = decode.expand_emcy(bytes([0x00]))
    # Only raw should be present; no over-eager parsing.
    assert "emcy.error_code" not in out
    assert out["emcy.raw"] == "00"
