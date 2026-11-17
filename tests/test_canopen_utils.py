"""Tests for canopener.canopen_utils helpers."""

from __future__ import annotations

import canopen
from canopen import objectdictionary as od_mod

from canopener import canopen_utils


def _var(name: str, index: int, subindex: int, data_type: int) -> od_mod.ODVariable:
    var = od_mod.ODVariable(name, index, subindex)
    var.data_type = data_type
    var.access_type = "ro"
    return var


def test_od_variables_flattens_records_in_order() -> None:
    od = canopen.ObjectDictionary()
    record = od_mod.ODRecord("Identity", 0x1018)
    record.add_member(_var("Vendor-ID", 0x1018, 1, od_mod.UNSIGNED32))
    record.add_member(_var("Count", 0x1018, 0, od_mod.UNSIGNED8))
    od.add_object(record)
    od.add_object(_var("Device Type", 0x1000, 0, od_mod.UNSIGNED32))

    assert [
        (name, v.index, v.subindex) for name, v in canopen_utils._od_variables(od)
    ] == [
        ("Device Type", 0x1000, 0),
        ("Identity.Count", 0x1018, 0),
        ("Identity.Vendor-ID", 0x1018, 1),
    ]


def test_format_od_value_signed_int() -> None:
    var = _var("Vendor Value", 0x2000, 0, od_mod.INTEGER32)
    assert canopen_utils._format_od_value(var, b"\xff\xff\xff\xff") == (
        "0xFFFFFFFF (-1)"
    )


def test_format_od_value_string_and_domain() -> None:
    name = _var("Name", 0x1008, 0, od_mod.VISIBLE_STRING)
    assert canopen_utils._format_od_value(name, b"FlexPro") == "'FlexPro'"
    dom = _var("Blob", 0x2000, 0, od_mod.DOMAIN)
    assert canopen_utils._format_od_value(dom, b"\x01\xab") == "[01 ab]"


def test_format_od_value_size_mismatch_shows_raw() -> None:
    var = _var("Homing Speeds", 0x6099, 0, od_mod.UNSIGNED32)
    assert canopen_utils._format_od_value(var, b"\x02").startswith("[02]  (")


def test_format_od_value_annotates_statusword() -> None:
    var = _var("Statusword", 0x6041, 0, od_mod.UNSIGNED16)
    text = canopen_utils._format_od_value(var, b"\x37\x02")
    assert text.startswith("0x0237 (567)  [")
