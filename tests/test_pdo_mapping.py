"""Tests for canopener.pdo_mapping.PdoMappingTracker."""

from __future__ import annotations

from canopener import pdo_mapping


def test_tpdo_register_and_decode_via_sdo_writes() -> None:
    """Mapping-then-comm SDO writes register a TPDO; payload decodes correctly."""
    tracker = pdo_mapping.PdoMappingTracker()

    # AMC FlexPro TPDO1 (comm 0x1800, map 0x1A00) on node 2:
    # Map: subindex 0 = 2 entries, then two 32-bit map words.
    #   entry 1: 0x60410010  -> 0x6041:00 (StatusWord), 16 bits
    #   entry 2: 0x60640020  -> 0x6064:00 (Position Actual Value), 32 bits
    tracker.handle_sdo_write(2, 0x1A00, 1, 0x60410010)
    tracker.handle_sdo_write(2, 0x1A00, 2, 0x60640020)
    # subindex 0 last so the rebuild sees a complete map.
    tracker.handle_sdo_write(2, 0x1A00, 0, 2)

    # Comm: subindex 1 = COB-ID (top bit clear = enabled).
    tracker.handle_sdo_write(2, 0x1800, 1, 0x182)

    # Mapping is registered.
    assert tracker.mapping_for(0x182) == [
        ("0x6041:00", 16),
        ("0x6064:00", 32),
    ]
    assert tracker.name_for(0x182) == "TPDO1"
    assert tracker.node_for(0x182) == 2

    # Decode a payload: status=0x0637 (16 bits LE), position=42 (32 bits LE).
    payload = (0x0637).to_bytes(2, "little") + (42).to_bytes(4, "little", signed=True)
    decoded = tracker.decode_payload(0x182, payload)
    assert decoded == [("0x6041:00", 0x0637), ("0x6064:00", 42)]


def test_disabled_pdo_does_not_register() -> None:
    """A COB-ID write with the disable bit set must not register a mapping."""
    tracker = pdo_mapping.PdoMappingTracker()

    tracker.handle_sdo_write(2, 0x1A00, 1, 0x60410010)
    tracker.handle_sdo_write(2, 0x1A00, 0, 1)
    # Disable bit (0x80000000) set -> rebuild should bail.
    tracker.handle_sdo_write(2, 0x1800, 1, 0x80000182)

    assert tracker.mapping_for(0x182) is None


def test_yaml_round_trip(tmp_path) -> None:
    """set_mapping persists to YAML; load_config restores state on a fresh tracker."""
    config_path = tmp_path / "pdo.yaml"

    src = pdo_mapping.PdoMappingTracker(config_path=config_path)
    src.set_mapping(
        0x182,
        [("StatusWord", 16), ("Actual Position", 32)],
        name="TPDO1",
        node_id=2,
    )
    assert config_path.exists()

    dst = pdo_mapping.PdoMappingTracker(config_path=config_path)
    dst.load_config()

    assert dst.mapping_for(0x182) == [("StatusWord", 16), ("Actual Position", 32)]
    assert dst.name_for(0x182) == "TPDO1"
    assert dst.node_for(0x182) == 2
