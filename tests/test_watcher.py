"""Tests for canopener.watcher.WatchProcessor."""

from __future__ import annotations

import threading
import time

import can
import pytest

from canopener import sinks
from canopener import watcher


class _RecordingSink:
    def __init__(self) -> None:
        self.events: list[tuple[float, str, object]] = []
        self._lock = threading.Lock()

    def log(self, ts: float, channel: str, value: object) -> None:
        with self._lock:
            self.events.append((ts, channel, value))


@pytest.fixture
def cfg() -> watcher.WatchConfig:
    return watcher.WatchConfig(node_ids=[2])


@pytest.fixture
def setup(cfg: watcher.WatchConfig):
    """Build a wired WatchProcessor with a recording sink."""
    sink = _RecordingSink()
    bus = sinks.SinkBus([sink])
    with watcher.WatchProcessor(config=cfg, sink_buses={2: bus}) as proc:
        yield sink, proc.decoder, proc.trackers[2], proc


def _msg(arb_id: int, data: bytes) -> can.Message:
    return can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)


def test_pvt_start_broadcast_starts_tracker(setup) -> None:
    sink, _decoder, tracker, proc = setup
    # First, queue a stop point so the tracker has something to emit on start.
    tracker.update_actual_position(1000)
    tracker.add_point(position=42, velocity=0, time_ms=0, counter=0, ts=time.time())

    proc.process(_msg(0x500, bytes(8)))

    deadline = time.time() + 1.0
    while time.time() < deadline and len(sink.events) < 5:
        time.sleep(0.005)
    # base 1000 + delta 42 = 1042
    assert any(c == "pvt_commanded.position" and v == 1042 for _, c, v in sink.events)
    assert any(c == "pvt_commanded.delta" and v == 42 for _, c, v in sink.events)


def test_pvt_rpdo_decoded_into_tracker(setup) -> None:
    sink, _decoder, tracker, proc = setup
    tracker.update_actual_position(0)
    # PVT point: pos=100 (0x000064), vel=0, time_ms=0 (stop point), counter=0.
    payload = (
        (100).to_bytes(3, "little", signed=True)
        + (0).to_bytes(3, "little", signed=True)
        + bytes([0, 0])
    )
    # Default RPDO COB-ID for node 2 is 0x502.
    proc.process(_msg(0x502, payload))
    # Then trigger start so the queued point gets emitted.
    proc.process(_msg(0x500, bytes(8)))

    deadline = time.time() + 1.0
    while time.time() < deadline and len(sink.events) < 5:
        time.sleep(0.005)
    pos_events = [v for _, c, v in sink.events if c == "pvt_commanded.position"]
    assert pos_events == [100]


def test_pvt_rpdo_negative_position(setup) -> None:
    sink, _decoder, tracker, proc = setup
    tracker.update_actual_position(0)
    # pos = -50, vel = 0, time = 0, counter = 0
    payload = (
        (-50).to_bytes(3, "little", signed=True)
        + (0).to_bytes(3, "little", signed=True)
        + bytes([0, 0])
    )
    proc.process(_msg(0x502, payload))
    proc.process(_msg(0x500, bytes(8)))

    deadline = time.time() + 1.0
    while time.time() < deadline and len(sink.events) < 5:
        time.sleep(0.005)
    assert [v for _, c, v in sink.events if c == "pvt_commanded.position"] == [-50]


def test_pvt_rpdo_cob_id_routing_tolerates_stale_mapping(setup) -> None:
    """A stale mapping (e.g. an old C++ run's 0x402 still in YAML) must not
    block routing of frames on the currently active COB-ID."""
    sink, decoder, tracker, proc = setup
    tracker.update_actual_position(0)

    # Stale entry for the same name+node from a prior run.
    decoder.pdo.set_mapping(
        cob_id=0x402,
        signals=[("interpolation_data_record.setpoint_1", 64)],
        name="RPDO24",
        node_id=2,
    )
    # Currently-active mapping (e.g. just registered by SDO).
    decoder.pdo.set_mapping(
        cob_id=0x502,
        signals=[("interpolation_data_record.setpoint_1", 64)],
        name="RPDO24",
        node_id=2,
    )

    payload = (
        (100).to_bytes(3, "little", signed=True)
        + (0).to_bytes(3, "little", signed=True)
        + bytes([0, 0])
    )
    proc.process(_msg(0x502, payload))
    proc.process(_msg(0x500, bytes(8)))

    deadline = time.time() + 1.0
    while time.time() < deadline and len(sink.events) < 5:
        time.sleep(0.005)
    pos_events = [v for _, c, v in sink.events if c == "pvt_commanded.position"]
    assert pos_events == [100]


def test_pvt_rpdo_cob_id_learned_from_pdo_mapping(setup) -> None:
    """A non-default RPDO 24 mapping (e.g. 0x402 from a C++ client) should
    re-route the per-node PVT-data lookup via PdoMappingTracker so PVT frames
    still reach the tracker."""
    sink, decoder, tracker, proc = setup
    tracker.update_actual_position(0)

    # Simulate the result of the C++ client's RPDO 24 reconfiguration.
    decoder.pdo.set_mapping(
        cob_id=0x402,
        signals=[("interpolation_data_record.setpoint_1", 64)],
        name="RPDO24",
        node_id=2,
    )

    # PVT data on the new COB-ID 0x402 should now reach the tracker.
    payload = (
        (100).to_bytes(3, "little", signed=True)
        + (0).to_bytes(3, "little", signed=True)
        + bytes([0, 0])
    )
    proc.process(_msg(0x402, payload))
    proc.process(_msg(0x500, bytes(8)))

    deadline = time.time() + 1.0
    while time.time() < deadline and len(sink.events) < 5:
        time.sleep(0.005)
    pos_events = [v for _, c, v in sink.events if c == "pvt_commanded.position"]
    assert pos_events == [100]


def test_buffer_clear_via_sdo_write(setup) -> None:
    sink, _decoder, tracker, proc = setup

    # Enqueue a point; mid-flight clear should drop it.
    tracker.add_point(position=99, velocity=0, time_ms=200, counter=0, ts=time.time())
    proc.process(_msg(0x500, bytes(8)))
    time.sleep(0.02)

    # SDO download to node 2 (cob 0x602), 60C4.06h <- 0:
    # cmd=0x23 (expedited write, 4 bytes data), index=60C4, subindex=06, value=0
    sdo = bytes([0x23, 0xC4, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00])
    proc.process(_msg(0x602, sdo))

    time.sleep(0.25)
    assert sink.events == []


def test_pdo_signals_fanned_out(setup) -> None:
    sink, decoder, _tracker, proc = setup
    # Pre-register a PDO mapping: TPDO on cob 0x182 with two signals.
    decoder.pdo.set_mapping(0x182, [("Actual Position", 32), ("StatusWord", 16)])

    # Build payload: actual_position = 42 (4 bytes LE), statusword = 0x0637 (2 bytes LE)
    payload = (42).to_bytes(4, "little", signed=True) + (0x0637).to_bytes(2, "little")
    proc.process(_msg(0x182, payload + b"\x00\x00"))  # pad to 8 bytes

    channels = {c: v for _, c, v in sink.events}
    assert channels["actual_position"] == 42
    assert channels["statusword.raw"] == 0x0637
    assert channels["statusword.state"] == "Operation enabled"
    assert channels["statusword.targetreached"] == 1


def test_emcy_emitted_to_node_sink(setup) -> None:
    sink, _decoder, _tracker, proc = setup
    # AMC PVT Buffer Underflow on node 2 -> COB-ID 0x082.
    proc.process(_msg(0x082, bytes([0x00, 0x00, 0x00, 0x02, 0, 0, 0, 0])))

    channels = {c: v for _, c, v in sink.events}
    assert channels["emcy.amc_code"] == 0x02
    assert channels["emcy.error_code"] == 0
    text = channels["emcy.text"]
    assert isinstance(text, str)
    assert "PVT Buffer Underflow" in text


def test_emcy_for_unconfigured_node_ignored(cfg: watcher.WatchConfig) -> None:
    sink = _RecordingSink()
    bus = sinks.SinkBus([sink])
    with watcher.WatchProcessor(cfg, {2: bus}) as proc:
        # EMCY for node 3 (not configured) -> COB-ID 0x083.
        proc.process(_msg(0x083, bytes([0x00, 0x21, 0x02, 0, 0, 0, 0, 0])))
        assert sink.events == []


def test_unconfigured_node_ignored(cfg: watcher.WatchConfig) -> None:
    sink = _RecordingSink()
    bus = sinks.SinkBus([sink])
    with watcher.WatchProcessor(cfg, {2: bus}) as proc:
        # Mapping for cob 0x183 (node 3 — not configured)
        proc.decoder.pdo.set_mapping(0x183, [("Actual Position", 32)])
        proc.process(_msg(0x183, (42).to_bytes(8, "little")))
        assert sink.events == []


def test_on_signal_fires_for_pdo_fields(setup) -> None:
    _sink, decoder, _tracker, proc = setup
    decoder.pdo.set_mapping(0x182, [("Actual Position", 32), ("StatusWord", 16)])
    seen: list[tuple[int, str, object]] = []
    proc.on_signal = lambda nid, ch, v: seen.append((nid, ch, v))

    payload = (42).to_bytes(4, "little", signed=True) + (0x0637).to_bytes(2, "little")
    proc.process(_msg(0x182, payload + b"\x00\x00"))

    channels = {ch: v for _, ch, v in seen}
    assert channels["actual_position"] == 42
    assert channels["statusword.state"] == "Operation enabled"
    assert all(nid == 2 for nid, _, _ in seen)


def test_on_signal_and_on_emcy_fire_for_emcy(setup) -> None:
    _sink, _decoder, _tracker, proc = setup
    signal_events: list[tuple[int, str, object]] = []
    emcy_events: list[tuple[int, dict[str, object]]] = []
    proc.on_signal = lambda nid, ch, v: signal_events.append((nid, ch, v))
    proc.on_emcy = lambda nid, fields: emcy_events.append((nid, fields))

    proc.process(_msg(0x082, bytes([0x00, 0x00, 0x00, 0x02, 0, 0, 0, 0])))

    sig_channels = {ch: v for _, ch, v in signal_events}
    assert sig_channels["emcy.amc_code"] == 0x02
    assert all(nid == 2 for nid, _, _ in signal_events)
    assert len(emcy_events) == 1
    nid, fields = emcy_events[0]
    assert nid == 2
    assert fields["emcy.amc_code"] == 0x02
