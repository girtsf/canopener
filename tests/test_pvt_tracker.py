"""Tests for canopener.pvt_tracker.PvtTracker."""

from __future__ import annotations

import threading
import time

import pytest

from canopener import pvt_tracker
from canopener import sinks


class _RecordingSink:
    def __init__(self) -> None:
        self.events: list[tuple[float, str, object]] = []
        self._lock = threading.Lock()

    def log(self, ts: float, channel: str, value: object) -> None:
        with self._lock:
            self.events.append((ts, channel, value))

    def channel_values(self, channel: str) -> list[object]:
        with self._lock:
            return [v for _, c, v in self.events if c == channel]


@pytest.fixture
def sink() -> _RecordingSink:
    return _RecordingSink()


@pytest.fixture
def bus(sink: _RecordingSink) -> sinks.SinkBus:
    return sinks.SinkBus([sink])


def _wait_for(predicate, timeout: float = 2.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError(f"timeout waiting for {predicate}")


def test_replay_emits_at_end_of_segment(
    bus: sinks.SinkBus, sink: _RecordingSink
) -> None:
    tracker = pvt_tracker.PvtTracker(node_id=2, sink=bus)
    try:
        # Two motion points + stop point.  Use 30 ms / 30 ms / 0 ms so the
        # whole replay completes in <100 ms.  Snapshot a base position before
        # start so absolute commanded positions are based off 1000.
        t0 = time.time()
        tracker.update_actual_position(1000)
        tracker.add_point(position=100, velocity=10, time_ms=30, counter=0, ts=t0)
        tracker.add_point(position=200, velocity=0, time_ms=30, counter=1, ts=t0)
        tracker.add_point(position=0, velocity=0, time_ms=0, counter=2, ts=t0)
        tracker.start(t0)

        # Wait for all three points to be emitted (5 events each = 15).
        _wait_for(lambda: len(sink.events) >= 15, timeout=2.0)
    finally:
        tracker.stop()

    positions = sink.channel_values("pvt_commanded.position")
    deltas = sink.channel_values("pvt_commanded.delta")
    velocities = sink.channel_values("pvt_commanded.velocity")
    counters = sink.channel_values("pvt_commanded.counter")
    times_ms = sink.channel_values("pvt_commanded.time_ms")
    # base 1000 + cumulative deltas: 100, 300, 300
    assert positions == [1100, 1300, 1300]
    assert deltas == [100, 200, 0]
    assert velocities == [10, 0, 0]
    assert counters == [0, 1, 2]
    assert times_ms == [30, 30, 0]

    # First emission lands at ~t0 + 30ms; second ~t0 + 60ms.  Allow slack.
    pos_events = [(ts, v) for ts, c, v in sink.events if c == "pvt_commanded.position"]
    assert pos_events[0][0] == pytest.approx(t0 + 0.030, abs=0.005)
    assert pos_events[1][0] == pytest.approx(t0 + 0.060, abs=0.005)


def test_clear_drops_pending_points(bus: sinks.SinkBus, sink: _RecordingSink) -> None:
    tracker = pvt_tracker.PvtTracker(node_id=2, sink=bus)
    try:
        # Far-future segment so the replayer is asleep.
        t0 = time.time()
        tracker.add_point(position=1, velocity=1, time_ms=200, counter=0, ts=t0)
        tracker.start(t0)
        # Let the replayer pick up the work item and start sleeping.
        time.sleep(0.02)
        tracker.clear()
        # After clear, even waiting past the original segment end should produce
        # nothing.
        time.sleep(0.25)
        assert sink.events == []
    finally:
        tracker.stop()


def test_counter_discontinuity_resyncs(
    bus: sinks.SinkBus, sink: _RecordingSink
) -> None:
    tracker = pvt_tracker.PvtTracker(node_id=2, sink=bus)
    try:
        t0 = time.time()
        tracker.update_actual_position(0)
        # counter jumps from expected 0 to 5; tracker must still accept it
        # and resync to expecting counter=6 next.
        tracker.add_point(position=1, velocity=0, time_ms=0, counter=5, ts=t0)
        tracker.start(t0)
        _wait_for(lambda: len(sink.events) >= 5)
    finally:
        tracker.stop()
    counters = sink.channel_values("pvt_commanded.counter")
    assert counters == [5]


def test_start_before_points(bus: sinks.SinkBus, sink: _RecordingSink) -> None:
    tracker = pvt_tracker.PvtTracker(node_id=2, sink=bus)
    try:
        t0 = time.time()
        tracker.update_actual_position(500)
        tracker.start(t0)
        # Replayer is now blocked waiting for points.
        time.sleep(0.02)
        tracker.add_point(position=42, velocity=0, time_ms=0, counter=0, ts=t0)
        _wait_for(lambda: len(sink.events) >= 5)
    finally:
        tracker.stop()
    # base 500 + delta 42 = 542
    assert sink.channel_values("pvt_commanded.position") == [542]
    assert sink.channel_values("pvt_commanded.delta") == [42]


def test_stop_point_ends_motion_and_allows_restart(
    bus: sinks.SinkBus, sink: _RecordingSink
) -> None:
    tracker = pvt_tracker.PvtTracker(node_id=2, sink=bus)
    try:
        t0 = time.time()
        # First move: drive at 100, single +10 stop point -> absolute 110.
        tracker.update_actual_position(100)
        tracker.add_point(position=10, velocity=0, time_ms=0, counter=0, ts=t0)
        tracker.start(t0)
        _wait_for(lambda: len(sink.events) >= 5)

        # After stop point, state should be idle; counter expects 0 again.
        # New move with counter=0 should be accepted without warning.  Drive
        # has moved to 110; second move +20 -> absolute 130.
        t1 = time.time()
        tracker.update_actual_position(110)
        tracker.add_point(position=20, velocity=0, time_ms=0, counter=0, ts=t1)
        tracker.start(t1)
        _wait_for(lambda: len(sink.events) >= 10)
    finally:
        tracker.stop()
    assert sink.channel_values("pvt_commanded.position") == [110, 130]


def test_stop_is_idempotent(bus: sinks.SinkBus) -> None:
    tracker = pvt_tracker.PvtTracker(node_id=2, sink=bus)
    tracker.stop()
    tracker.stop()  # must not raise
