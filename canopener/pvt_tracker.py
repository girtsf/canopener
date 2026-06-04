"""PVT command-stream replay state machine for ``canopener watch``.

Tracks the points being streamed into one drive's PVT buffer over RPDO 24,
then — once the 0x500 broadcast fires — replays them into a sink at the
wallclock time each segment is expected to *end* (i.e. when the drive
should have arrived at that point).  This gives us a "commanded" channel
aligned in time with the "actual" telemetry sampled around the same
moment, so they can be diff'd in post-hoc analysis.

Lifecycle (per node):
  add_point()  - decoded RPDO 24 frame
  clear()      - SDO write of 0 to 60C4.06h (buffer reset)
  start(t)     - 0x500 broadcast frame timestamp
  stop()       - shutdown the replay thread on session exit
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from loguru import logger

from canopener import sinks


@dataclass
class PvtPoint:
    """A single PVT command observed on the wire."""

    position: int
    velocity: int
    time_ms: int
    counter: int
    received_ts: float


class PvtTracker:
    """Per-node PVT command tracker + replayer thread."""

    def __init__(self, node_id: int, sink: sinks.SinkBus) -> None:
        self._node_id = node_id
        self._sink = sink
        self._cv = threading.Condition()
        self._queue: list[PvtPoint] = []
        self._expected_counter = 0
        self._t_start: float | None = None
        self._idx = 0
        self._cum_s = 0.0
        # Cumulative absolute commanded position. Snapshotted from the most
        # recent observed actual_position on start(), then incremented by each
        # PVT point's (relative) position delta as it's emitted.
        self._last_actual_position: int | None = None
        self._base_position = 0
        self._cum_delta = 0
        self._stopped = False
        self._thread = threading.Thread(
            target=self._replay, name=f"pvt-replay-{node_id}", daemon=True
        )
        self._thread.start()

    def update_actual_position(self, value: int) -> None:
        """Cache the most recent actual_position observed on the wire.

        Used by :meth:`start` to snapshot the absolute position the drive is at
        when motion begins, so commanded positions can be reported in the same
        encoder-counts frame as actual.
        """
        with self._cv:
            self._last_actual_position = value

    def add_point(
        self, position: int, velocity: int, time_ms: int, counter: int, ts: float
    ) -> None:
        """Enqueue a PVT command observed on RPDO 24."""
        with self._cv:
            if counter != self._expected_counter:
                logger.warning(
                    f"node {self._node_id}: PVT counter discontinuity: "
                    f"expected {self._expected_counter}, got {counter}"
                )
            self._expected_counter = (counter + 1) & 0xFF
            pt = PvtPoint(position, velocity, time_ms, counter, ts)
            logger.debug(f"{self}: adding {pt}")
            self._queue.append(pt)
            self._cv.notify_all()

    def clear(self) -> None:
        """Buffer cleared on the wire (SDO 60C4.06h <- 0).  Reset all state."""
        with self._cv:
            self._queue.clear()
            self._expected_counter = 0
            self._t_start = None
            self._idx = 0
            self._cum_s = 0.0
            self._base_position = 0
            self._cum_delta = 0
            self._cv.notify_all()

    def start(self, t_start: float) -> None:
        """0x500 broadcast frame received; begin replay against ``t_start``."""
        with self._cv:
            if self._t_start is not None:
                logger.warning(
                    f"node {self._node_id}: PVT start while already running; ignoring"
                )
                return
            self._t_start = t_start
            self._idx = 0
            self._cum_s = 0.0
            self._cum_delta = 0
            if self._last_actual_position is None:
                logger.warning(
                    f"node {self._node_id}: PVT start with no observed "
                    "actual_position; commanded position will start at 0"
                )
                self._base_position = 0
            else:
                self._base_position = self._last_actual_position
            self._cv.notify_all()

    def stop(self) -> None:
        """Shut down the replay thread.  Idempotent."""
        with self._cv:
            self._stopped = True
            self._cv.notify_all()
        self._thread.join(timeout=2.0)

    def _emit(self, ts: float, pt: PvtPoint, absolute_position: int) -> None:
        # Each PVT point describes the END of its segment (the position/velocity
        # the drive should have reached after time_ms ms of motion).  We log it
        # at that end-of-segment moment so the recorded commanded value can be
        # diff'd against the actual telemetry sampled around the same wallclock
        # time.  ``pvt_commanded.position`` is the absolute position in the same
        # encoder-counts frame as actual_position, computed as
        # ``base_position + cumulative_delta``; ``pvt_commanded.delta`` is the
        # raw 24-bit signed delta from the wire for debugging.
        self._sink.log(ts, "pvt_commanded.position", absolute_position)
        self._sink.log(ts, "pvt_commanded.delta", pt.position)
        self._sink.log(ts, "pvt_commanded.velocity", pt.velocity)
        self._sink.log(ts, "pvt_commanded.time_ms", pt.time_ms)
        self._sink.log(ts, "pvt_commanded.counter", pt.counter)
        logger.debug(f"{self}: PVT motion: {pt}")

    def _replay(self) -> None:
        while True:
            with self._cv:
                while not self._stopped and (
                    self._t_start is None or self._idx >= len(self._queue)
                ):
                    self._cv.wait(timeout=1.0)
                if self._stopped:
                    return
                assert self._t_start is not None
                pt = self._queue[self._idx]
                segment_end = self._t_start + self._cum_s + pt.time_ms / 1000.0
                absolute_position = self._base_position + self._cum_delta + pt.position
                run_id = (self._t_start, self._idx)

            # Wait (lock released) until segment_end, waking early on state change.
            while True:
                sleep_for = segment_end - time.time()
                if sleep_for <= 0:
                    break
                with self._cv:
                    if self._stopped or (self._t_start, self._idx) != run_id:
                        break
                    self._cv.wait(timeout=sleep_for)

            with self._cv:
                if self._stopped:
                    return
                if (self._t_start, self._idx) != run_id:
                    continue  # cleared / restarted during the wait

            self._emit(segment_end, pt, absolute_position)

            with self._cv:
                if (self._t_start, self._idx) != run_id:
                    continue
                self._idx += 1
                self._cum_s += pt.time_ms / 1000.0
                self._cum_delta += pt.position
                if pt.time_ms == 0:
                    # Stop point ends motion; reset to idle.
                    self._queue.clear()
                    self._idx = 0
                    self._cum_s = 0.0
                    self._base_position = 0
                    self._cum_delta = 0
                    self._t_start = None
                    self._expected_counter = 0
                    logger.debug(f"{self}: PVT motion tracking stopped")

    def __str__(self) -> str:
        return f"<PvtTracker node_id={self._node_id}>"
