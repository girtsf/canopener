"""AMC FlexPro PVT mode runners (called from cli.py).

Generates a trapezoidal motion profile split into three speed windows
(acceleration, cruise, deceleration) and executes it via the AMC
PVT (Interpolated Position Mode) protocol over CANopen.

Protocol notes (AMC-specific, deviates from CiA 402):
  - PVT data goes on RPDO 24 (1417h), not a user-selectable RPDO.
  - Motion starts via a broadcast on COB-ID 0x500, not via ControlWord.
  - Every point carries an 8-bit integrity counter; must increment by 1.
  - Stop requires two points: penultimate (vel=0, T>0) + stop (vel=0, T=0).
  - Drive must be at zero velocity when the 0x500 broadcast is sent.
"""

from __future__ import annotations

import contextlib
import pathlib
import threading
import time
from typing import NamedTuple

import canopen
import canopen.objectdictionary
import click
from loguru import logger

from canopener import canopen_utils
from canopener import watcher


_BUFFER_SIZE = 15  # AMC fixed FIFO depth


# ---------------------------------------------------------------------------
# PVT point type and encoding
# ---------------------------------------------------------------------------


class PvtPoint(NamedTuple):
    """A single PVT trajectory knot point."""

    position: int  # encoder counts (absolute)
    velocity: int  # counts/second
    time_ms: int  # segment duration in milliseconds; 0 = stop point


def _pack24(value: int) -> bytes:
    """Pack a signed integer as 24-bit little-endian two's complement."""
    v = value & 0xFFFFFF
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])


def encode_pvt(point: PvtPoint, counter: int) -> bytes:
    """Encode a PVT point as an 8-byte PDO payload.

    Wire format: [P_LSB P_MID P_MSB V_LSB V_MID V_MSB Time Counter]

    Args:
        point: PVT trajectory point.
        counter: Integrity counter (0–255, wraps at 256).

    Returns:
        8-byte PDO data.
    """
    if not (-8_388_607 <= point.position <= 8_388_607):
        raise ValueError(
            f"Position {point.position} exceeds 24-bit signed range ±8,388,607"
        )
    if not (-8_388_607 <= point.velocity <= 8_388_607):
        raise ValueError(
            f"Velocity {point.velocity} out of 24-bit signed range ±8,388,607"
        )
    return (
        _pack24(point.position)
        + _pack24(point.velocity)
        + bytes([point.time_ms & 0xFF, counter & 0xFF])
    )


# ---------------------------------------------------------------------------
# Trajectory generation
# ---------------------------------------------------------------------------


def generate_trapezoid(
    from_pos: int,
    to_pos: int,
    total_ms: int,
    segment_ms: int,
    accel_frac: float,
    decel_frac: float,
) -> list[PvtPoint]:
    """Generate a trapezoidal velocity PVT trajectory.

    Three speed windows:
    - Acceleration: velocity ramps linearly 0 → v_peak
    - Cruise: constant v_peak
    - Deceleration: velocity ramps linearly v_peak → 0

    The peak velocity is computed from the position / time constraint so
    the drive arrives at ``to_pos`` exactly at ``total_ms``.

    The returned list always ends with a two-point stop sequence as required
    by AMC: the penultimate point arrives at the target with vel=0 and a
    normal segment time; the stop point has the same position, vel=0, time=0.

    Args:
        from_pos: Starting position in encoder counts.
        to_pos: Target position in encoder counts.
        total_ms: Total move duration in milliseconds.
        segment_ms: Duration of each PVT segment (2–255 ms).
        accel_frac: Fraction of total time for the acceleration phase (0–1).
        decel_frac: Fraction of total time for the deceleration phase (0–1).

    Returns:
        List of PvtPoint including the trailing stop point.

    Raises:
        ValueError: If the profile parameters are out of range.
    """
    delta_counts = to_pos - from_pos
    cruise_frac = 1.0 - accel_frac - decel_frac
    if cruise_frac < 0.0:
        raise ValueError(f"accel_frac ({accel_frac}) + decel_frac ({decel_frac}) > 1.0")
    if segment_ms < 2 or segment_ms > 255:
        raise ValueError(f"segment_ms {segment_ms} not in [2, 255]")

    T = total_ms / 1000.0  # seconds
    t_a = accel_frac * T  # end of acceleration phase
    t_c = t_a + cruise_frac * T  # end of cruise phase

    # Peak velocity from area-under-curve = delta_counts
    # area = 0.5*v*t_a + v*(t_c - t_a) + 0.5*v*(T - t_c)
    #      = v * T * (1 - 0.5*accel_frac - 0.5*decel_frac)
    denom = 1.0 - 0.5 * accel_frac - 0.5 * decel_frac
    v_peak = (delta_counts / T) / denom
    logger.info(
        f"Peak velocity: {v_peak:.1f} counts/s ({v_peak * 60 / 4096:.1f} RPM @ 4096 cnt/rev)"
    )

    n_segs = round(total_ms / segment_ms)
    if n_segs < 2:
        raise ValueError("total_ms / segment_ms must give at least 2 segments")

    p_at_ta = 0.5 * v_peak * t_a  # position at end of acceleration

    def _vel_at(t: float) -> float:
        if t <= t_a:
            return v_peak * (t / t_a) if t_a > 0 else v_peak
        if t <= t_c:
            return v_peak
        t_d = T - t_c
        return v_peak * (1.0 - (t - t_c) / t_d) if t_d > 0 else 0.0

    def _pos_at(t: float) -> float:
        if t <= t_a:
            return 0.5 * v_peak * t**2 / t_a if t_a > 0 else 0.0
        if t <= t_c:
            return p_at_ta + v_peak * (t - t_a)
        p_at_tc = p_at_ta + v_peak * (t_c - t_a)
        dt = t - t_c
        t_d = T - t_c
        return p_at_tc + v_peak * dt - (0.5 * v_peak * dt**2 / t_d if t_d > 0 else 0.0)

    points: list[PvtPoint] = []
    for i in range(1, n_segs + 1):
        t = min(i * segment_ms / 1000.0, T)
        pos = int(round(_pos_at(t))) + from_pos
        vel = int(round(_vel_at(t)))

        # Force exact end-point values at the final motion segment
        if i == n_segs:
            pos = to_pos
            vel = 0

        points.append(PvtPoint(pos, vel, segment_ms))

    # Stop point: same final position, zero velocity, time = 0
    points.append(PvtPoint(to_pos, 0, 0))

    return points


def print_trajectory(points: list[PvtPoint], segment_ms: int) -> None:
    """Print a formatted trajectory table to stdout.

    Args:
        points: PVT points including the final stop point.
        segment_ms: Duration of each motion segment in milliseconds.
    """
    n_motion = len(points) - 1  # exclude stop point
    click.echo(
        f"\nTrajectory: {n_motion} motion segments × {segment_ms} ms"
        f" + 1 stop point  (total {len(points)} points)\n"
    )
    click.echo(
        f"{'#':>4}  {'Position':>12}  {'Velocity':>10}  {'Time ms':>8}  {'Note'}"
    )
    click.echo("-" * 58)
    for i, pt in enumerate(points):
        note = ""
        if i == 0:
            note = "start"
        elif i == n_motion - 1 and pt.velocity == 0:
            note = "penultimate (vel=0)"
        elif i == n_motion:
            note = "STOP (time=0)"
        click.echo(
            f"{i:>4}  {pt.position:>12}  {pt.velocity:>10}  {pt.time_ms:>8}  {note}"
        )
    click.echo()


# ---------------------------------------------------------------------------
# Drive control helpers
# ---------------------------------------------------------------------------


def setup_pvt(
    node: canopen.RemoteNode,
    rpdo_cob_id: int,
    tpdo_cob_id: int,
    threshold: int,
    start_cob_id: int,
    extra_tpdo_enables: dict[int, int] | None = None,
    pvt_status_cob_id: int | None = None,
) -> None:
    """Configure the drive for PVT mode.

    Goes pre-operational to configure PDO COB-IDs, then returns to
    operational, sets PVT mode, configures buffer parameters, and clears
    the buffer.  Position mode is set to incremental (deltas).

    Args:
        node: Remote CANopen node.
        rpdo_cob_id: COB-ID for RPDO 24 (PVT data; we transmit this).
        tpdo_cob_id: COB-ID for TPDO 24 (buffer-low notification; drive transmits).
        threshold: Buffer depth at which TPDO 24 fires (1–14).
        start_cob_id: COB-ID for RPDO 26 (PVT start broadcast → 0x2047.02).
        extra_tpdo_enables: Optional mapping of comm object index → COB-ID for
            additional TPDOs to enable during the pre-operational phase.
        pvt_status_cob_id: If provided, configure TPDO 26 to transmit PVT Status
            (0x201D.01h) on every SYNC using this COB-ID.
    """
    logger.info("NMT: pre-operational (for PDO configuration)")
    node.nmt.state = "PRE-OPERATIONAL"
    time.sleep(0.1)

    logger.info(f"RPDO 24 (1417.01h) COB-ID = 0x{rpdo_cob_id:03X}")
    canopen_utils._sdo(node, 0x1417, 0x01).raw = rpdo_cob_id  # bit 31 = 0 → enabled
    canopen_utils._sdo(node, 0x1417, 0x02).raw = 0xFF  # asynchronous

    logger.info(f"RPDO 26 (1419.01h) COB-ID = 0x{start_cob_id:03X} (PVT start)")
    canopen_utils._sdo(node, 0x1419, 0x01).raw = start_cob_id  # bit 31 = 0 → enabled
    canopen_utils._sdo(node, 0x1419, 0x02).raw = 0xFF  # asynchronous

    logger.info(f"TPDO 24 (1817.01h) COB-ID = 0x{tpdo_cob_id:03X}")
    canopen_utils._sdo(node, 0x1817, 0x01).raw = tpdo_cob_id  # bit 31 = 0 → enabled
    canopen_utils._sdo(node, 0x1817, 0x02).raw = 0xFF  # event-driven (asynchronous)
    tx_type = canopen_utils._sdo_read(node, 0x1817, 0x02)
    logger.info(f"TPDO 24 transmission type readback: 0x{tx_type:02X}")

    for comm_idx, cob_id in (extra_tpdo_enables or {}).items():
        logger.info(
            f"TPDO 0x{comm_idx:04X}.01h COB-ID = 0x{cob_id:03X} (enabling, sync)"
        )
        canopen_utils._sdo(node, comm_idx, 0x01).raw = cob_id
        canopen_utils._sdo(node, comm_idx, 0x02).raw = 0x01  # transmit on every SYNC

    if pvt_status_cob_id is not None:
        logger.info(
            f"TPDO 26 (1819h) COB-ID = 0x{pvt_status_cob_id:03X}, "
            f"mapping = 201D.01h (PVT Status) + 200C.01h (PVT Quick Status), sync"
        )
        canopen_utils._sdo(
            node, 0x1A19, 0x00
        ).raw = 0  # disable all mappings before changing
        canopen_utils._sdo(
            node, 0x1A19, 0x01
        ).raw = 0x201D0110  # PVT Status, sub 1, 16 bits
        canopen_utils._sdo(
            node, 0x1A19, 0x02
        ).raw = 0x200C0110  # PVT Quick Status, sub 1, 16 bits
        canopen_utils._sdo(node, 0x1A19, 0x00).raw = 2  # enable 2 mapped objects
        canopen_utils._sdo(node, 0x1819, 0x02).raw = 0x01  # transmit on every SYNC
        canopen_utils._sdo(
            node, 0x1819, 0x01
        ).raw = pvt_status_cob_id  # enable (bit 31 = 0)

    logger.info("NMT: operational")
    node.nmt.state = "OPERATIONAL"
    time.sleep(0.1)

    logger.info("6060h = 7 (Interpolated Position / PVT mode)")
    canopen_utils._sdo(node, 0x6060).raw = 7
    time.sleep(0.1)
    display = canopen_utils._sdo_read(node, 0x6061)
    if display != 7:
        raise RuntimeError(f"Mode display (6061h) returned {display}, expected 7")
    logger.info("PVT mode confirmed via 6061h")

    logger.info(f"2048.01h = {threshold} (buffer threshold)")
    canopen_utils._sdo(node, 0x2048, 0x01).raw = threshold

    logger.info("2048.02h = 1 (incremental position)")
    canopen_utils._sdo(node, 0x2048, 0x02).raw = 1

    logger.info("60C4.06h = 0 (clear PVT buffer; counter restarts at 0x00)")
    canopen_utils._sdo(node, 0x60C4, 0x06).raw = 0


def _check_zero_velocity(node: canopen.RemoteNode, timeout: float = 5.0) -> None:
    """Wait until the drive reports zero velocity (200Ch.01h bit 8).

    AMC requires the drive to be at rest before the 0x500 start broadcast.

    Args:
        node: Remote CANopen node.
        timeout: Maximum wait in seconds.

    Raises:
        TimeoutError: If zero velocity is not reached within timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        qs = canopen_utils._sdo_read(node, 0x200C, 0x01)
        if qs & (1 << 8):
            logger.info("Zero velocity confirmed (200Ch.01h bit 8 set)")
            return
        time.sleep(0.1)
    raise TimeoutError(f"Drive not at zero velocity within {timeout}s")


# ---------------------------------------------------------------------------
# PVT execution
# ---------------------------------------------------------------------------


def execute_pvt(
    network: canopen.Network,
    node: canopen.RemoteNode,
    rpdo_cob_id: int,
    points: list[PvtPoint],
    threshold: int,
    refill_event: threading.Event,
    fault_event: threading.Event,
    start_cob_id: int,
) -> None:
    """Stream PVT points to the drive and wait for motion to complete.

    Loads up to 15 points into the buffer, starts motion via the
    ``start_cob_id`` broadcast, then refills the buffer each time
    ``refill_event`` fires
    (caller wires this to PVT Status buf_threshold via
    :class:`watcher.WatchProcessor`'s ``on_signal`` hook).  Stops cleanly
    when all points (including the trailing stop point) have been sent
    and the drive signals Target Reached.  Positions are encoded as
    deltas from the previous absolute position; the first delta is
    relative to 0.

    Aborts with :class:`RuntimeError` if ``fault_event`` is set (caller
    wires this to EMCY via ``on_emcy``).  ``fault_event`` setters must
    also set ``refill_event`` to wake the wait loop.
    """
    counter = 0
    idx = 0
    total = len(points)
    prev_pos = 0

    def _send(pt: PvtPoint, slot: int) -> None:
        nonlocal counter, prev_pos
        encoded = PvtPoint(pt.position - prev_pos, pt.velocity, pt.time_ms)
        prev_pos = pt.position
        payload = encode_pvt(encoded, counter)
        network.send_message(rpdo_cob_id, payload)
        logger.debug(
            f"  [{slot:3d}] pos={encoded.position:9d}  vel={encoded.velocity:8d}  "
            f"t={encoded.time_ms:3d}ms  ctr={counter:3d}  raw={payload.hex()}"
        )
        counter = (counter + 1) & 0xFF

    initial = min(_BUFFER_SIZE, total)
    logger.info(f"Loading {initial}/{total} points into PVT buffer")
    for _ in range(initial):
        _send(points[idx], idx)
        idx += 1
    buf_level = canopen_utils._sdo_read(node, 0x60C4, 0x04)
    logger.info(f"PVT buffer level after initial load: {buf_level}")

    # Log a few rounds of SYNC-generated TPDOs before we start spinning.
    time.sleep(0.5)

    # AMC-specific: start via broadcast on RPDO 26, not via ControlWord
    logger.info(f"Sending PVT start broadcast (COB-ID 0x{start_cob_id:03X})")
    network.send_message(start_cob_id, bytes(8))

    # Drop any pre-motion buf_threshold signal: the drive reports
    # buf_threshold=1 right after we clear the buffer (60C4.06h=0) since 0
    # is below threshold, and TPDO 26 may have already delivered that stale
    # state to on_signal.  Discard it now that the buffer is loaded.
    refill_event.clear()

    # Refill the buffer whenever PVT Status's buf_threshold rises.  When the
    # buffer crosses below threshold the bit fires once; it fires again only
    # after the buffer refills above threshold and drops again.  With
    # threshold=10 and buffer=15 the refill batch = 6.
    refill_batch = _BUFFER_SIZE - threshold + 1
    while idx < total:
        if not refill_event.wait(timeout=10.0):
            raise TimeoutError(
                "No PVT buf_threshold signal within 10s; drive may have stopped."
            )
        refill_event.clear()
        if fault_event.is_set():
            raise RuntimeError("EMCY fault during PVT motion; aborting")

        buf_level = canopen_utils._sdo_read(node, 0x60C4, 0x04)
        logger.info(f"PVT buffer level before refill: {buf_level}")

        n = min(refill_batch, total - idx)
        logger.info(f"Buffer low: refilling {n} points [idx {idx}..{idx + n - 1}]")
        for _ in range(n):
            if idx >= total:
                break
            _send(points[idx], idx)
            idx += 1
        buf_level = canopen_utils._sdo_read(node, 0x60C4, 0x04)
        logger.info(f"PVT buffer level after refill: {buf_level}")

    logger.info("All points sent; waiting for PVT to stop...")
    canopen_utils._wait_status(
        node, mask=1 << 6, expected=0, timeout=30.0, index=0x201D, subindex=0x01
    )
    logger.info("Motion complete — Target Reached.")

    # Stick around for a few more seconds so we can log the actual velocity stop.
    time.sleep(3)


# ---------------------------------------------------------------------------
# PVT-only helpers
# ---------------------------------------------------------------------------


def _tpdo_cob_id(od: canopen.ObjectDictionary, pdo_1based_idx: int) -> int:
    """Read the EDS-default COB-ID for a TPDO."""
    comm = od[0x1800 + (pdo_1based_idx - 1)]
    assert isinstance(comm, canopen.objectdictionary.ODRecord)
    raw = comm[1].default
    assert raw is not None
    return int(raw) & 0x7FF


def move(
    interface: str,
    node_id: int,
    to_pos: int,
    total_ms: int,
    segment_ms: int,
    accel_frac: float,
    decel_frac: float,
    threshold: int,
    eds_path: str,
    dry_run: bool,
    sync_interval: float,
    start_cob_id: int,
) -> None:
    """Execute a trapezoidal PVT move.

    Moves the drive by ``to_pos`` encoder counts (offset relative to the
    current position) over ``total_ms`` milliseconds using a trapezoidal
    velocity profile split into three windows: acceleration
    (``accel_frac`` of total time), constant cruise, and deceleration
    (``decel_frac`` of total time).  Points are streamed in ``segment_ms``
    chunks via RPDO 24 and the buffer is refilled each time PVT Status's
    ``buf_threshold`` bit rises (delivered via TPDO 26).

    Trajectory is generated relative to 0 since the drive operates in
    incremental position mode; the drive's actual position is irrelevant.
    """
    logger.info(
        f"delta={to_pos} counts, total={total_ms} ms, "
        f"segment={segment_ms} ms, accel={accel_frac}, decel={decel_frac}"
    )

    points = generate_trapezoid(0, to_pos, total_ms, segment_ms, accel_frac, decel_frac)
    print_trajectory(points, segment_ms)

    if dry_run:
        logger.info("Dry run — not connecting to CAN bus")
        return

    rpdo_cob_id = 0x500 + node_id  # e.g., 0x501 for node 1
    tpdo_cob_id = 0x380 + node_id  # e.g., 0x381 for node 1
    tpdo26_cob_id = 0x490 + node_id  # default: $NODEID + 0x80000490, bit 31 stripped
    logger.info(
        f"RPDO 24 COB-ID = 0x{rpdo_cob_id:03X}, TPDO 24 COB-ID = 0x{tpdo_cob_id:03X}"
    )

    refill_event = threading.Event()
    fault_event = threading.Event()
    last_refill = 0.0

    def _on_signal(_nid: int, channel: str, value: object) -> None:
        nonlocal last_refill
        if channel == "pvt_status.buf_threshold" and value == 1:
            now = time.monotonic()
            if not refill_event.is_set() and now - last_refill >= 0.5:
                last_refill = now
                logger.debug("requesting refill")
                refill_event.set()

    def _on_emcy(_nid: int, fields: dict[str, object]) -> None:
        logger.error(f"EMCY: {fields.get('emcy.text', '<no text>')}")
        fault_event.set()
        refill_event.set()  # wake execute_pvt's wait loop

    watch_cfg = watcher.WatchConfig(node_ids=[node_id], pvt_start_cob_id=start_cob_id)
    pdo_config_path = pathlib.Path("capture") / "pdo.yaml"

    with contextlib.ExitStack() as stack:
        buses = {node_id: watcher.csv_bus_for_node(stack, node_id)}
        processor = stack.enter_context(
            watcher.WatchProcessor(
                watch_cfg, buses, eds=eds_path, pdo_config_path=pdo_config_path
            )
        )
        processor.on_signal = _on_signal
        processor.on_emcy = _on_emcy

        network = canopen.Network()
        network.connect(
            channel=interface, bustype="socketcan", receive_own_messages=True
        )
        try:
            node: canopen.RemoteNode = network.add_node(node_id, eds_path)
            assert network.notifier is not None
            # Run processor.process on a worker thread so PDO decode work
            # doesn't block the notifier thread (which the SDO client also
            # depends on for matching responses).
            listener = stack.enter_context(watcher.ThreadedListener(processor.process))
            network.notifier.add_listener(listener)

            extra_tpdo_enables = {
                0x1800 + (i - 1): _tpdo_cob_id(node.object_dictionary, i)
                for i in (1, 21, 22, 23)
            }
            setup_pvt(
                node,
                rpdo_cob_id,
                tpdo_cob_id,
                threshold,
                start_cob_id,
                extra_tpdo_enables=extra_tpdo_enables,
                pvt_status_cob_id=tpdo26_cob_id,
            )
            canopen_utils.enable_drive(node)
            _check_zero_velocity(node)

            logger.info(f"Starting SYNC at {sync_interval * 1000:.0f} ms interval")
            network.sync.start(sync_interval)
            try:
                execute_pvt(
                    network,
                    node,
                    rpdo_cob_id,
                    points,
                    threshold,
                    refill_event,
                    fault_event,
                    start_cob_id,
                )
            finally:
                network.sync.stop()

        finally:
            network.disconnect()
