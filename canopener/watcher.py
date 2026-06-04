"""Live `watch` runner: decodes CANopen frames into per-node sink streams.

Wires together :class:`sniffer.CanOpenDecoder` (PDO mapping discovery + decode),
:func:`decode.expand_to_fields` (bitfield fan-out), and per-node
:class:`pvt_tracker.PvtTracker` instances (PVT command replay).
"""

from __future__ import annotations

from collections.abc import Callable
import contextlib
import pathlib
import queue
import threading
import time
from dataclasses import dataclass

import can
from loguru import logger

from canopener import capture
from canopener import csv_logger
from canopener import decode
from canopener import pvt_tracker
from canopener import sinks
from canopener import sniffer


@dataclass
class WatchConfig:
    """Top-level watch configuration."""

    node_ids: list[int]
    pvt_start_cob_id: int = 0x500


class WatchProcessor:
    """Decode CAN frames and dispatch values into per-node sinks + PVT trackers."""

    def __init__(
        self,
        config: WatchConfig,
        sink_buses: dict[int, sinks.SinkBus],
        eds: str | None = None,
        pdo_config_path: pathlib.Path | None = None,
    ) -> None:
        self._config = config
        self._buses = sink_buses
        self.decoder = sniffer.CanOpenDecoder(
            eds, pdo_config_path=pdo_config_path, on_sdo_write=self._on_sdo_write
        )
        self.decoder.pdo.load_config()
        self.trackers: dict[int, pvt_tracker.PvtTracker] = {
            nid: pvt_tracker.PvtTracker(nid, sink_buses[nid]) for nid in config.node_ids
        }
        # Fires after every ``bus.log(ts, channel, value)``, with the owning
        # node ID. Used to react to decoded fields without subscribing a sink.
        self.on_signal: Callable[[int, str, object], None] | None = None
        # Fires after every EMCY frame's fields are emitted to the bus, with
        # the owning node ID and the decoded fields dict.
        self.on_emcy: Callable[[int, dict[str, object]], None] | None = None

    def __enter__(self) -> WatchProcessor:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop()

    def stop(self) -> None:
        """Stop owned PvtTracker threads. Idempotent."""
        for tracker in self.trackers.values():
            tracker.stop()

    def _on_sdo_write(
        self, node_id: int, index: int, subindex: int, value: int
    ) -> None:
        """SDO download hook from :class:`CanOpenDecoder`."""
        tracker = self.trackers.get(node_id)
        if tracker is None:
            return
        if index == 0x60C4 and subindex == 0x06 and value == 0:
            logger.debug(f"node {node_id}: PVT buffer cleared (60C4.06h <- 0)")
            tracker.clear()

    def _pvt_node_for_cob(self, cob: int) -> int | None:
        """Return the watched node whose PVT-data RPDO is ``cob``, or None.

        First check PdoMappingTracker for a learned RPDO 24 mapping; fall
        back to the AMC default ``0x500 + node_id`` for any watched node.
        """
        if self.decoder.pdo.name_for(cob, default="") == "RPDO24":
            node_id = self.decoder.pdo.node_for(cob)
            if node_id in self.trackers:
                return node_id
        if cob & ~0x7F == 0x500:
            node_id = cob - 0x500
            if node_id in self.trackers:
                return node_id
        return None

    def process(self, msg: can.Message) -> None:
        """Process one CAN frame."""
        cob = msg.arbitration_id
        ts = time.time()

        # PVT broadcast start fires every configured node's tracker.
        if cob == self._config.pvt_start_cob_id:
            logger.debug(f"PVT start broadcast (cob 0x{cob:03X})")
            for tracker in self.trackers.values():
                tracker.start(ts)
            return

        # Drive the decoder first so PdoMappingTracker sees any SDO writes
        # (e.g. RPDO 24's COB-ID being reconfigured) before we look up the
        # PVT-data routing for *this* frame.  decoder.decode() is cheap for
        # non-SDO frames (only state-machine bookkeeping for heartbeat etc).
        self.decoder.decode(msg)

        # Per-node PVT data RPDO: decode the 8-byte point and feed the tracker.
        node_id = self._pvt_node_for_cob(cob)
        if node_id is not None:
            if len(msg.data) >= 8:
                pos = int.from_bytes(msg.data[0:3], "little", signed=True)
                vel = int.from_bytes(msg.data[3:6], "little", signed=True)
                self.trackers[node_id].add_point(
                    position=pos,
                    velocity=vel,
                    time_ms=msg.data[6],
                    counter=msg.data[7],
                    ts=ts,
                )
            else:
                logger.warning(
                    f"node {node_id}: short PVT RPDO frame ({len(msg.data)} bytes)"
                )
            return

        # EMCY (0x081–0x0FF): emit decoded fault info to the owning node's bus.
        if 0x081 <= cob <= 0x0FF:
            emcy_node = cob - 0x080
            bus = self._buses.get(emcy_node)
            if bus is None:
                return
            fields = decode.expand_emcy(msg.data)
            logger.warning(
                f"node {emcy_node}: {fields.get('emcy.text', msg.data.hex())}"
            )
            for channel, value in fields.items():
                bus.log(ts, channel, value)
                if self.on_signal is not None:
                    self.on_signal(emcy_node, channel, value)
            if self.on_emcy is not None:
                self.on_emcy(emcy_node, fields)
            return

        # Fan PDO signals out to the owning node's sink, if it's a node we watch.
        # Prefer the tracker's owning-node ID; PDOs may be remapped away from
        # the default 0x180+node_id layout.
        node_id = self.decoder.pdo.node_for(cob)
        if node_id is None:
            node_id = cob & 0x7F
        bus = self._buses.get(node_id)
        if bus is None:
            return
        signals = self.decoder.pdo.decode_payload(cob, msg.data)
        if signals is None:
            return
        tracker = self.trackers.get(node_id)
        for name, value in signals:
            for channel, decoded in decode.expand_to_fields(name, value).items():
                bus.log(ts, channel, decoded)
                if self.on_signal is not None:
                    self.on_signal(node_id, channel, decoded)
                if (
                    tracker is not None
                    and channel == "actual_position"
                    and isinstance(decoded, int)
                ):
                    tracker.update_actual_position(decoded)


class ThreadedListener:
    """python-can listener that dispatches frames on a worker thread.

    The canopen library's notifier calls every listener inline on its single
    reader thread.  When a listener's work is heavy (e.g. PDO decode + sink
    writes), it can delay the reader enough to miss SDO responses sitting in
    the kernel socket buffer, surfacing as ``SdoCommunicationError("No SDO
    response received")`` even though the response did arrive on the wire.

    Wrapping the heavy callback in this listener moves the work off the
    notifier thread: ``__call__`` enqueues and returns immediately; a worker
    thread drains the queue at its own pace.
    """

    def __init__(self, callback: Callable[[can.Message], None]) -> None:
        self._callback = callback
        self._queue: queue.Queue[can.Message | None] = queue.Queue()
        self._thread = threading.Thread(
            target=self._run, name="threaded-listener", daemon=True
        )
        self._thread.start()

    def __enter__(self) -> ThreadedListener:
        return self

    def __exit__(self, *_: object) -> None:
        self.stop()

    def __call__(self, msg: can.Message) -> None:
        self._queue.put(msg)

    def stop(self) -> None:
        """Drain the queue and shut down the worker thread.  Idempotent."""
        self._queue.put(None)
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while True:
            msg = self._queue.get()
            if msg is None:
                return
            self._callback(msg)


def csv_bus_for_node(stack: contextlib.ExitStack, node_id: int) -> sinks.SinkBus:
    """Build the default per-node CSV SinkBus, registering its lifetime in ``stack``."""
    log = stack.enter_context(csv_logger.CsvLogger(name_suffix=f"-node{node_id}"))
    return sinks.SinkBus([sinks.CsvSink(log)])


@contextlib.contextmanager
def _foxglove_server(port: int):
    """Start a Foxglove websocket server bound to ``0.0.0.0:port``."""
    import foxglove

    server = foxglove.start_server(host="0.0.0.0", port=port, name="canopener")
    logger.info(f"Foxglove websocket server: ws://0.0.0.0:{port}")
    try:
        yield server
    finally:
        server.stop()


def run(
    interface: str,
    eds: str | None,
    config: WatchConfig,
    foxglove_port: int | None = None,
) -> None:
    """Run the watcher on a live SocketCAN interface until Ctrl+C.

    Args:
        interface: SocketCAN interface name.
        eds: Optional EDS file for signal name resolution.
        config: Per-node watch configuration.
        foxglove_port: If set, also publish all logged samples to a Foxglove
            websocket server on this port.
    """
    pdo_config_path = pathlib.Path("capture") / "pdo.yaml"
    with contextlib.ExitStack() as stack:
        if foxglove_port is not None:
            stack.enter_context(_foxglove_server(foxglove_port))

        buses: dict[int, sinks.SinkBus] = {}
        for nid in config.node_ids:
            log_sinks: list[sinks.LogSink] = [
                sinks.CsvSink(
                    stack.enter_context(csv_logger.CsvLogger(name_suffix=f"-node{nid}"))
                )
            ]
            if foxglove_port is not None:
                log_sinks.append(sinks.FoxgloveSink(f"/node{nid}/"))
            buses[nid] = sinks.SinkBus(log_sinks)

        processor = stack.enter_context(
            WatchProcessor(config, buses, eds=eds, pdo_config_path=pdo_config_path)
        )

        node_ids_str = ", ".join(str(n) for n in config.node_ids)
        logger.info(
            f"Watching {interface} for nodes [{node_ids_str}] — press Ctrl+C to stop"
        )
        with capture.LiveCapture(interface) as cap:
            try:
                for msg in cap:
                    processor.process(msg)
            except KeyboardInterrupt:
                logger.info("Stopped.")
