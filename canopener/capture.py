"""Live CAN bus capture via SocketCAN."""

from __future__ import annotations

from collections.abc import Iterator

import can


class LiveCapture:
    """Context manager that yields CAN messages from a live SocketCAN interface.

    Usage::

        with LiveCapture("can0") as cap:
            for msg in cap:
                process(msg)

    Iteration stops cleanly on KeyboardInterrupt.
    """

    def __init__(self, interface: str) -> None:
        """Args:
        interface: SocketCAN interface name (e.g. ``"can0"``).
        """
        self._interface = interface
        self._bus: can.BusABC | None = None
        self._reader: can.BufferedReader | None = None
        self._notifier: can.Notifier | None = None

    def __enter__(self) -> LiveCapture:
        bus = can.Bus(interface="socketcan", channel=self._interface)
        self._bus = bus
        self._reader = can.BufferedReader()
        self._notifier = can.Notifier(bus, [self._reader])
        return self

    def __exit__(self, *_: object) -> None:
        if self._notifier is not None:
            self._notifier.stop()
        if self._bus is not None:
            self._bus.shutdown()

    def __iter__(self) -> Iterator[can.Message]:
        """Yield messages until the bus is silent or KeyboardInterrupt is raised."""
        assert self._reader is not None, "Use LiveCapture as a context manager"
        try:
            while True:
                msg = self._reader.get_message(timeout=1.0)
                if msg is None:
                    continue
                yield msg
        except KeyboardInterrupt:
            return
