"""Pluggable log sinks for the watch command.

A sink consumes ``(timestamp, channel, value)`` triples.  ``SinkBus`` fans one
emit out to several sinks so the watch command can write to CSV today and add
MCAP / Foxglove-live targets later without rewiring the producer side.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

from canopener import csv_logger

if TYPE_CHECKING:
    import foxglove


class LogSink(Protocol):
    """Records a single ``(timestamp, channel, value)`` sample."""

    def log(self, ts: float, channel: str, value: object) -> None: ...


class SinkBus:
    """Fan one log call out to a list of sinks."""

    def __init__(self, sinks: list[LogSink]) -> None:
        self._sinks = list(sinks)

    def log(self, ts: float, channel: str, value: object) -> None:
        for s in self._sinks:
            s.log(ts, channel, value)


class CsvSink:
    """LogSink adapter that forwards into a :class:`CsvLogger`."""

    def __init__(self, csv: csv_logger.CsvLogger) -> None:
        self._csv = csv

    def log(self, ts: float, channel: str, value: object) -> None:
        self._csv.log_value(channel, value, timestamp=ts)


_JSON_TYPE: dict[type, str] = {
    bool: "boolean",
    int: "number",
    float: "number",
    str: "string",
}


class FoxgloveSink:
    """LogSink that publishes each sample to a Foxglove websocket channel.

    One Foxglove ``Channel`` per ``(prefix + signal name)`` pair, created on
    first sight with a JSON Schema derived from the first value's type so
    Foxglove Studio's Plot panel can resolve ``.value``. Each message is
    ``{"value": ...}``.
    """

    def __init__(self, topic_prefix: str = "/") -> None:
        import foxglove

        self._foxglove = foxglove
        self._prefix = topic_prefix
        self._channels: dict[str, foxglove.Channel] = {}

    def log(self, ts: float, channel: str, value: object) -> None:
        topic = self._prefix + channel.replace(".", "/")
        ch = self._channels.get(topic)
        if ch is None:
            json_type = _JSON_TYPE.get(type(value), "string")
            schema = {
                "type": "object",
                "properties": {"value": {"type": json_type}},
            }
            ch = self._foxglove.Channel(topic, schema=schema)
            self._channels[topic] = ch
        ch.log({"value": value}, log_time=int(ts * 1e9))
