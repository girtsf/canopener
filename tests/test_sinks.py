"""Tests for canopener.sinks."""

from __future__ import annotations

import csv
import pathlib

import pytest

from canopener import csv_logger
from canopener import sinks


class _RecordingSink:
    def __init__(self) -> None:
        self.events: list[tuple[float, str, object]] = []

    def log(self, ts: float, channel: str, value: object) -> None:
        self.events.append((ts, channel, value))


def test_sink_bus_fans_out_to_all_sinks() -> None:
    a = _RecordingSink()
    b = _RecordingSink()
    bus = sinks.SinkBus([a, b])
    bus.log(123.0, "chan", 42)
    assert a.events == [(123.0, "chan", 42)]
    assert b.events == [(123.0, "chan", 42)]


def test_sink_bus_empty() -> None:
    bus = sinks.SinkBus([])
    bus.log(0.0, "x", 0)  # must not raise


def test_sink_bus_preserves_order() -> None:
    a = _RecordingSink()
    bus = sinks.SinkBus([a])
    bus.log(1.0, "x", 1)
    bus.log(2.0, "x", 2)
    bus.log(3.0, "x", 3)
    assert [e[2] for e in a.events] == [1, 2, 3]


def test_csv_sink_forwards_to_csv_logger(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    with csv_logger.CsvLogger() as log:
        sink = sinks.CsvSink(log)
        sink.log(1000.0, "actual_position", 12345)
        sink.log(1000.0, "actual_velocity", 678.9)

    files = list((tmp_path / "log").glob("*.csv"))
    assert len(files) == 1
    with files[0].open() as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    assert len(rows) == 1
    assert rows[0]["actual_position"] == "12345"
    assert rows[0]["actual_velocity"] == "678.9"
