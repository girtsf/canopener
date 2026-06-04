"""Tests for canopener.csv_logger.CsvLogger."""

from __future__ import annotations

import csv
import datetime
import pathlib

import pytest

from canopener import csv_logger


def _read_csv(log_dir: pathlib.Path) -> tuple[list[str], list[dict[str, str]]]:
    """Return (header, rows) from the single CSV file under ``log_dir``."""
    files = sorted(log_dir.glob("*.csv"))
    assert len(files) == 1, f"expected 1 CSV under {log_dir}, got {files}"
    with files[0].open() as f:
        reader = csv.DictReader(f)
        assert reader.fieldnames is not None
        return list(reader.fieldnames), list(reader)


def test_single_value_single_row(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    with csv_logger.CsvLogger() as log:
        log.log_value("Some Field", 42, timestamp=1_700_000_000.0)

    header, rows = _read_csv(tmp_path / "log")
    assert header == ["timestamp", "Some Field"]
    assert len(rows) == 1
    assert rows[0]["Some Field"] == "42"
    assert rows[0]["timestamp"].startswith("2023-")  # 2023-11-something


def test_explicit_timestamp_drives_row_label(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # 2024-01-15 12:34:56.789 UTC.  We render with fromtimestamp() so the test
    # only checks that the supplied epoch is what the formatter sees, not the
    # wall-clock timezone of the test host — but the milliseconds are exact.
    monkeypatch.chdir(tmp_path)
    ts = 1_705_320_896.789
    with csv_logger.CsvLogger() as log:
        log.log_value("F", 1, timestamp=ts)

    _, rows = _read_csv(tmp_path / "log")
    # Full label is YYYY-MM-DD HH:MM:SS.mmm (never year-day-month), rendered in
    # the host timezone — compare against the same fromtimestamp() the code uses.
    expected = (
        datetime.datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S") + ".789"
    )
    assert rows[0]["timestamp"] == expected


def test_values_within_timeout_share_a_row(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    with csv_logger.CsvLogger(flush_timeout=0.1) as log:
        log.log_value("A", 1, timestamp=1000.000)
        log.log_value("B", 2, timestamp=1000.050)  # +50ms < 100ms

    header, rows = _read_csv(tmp_path / "log")
    assert header == ["timestamp", "A", "B"]
    assert len(rows) == 1
    assert rows[0]["A"] == "1"
    assert rows[0]["B"] == "2"


def test_values_beyond_timeout_split_rows(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    with csv_logger.CsvLogger(flush_timeout=0.1) as log:
        log.log_value("A", 1, timestamp=1000.000)
        log.log_value("A", 2, timestamp=1000.200)  # +200ms > 100ms

    header, rows = _read_csv(tmp_path / "log")
    assert header == ["timestamp", "A"]
    assert len(rows) == 2
    assert rows[0]["A"] == "1"
    assert rows[1]["A"] == "2"


def test_name_suffix_changes_filename(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    with csv_logger.CsvLogger(name_suffix="-node3") as log:
        log.log_value("X", 0, timestamp=1000.0)

    files = list((tmp_path / "log").glob("*.csv"))
    assert len(files) == 1
    assert files[0].name.endswith("-node3.csv")


def test_two_loggers_distinct_files(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    with (
        csv_logger.CsvLogger(name_suffix="-node2") as log2,
        csv_logger.CsvLogger(name_suffix="-node3") as log3,
    ):
        log2.log_value("X", 1, timestamp=1000.0)
        log3.log_value("X", 2, timestamp=1000.0)

    files = sorted((tmp_path / "log").glob("*.csv"))
    names = [f.name for f in files]
    assert any(n.endswith("-node2.csv") for n in names)
    assert any(n.endswith("-node3.csv") for n in names)


def test_pending_row_flushed_on_exit(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    with csv_logger.CsvLogger(flush_timeout=10.0) as log:
        # Single value, no second log_value to trigger batched flush.
        log.log_value("Only", "v", timestamp=1000.0)

    _, rows = _read_csv(tmp_path / "log")
    assert len(rows) == 1
    assert rows[0]["Only"] == "v"


def test_no_timestamp_falls_back_to_now(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # When no timestamp is passed, the row label must still be populated.
    monkeypatch.chdir(tmp_path)
    with csv_logger.CsvLogger() as log:
        log.log_value("F", 1)

    _, rows = _read_csv(tmp_path / "log")
    assert rows[0]["timestamp"]  # non-empty
