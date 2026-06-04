"""CSV logger that batches fields into rows by timeout or completion."""

from __future__ import annotations

import csv
import datetime
import os
import threading
import time


class CsvLogger:
    """Context manager that buffers named fields into rows, writing on exit.

    Batches calls to :meth:`log_value` into rows: a row is closed when the
    next :meth:`log_value` call arrives after *flush_timeout* seconds have
    elapsed since the first field of the current row.  On context exit, any
    pending row is closed, then all rows are written to ``log/YYYYMMDD-HHMMSS.csv``
    with a header derived from every field name seen (timestamp first, rest
    sorted alphabetically).

    Usage::

        with CsvLogger() as log:
            log.log_value("StatusWord", "State=Operation Enabled")
            log.log_value("Actual Position", 12345)
    """

    def __init__(self, flush_timeout: float = 0.01, name_suffix: str = "") -> None:
        """Args:
        flush_timeout: Seconds after first field of a row before the row is
            closed.  Checked on the next log_value() call.
        name_suffix: String appended to the auto-generated filename (before
            ``.csv``).  E.g. ``"-node2"`` produces ``log/YYYYMMDD-HHMMSS-node2.csv``.
        """
        self._flush_timeout = flush_timeout
        self._name_suffix = name_suffix
        self._pending: dict[str, object] = {}
        self._row_start: float | None = None
        self._row_ts: str = ""
        self._rows: list[dict[str, object]] = []
        self._all_fields: set[str] = set()
        self._lock = threading.Lock()
        self._open_ts: str = ""

    def __enter__(self) -> CsvLogger:
        self._open_ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        return self

    def __exit__(self, *_: object) -> None:
        with self._lock:
            if self._pending:
                self._flush_locked()
            rows = list(self._rows)

            fields = ["timestamp"] + sorted(self._all_fields)

        os.makedirs("log", exist_ok=True)
        path = os.path.join("log", self._open_ts + self._name_suffix + ".csv")
        with open(path, "w", newline="") as f:  # noqa: SIM115
            writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

    def log_value(
        self,
        field_name: str,
        value: object,
        timestamp: float | None = None,
    ) -> None:
        """Record a value for one field, closing a timed-out row first if needed.

        Args:
            field_name: Field name; determines CSV column (auto-discovered).
            value: Value to record.
            timestamp: Wallclock Unix epoch seconds.  If ``None``, ``time.time()``
                is used.  The same timestamp drives both the row's CSV label and
                the row-batching window — pass the same value for every field of
                a single event so they batch into one row.
        """
        ts = timestamp if timestamp is not None else time.time()
        with self._lock:
            if self._pending and self._row_start is not None:
                if ts - self._row_start >= self._flush_timeout:
                    self._flush_locked()
            if not self._pending:
                self._row_start = ts
                _now = datetime.datetime.fromtimestamp(ts)
                self._row_ts = f"{_now.strftime('%Y-%m-%d %H:%M:%S')}.{_now.microsecond // 1000:03d}"
            self._pending[field_name] = value
            self._all_fields.add(field_name)

    def _flush_locked(self) -> None:
        self._rows.append({"timestamp": self._row_ts, **self._pending})
        self._pending = {}
        self._row_start = None
