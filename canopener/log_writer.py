"""Rotating log writer for CAN frames in candump -ta format."""

from __future__ import annotations

import bz2
import datetime
import os
import shutil
import time
from typing import IO

import can
from loguru import logger


class LogWriter:
    """Context manager that writes CAN frames to rotating log files.

    Files are named ``YYYYMMDD-HHMMSS.log`` and written in ``candump -ta``
    format so they can be replayed with ``--candump``.  When *compress* is
    enabled, each closed file is bzip2-compressed to ``*.log.bz2``.

    Usage::

        with LogWriter("capture", "can0", rotate_seconds=300) as lw:
            for msg in cap:
                lw.write(msg)
    """

    def __init__(
        self,
        save_dir: str,
        interface: str,
        rotate_seconds: float,
        compress: bool = True,
    ) -> None:
        """Args:
        save_dir: Directory to write log files into.
        interface: CAN interface name written into each line.
        rotate_seconds: Open a new file after this many seconds.
        compress: Bzip2-compress each file after it is rotated.
        """
        self._save_dir = save_dir
        self._interface = interface
        self._rotate_seconds = rotate_seconds
        self._compress = compress
        self._file: IO[str] | None = None
        self._current_path: str | None = None
        self._file_open_time: float = 0.0

    def __enter__(self) -> LogWriter:
        self._open_file()
        return self

    def __exit__(self, *_: object) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            if self._compress and self._current_path is not None:
                _bzip(self._current_path)

    def write(self, msg: can.Message) -> None:
        """Write one CAN frame, rotating the file first if the interval has elapsed.

        Args:
            msg: CAN message to write.
        """
        if time.time() - self._file_open_time > self._rotate_seconds:
            self._rotate()

        assert self._file is not None
        dlc = len(msg.data)
        id_hex = (
            f"{msg.arbitration_id:08X}"
            if msg.is_extended_id
            else f"{msg.arbitration_id:03X}"
        )
        data_hex = " ".join(f"{b:02X}" for b in msg.data)
        self._file.write(
            f" ({msg.timestamp:.6f})  {self._interface}  {id_hex}   [{dlc}]  {data_hex}\n"
        )

    def _open_file(self) -> None:
        os.makedirs(self._save_dir, exist_ok=True)
        name = datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".log"
        path = os.path.join(self._save_dir, name)
        self._file = open(path, "a")  # noqa: SIM115
        self._current_path = path
        self._file_open_time = time.time()
        logger.info(f"Saving to {path}")

    def _rotate(self) -> None:
        if self._file is not None:
            self._file.close()
            if self._compress and self._current_path is not None:
                _bzip(self._current_path)
        self._open_file()


def _bzip(path: str) -> None:
    """Compress *path* to *path*.bz2 and remove the original."""
    dest = path + ".bz2"
    with open(path, "rb") as src, bz2.open(dest, "wb") as out:
        shutil.copyfileobj(src, out)
    os.remove(path)
    logger.debug(f"Compressed {path} → {dest}")
