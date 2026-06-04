"""PDO mapping discovery, persistence, and payload decoding.

Tracks per-node PDO communication and mapping parameter state, materializes
COB-ID -> ordered ``(signal_name, bit_length)`` mappings, and decodes PDO
payloads against them.

Mapping sources, in effective precedence (highest first):
    1. Observed SDO writes (latest write wins; rebuild fires on relevant subindex).
    2. YAML config (loaded explicitly via ``load_config``).
    3. EDS defaults (seed raw comm/map state only; do not register a mapping
       on their own — a later SDO write must complete the picture).
"""

from __future__ import annotations

from collections.abc import Callable
import os
import pathlib
import tempfile

import canopen
import canopen.objectdictionary
from loguru import logger
import yaml

OdProvider = Callable[[int], canopen.ObjectDictionary | None]


class PdoMappingTracker:
    """Tracks PDO mappings discovered from SDO writes / YAML / EDS defaults."""

    def __init__(
        self,
        config_path: pathlib.Path | None = None,
        od_provider: OdProvider | None = None,
    ) -> None:
        """Instantiates PdoMappingTracker.

        Args:
            config_path: Optional YAML file used to persist discovered PDO
                mappings across runs.
            od_provider: Optional callback returning the OD for a given node ID, used to resolve mapped objects to human-readable names.
        """
        self._config_path = config_path
        self._od_provider = od_provider
        # mappings[cob_id] = list of (signal_name, bit_length) tuples
        self._mappings: dict[int, list[tuple[str, int]]] = {}
        # names[cob_id] = label like "TPDO26"
        self._names: dict[int, str] = {}
        # nodes[cob_id] = owning node ID (the node whose comm param produced this COB-ID)
        self._nodes: dict[int, int] = {}
        # Raw PDO comm/map state per node: node_id -> pdo_idx -> subidx -> value
        self._tpdo_comm_raw: dict[int, dict[int, dict[int, int]]] = {}
        self._rpdo_comm_raw: dict[int, dict[int, dict[int, int]]] = {}
        self._tpdo_map_raw: dict[int, dict[int, dict[int, int]]] = {}
        self._rpdo_map_raw: dict[int, dict[int, dict[int, int]]] = {}

    # ---- read ----

    def mapping_for(self, cob_id: int) -> list[tuple[str, int]] | None:
        """Return ordered ``(signal_name, bit_length)`` for ``cob_id``, or ``None``."""
        return self._mappings.get(cob_id)

    def name_for(self, cob_id: int, default: str = "PDO") -> str:
        """Return label like ``TPDO26`` for ``cob_id``, or ``default`` if unknown."""
        return self._names.get(cob_id, default)

    def node_for(self, cob_id: int) -> int | None:
        """Return the owning node ID for ``cob_id``, or ``None`` if unknown.

        Set when the mapping is registered (via SDO write tracking or an
        explicit :meth:`set_mapping` call with ``node_id``).  Persisted
        through YAML.
        """
        return self._nodes.get(cob_id)

    def decode_payload(
        self, cob_id: int, data: bytes | bytearray
    ) -> list[tuple[str, int]] | None:
        """Decode a PDO payload into ``(signal_name, raw_value)`` pairs.

        Returns ``None`` if no mapping is registered for ``cob_id``.
        """
        signals = self._mappings.get(cob_id)
        if signals is None:
            return None
        out: list[tuple[str, int]] = []
        bit_offset = 0
        raw = int.from_bytes(data, "little")
        for name, bits in signals:
            mask = (1 << bits) - 1
            out.append((name, (raw >> bit_offset) & mask))
            bit_offset += bits
        return out

    # ---- mutate ----

    def set_mapping(
        self,
        cob_id: int,
        signals: list[tuple[str, int]],
        name: str | None = None,
        node_id: int | None = None,
    ) -> None:
        """Register a mapping; persist to ``config_path`` if set."""
        self._mappings[cob_id] = signals
        if name is not None:
            self._names[cob_id] = name
        if node_id is not None:
            self._nodes[cob_id] = node_id
        self._save_config()

    def handle_sdo_write(
        self, node_id: int, index: int, subindex: int, value: int
    ) -> None:
        """Update internal state from an observed expedited SDO write.

        Tracks writes to TPDO/RPDO communication (0x1800/0x1400) and mapping
        (0x1A00/0x1600) parameter objects; calls :meth:`set_mapping` when enough
        state is available to construct a complete mapping.  Writes outside
        those ranges are ignored.
        """
        pdo_idx = index & 0xFF

        if 0x1800 <= index <= 0x18FF:
            self._tpdo_comm_raw.setdefault(node_id, {}).setdefault(pdo_idx, {})[
                subindex
            ] = value
            if subindex == 1 and not (value & 0x80000000):
                self._rebuild(node_id, pdo_idx, is_tpdo=True)
        elif 0x1400 <= index <= 0x14FF:
            self._rpdo_comm_raw.setdefault(node_id, {}).setdefault(pdo_idx, {})[
                subindex
            ] = value
            if subindex == 1 and not (value & 0x80000000):
                self._rebuild(node_id, pdo_idx, is_tpdo=False)
        elif 0x1A00 <= index <= 0x1AFF:
            self._tpdo_map_raw.setdefault(node_id, {}).setdefault(pdo_idx, {})[
                subindex
            ] = value
            if subindex == 0 and value > 0:
                self._rebuild(node_id, pdo_idx, is_tpdo=True)
        elif 0x1600 <= index <= 0x16FF:
            self._rpdo_map_raw.setdefault(node_id, {}).setdefault(pdo_idx, {})[
                subindex
            ] = value
            if subindex == 0 and value > 0:
                self._rebuild(node_id, pdo_idx, is_tpdo=False)

    def apply_eds_defaults(self, node_id: int, od: canopen.ObjectDictionary) -> None:
        """Seed raw comm/map state from PDO defaults in ``od``.

        Does not register any mapping on its own; a subsequent SDO write that
        completes the picture will trigger a rebuild.
        """
        for comm_base, comm_dict in (
            (0x1800, self._tpdo_comm_raw),
            (0x1A00, self._tpdo_map_raw),
            (0x1400, self._rpdo_comm_raw),
            (0x1600, self._rpdo_map_raw),
        ):
            for pdo_idx in range(0x100):
                comm_idx = comm_base + pdo_idx
                if comm_idx not in od:
                    continue
                comm_obj = od[comm_idx]
                if not isinstance(comm_obj, canopen.objectdictionary.ODRecord):
                    continue
                count_sub = comm_obj[0].default
                assert count_sub is not None
                sub_index_count = int(count_sub)
                comm_dict.setdefault(node_id, {}).setdefault(pdo_idx, {})[0] = (
                    sub_index_count
                )

                for subidx in range(1, sub_index_count + 1):
                    entry = comm_obj[subidx]
                    value = entry.default
                    if value is not None:
                        comm_dict[node_id][pdo_idx][subidx] = value

    # ---- persistence ----

    def load_config(self) -> None:
        """Load mappings from ``config_path``, if it exists.

        Populates internal state directly without firing a save.
        """
        if self._config_path is None or not self._config_path.exists():
            return
        with self._config_path.open() as f:
            data: object = yaml.safe_load(f)
        if not isinstance(data, dict):
            return
        for key, entry in data.items():
            cob_id = int(key, 16)
            name: str = entry.get("name", "")
            node_id = entry.get("node_id")
            signals = [(s["name"], int(s["bits"])) for s in entry.get("signals", [])]
            if signals:
                if name:
                    self._names[cob_id] = name
                if node_id is not None:
                    self._nodes[cob_id] = int(node_id)
                self._mappings[cob_id] = signals
        logger.debug(
            f"Loaded {len(self._mappings)} PDO mappings from {self._config_path}"
        )

    # ---- internals ----

    def _save_config(self) -> None:
        if self._config_path is None:
            return
        data: dict[str, object] = {}
        for cob_id, signals in self._mappings.items():
            entry: dict[str, object] = {
                "name": self._names.get(cob_id, ""),
                "signals": [{"name": s[0], "bits": s[1]} for s in signals],
            }
            node_id = self._nodes.get(cob_id)
            if node_id is not None:
                entry["node_id"] = node_id
            data[f"0x{cob_id:03X}"] = entry
        self._config_path.parent.mkdir(parents=True, exist_ok=True)
        # Write to a temp file in the same directory, then atomically rename over
        # the target. A plain open("w") truncates in place, so a crash or a
        # concurrent shorter write leaves a partial/garbage file; os.replace is
        # atomic on POSIX, so readers always see a complete old or new file.
        fd, tmp_name = tempfile.mkstemp(
            dir=self._config_path.parent, prefix=self._config_path.name, suffix=".tmp"
        )
        try:
            with os.fdopen(fd, "w") as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_name, self._config_path)
        except BaseException:
            os.unlink(tmp_name)
            raise
        logger.debug(f"Saved PDO config to {self._config_path}")

    def _rebuild(self, node_id: int, pdo_idx: int, *, is_tpdo: bool) -> None:
        """Reconstruct a PDO mapping from accumulated comm/map state.

        Calls :meth:`set_mapping` only when both a valid (enabled) COB-ID and at
        least one mapping entry are present.
        """
        comm = (
            (self._tpdo_comm_raw if is_tpdo else self._rpdo_comm_raw)
            .get(node_id, {})
            .get(pdo_idx, {})
        )
        raw_map = (
            (self._tpdo_map_raw if is_tpdo else self._rpdo_map_raw)
            .get(node_id, {})
            .get(pdo_idx, {})
        )

        cob_raw = comm.get(1)
        if cob_raw is None or cob_raw & 0x80000000:  # no COB-ID or PDO disabled
            return
        cob_id = cob_raw & 0x7FF

        count = raw_map.get(0, 0)
        if not count:
            return

        od = self._od_provider(node_id) if self._od_provider is not None else None

        signals: list[tuple[str, int]] = []
        for j in range(1, count + 1):
            val = raw_map.get(j)
            if val is None:
                break
            obj_idx = (val >> 16) & 0xFFFF
            obj_sub = (val >> 8) & 0xFF
            bit_len = val & 0xFF
            if not bit_len:
                continue
            name = f"0x{obj_idx:04X}:{obj_sub:02X}"
            if od is not None:
                try:
                    obj = od[obj_idx]
                    obj_name: str = obj.name
                    if isinstance(
                        obj,
                        (
                            canopen.objectdictionary.ODRecord,
                            canopen.objectdictionary.ODArray,
                        ),
                    ):
                        try:
                            name = f"{obj_name}.{obj[obj_sub].name}"
                        except KeyError:
                            name = obj_name
                    else:
                        name = obj_name
                except (KeyError, AttributeError):
                    pass
            signals.append((name, bit_len))

        if not signals:
            return

        direction = "TPDO" if is_tpdo else "RPDO"
        signals_str = ", ".join([f"{s[0]}:{s[1]}" for s in signals])
        logger.debug(
            f"SDO: {direction}{pdo_idx + 1} node={node_id} cob=0x{cob_id:03X} "
            f"signals=[{signals_str}]"
        )
        self.set_mapping(
            cob_id, signals, name=f"{direction}{pdo_idx + 1}", node_id=node_id
        )
