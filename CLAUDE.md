# canopener

## Entry point

- `canopener` → `canopener/cli.py:cli` — click group with top-level subcommands
  `sniff`, `decode-candump`, `pcap2candump`, `watch`, `ping`, `sdo-read`,
  `sdo-read-all`, `sdo-write`, `nmt-reset`, `dump-pdo`, `home`, `limp`,
  `pvt-move`. The node-targeting commands (`ping`, `sdo-read`, `sdo-read-all`,
  `sdo-write`, `nmt-reset`, `dump-pdo`, `home`, `limp`, `pvt-move`) share
  `--interface` and `--node-id` via the `node_options` decorator; `sniff`,
  `decode-candump`, `pcap2candump`, and `watch` define their own options.

## Key modules

- `canopener/cli.py` — all click entrypoints; thin wrappers calling into other modules.
- `canopener/sniffer.py` — `CanOpenDecoder` class; decodes NMT, heartbeat, SDO, PDO from
  live capture or candump log. Loads EDS for signal names. Delegates PDO mapping
  discovery/persistence (`capture/pdo.yaml`) and payload decoding to a
  `pdo_mapping.PdoMappingTracker`. Two high-level entrypoints called from `cli.py`:
  `run(...)` for live SocketCAN capture and `decode_candump(...)` for offline replay.
- `canopener/canopen_utils.py` — non-PVT runners called from cli.py: `ping`, `sdo_read`,
  `sdo_read_all` (reads every readable EDS variable; aborts and EDS/device size
  mismatches are printed, not fatal), `sdo_write`, `nmt_reset`, `dump_pdo`, `home`,
  `limp`. Also hosts shared helpers used
  by `pvt_demo`:
  - `_sdo`, `_sdo_read` — SDO access primitives
  - `_wait_status(node, mask, expected, ..., index, subindex)` — polls any OD word
    (default 0x6041 status word); generalized to support arbitrary index/subindex
  - `enable_drive(node)` — CiA 402 enable sequence
  - `_cia402_fallback_od()` — minimal built-in CiA 402 object dictionary used by
    `home`/`limp` when `--eds` is omitted (typed access to controlword/statusword/
    position objects without an EDS file)
- `canopener/pvt_demo.py` — AMC FlexPro PVT-mode runner `move`, plus:
  - `generate_trapezoid(from_pos, to_pos, ...)` — generates absolute positions from_pos→to_pos
  - `encode_pvt(point, counter)` — validates 24-bit signed range for position and velocity
  - `execute_pvt(network, node, rpdo_cob_id, points, threshold, refill_event, fault_event, start_cob_id)`
    — loads up to 15 points, starts motion via the `start_cob_id` broadcast, refills the buffer
    on each `refill_event` (wired to PVT Status buf_threshold via `WatchProcessor.on_signal`);
    encodes each position as a delta from the previous absolute position (first delta relative
    to 0); aborts on `fault_event` (wired to EMCY via `on_emcy`)
  - `move` runner routes telemetry through a `watcher.WatchProcessor` (on a worker
    thread) writing to a per-node `SinkBus` (`watcher.csv_bus_for_node`); the decoder
    logs StatusWord, Actual Position/Velocity/Current from the monitoring TPDOs, and
    PVT Status + PVT Quick Status (200C.01h) from TPDO 26
  - Imports shared helpers from `canopen_utils` (`_sdo`, `_sdo_read`, `_wait_status`, `enable_drive`)
- `canopener/csv_logger.py` — `CsvLogger` context manager:
  - `CsvLogger(flush_timeout=0.01, name_suffix="")` — no upfront field names; discovers them
    at runtime. `name_suffix` (e.g. `"-node2"`) is appended to the filename.
  - `log_value(field_name, value)` — thread-safe; batches fields into rows by timeout
  - On exit, writes `log/YYYYMMDD-HHMMSS[suffix].csv` with header: `timestamp` first, rest sorted
  - Row timestamp format: `YYYY-MM-DD HH:MM:SS.sss`
- `canopener/decode.py` — CiA 402 decoders shared by both tools:
  - `_WORD_DECODERS` — keyed by lowercase parameter name (used with EDS-resolved names)
  - `_INDEX_DECODERS` — keyed by `(index, subindex)` (used for raw SDO reads without EDS)
  - `NAME_TO_INDEX` — maps well-known parameter names to `(index, subindex)`
  - `annotate_word(name, value)` — annotates by name
  - `annotate_sdo_data(leaf, data_bytes)` — annotates expedited SDO bytes by leaf name
  - `annotate_by_index(index, subindex, data_bytes)` — annotates by OD address
  - `expand_to_fields(name, value)` / `expand_emcy(data)` — fan a word/EMCY out into a
    `{channel: value}` dict of sink-friendly numeric/bitfield sub-channels
- `canopener/watcher.py` — `watch` runner: `WatchProcessor` owns the decoder + per-node
  `PvtTracker`s and is a context manager; decodes frames into per-node `SinkBus` streams.
  `run(...)` is the live multi-node entrypoint; `csv_bus_for_node(...)` builds a CSV
  `SinkBus`; optionally `--foxglove-port` enables live streaming (disabled by default;
  requires the `foxglove` extra). Shared by `pvt_demo.move`.
- `canopener/pdo_mapping.py` — `PdoMappingTracker`: tracks per-node PDO comm/mapping
  parameter state, materializes COB-ID -> `(signal_name, bit_length)` mappings (from
  observed SDO writes, EDS defaults, or `pdo.yaml`), persists to and loads from
  `capture/pdo.yaml`, and decodes PDO payloads.
- `canopener/pvt_tracker.py` — `PvtTracker` + `PvtPoint`: per-node PVT command-stream
  replay state machine; records points streamed into the PVT buffer (RPDO 24) and, after
  the 0x500 start broadcast, replays them into a sink at each segment's expected end time
  (commanded channel aligned with actual telemetry).
- `canopener/sinks.py` — pluggable log sinks for `watch`: `LogSink` protocol, `SinkBus`
  fan-out, `CsvSink` (adapter over `CsvLogger`), `FoxgloveSink` (lazily imports the
  optional `foxglove` extra; only constructed when `--foxglove-port` is set).
- `canopener/pcap2candump.py` — `read_pcap(path)` iterator + `convert(...)`: converts
  classic pcap captures of SocketCAN traffic (Linux SLL v1 from `tcpdump -i any`,
  dropping tx-queue duplicates, or raw SocketCAN link type) to `candump -ta` logs.
- `canopener/capture.py` — `LiveCapture`: live SocketCAN frame capture (iterator) via `can`.
- `canopener/log_writer.py` — `LogWriter`: rotating frame logger in `candump -ta` format,
  with optional bz2 compression.

## Hardware / protocol notes (AMC FlexPro, CiA 402)

- `--eds` is optional for most commands (signal-name decoding only); `home`/`limp`
  fall back to a built-in CiA 402 OD. `pvt-move` still requires `--eds`.
- Default node ID: 2, interface: `can0`
- PVT data → RPDO 24 (comm `0x1417`), buffer-low notification ← TPDO 24 (comm `0x1817`)
- PVT motion starts via broadcast on COB-ID `0x500`, not via ControlWord
- TPDOs 21/22/23 (comm `0x1814`–`0x1816`) are monitoring TPDOs; they are set to
  synchronous (transmission type `0x01`) and enabled during `setup_pvt`. SYNC (`0x080`)
  must be sent periodically to trigger them — `move --sync-interval` controls this
  (default 0.125 s).

## Development rules

- `uv run ruff check`, `uv run ruff format`, `uv run pyright` after every code change.
