# canopener

CANopen multitool for SocketCAN: live sniffer/decoder and AMC FlexPro PVT motion CLI.

Decodes NMT, heartbeat, PDO, and SDO messages with optional EDS file support
for named signal decoding. Can also parse candump log files offline. The
`pvt-move` and related commands drive an AMC FlexPro motor in PVT
(Interpolated Position) mode.

Kinda vibe-coded.

## Install

```
uv sync                      # core install
uv sync --extra foxglove     # add live Foxglove streaming for `watch --foxglove-port`
```

The `watch` command's Foxglove websocket streaming is optional and disabled by
default; pass `--foxglove-port <port>` (and install the `foxglove` extra) to
enable it.

## Usage

```
canopener [-v|--verbose] COMMAND [ARGS]...

Commands:
  sniff           Sniff/decode CANopen traffic on a SocketCAN interface.
  decode-candump  Decode a candump log file offline.
  watch           Watch traffic and log decoded values to one CSV per node.
  ping            Ping a CANopen node via SDO read of Device Type (0x1000).
  sdo-read        Read an SDO object from the node.
  sdo-write       Write an SDO object on the node.
  nmt-reset       Send an NMT Reset Node command.
  dump-pdo        Dump RPDO/TPDO communication and mapping parameters.
  home            Zero the position using homing method 35.
  limp            Disable the drive so the motor turns freely.
  pvt-move        Execute a trapezoidal PVT move.
```

See `canopener <command> --help` for options. Sniffer-saved files use
`candump -ta` format (`YYYYMMDD-HHMMSS.log`) and can be replayed with
`canopener decode-candump <file>`.

Most commands accept `--eds <path>` to load an EDS file, which resolves raw
object-dictionary addresses to human-readable signal names in decoded output.
It is optional: without it, signals fall back to `0xIDXX:SUB` notation, and
`home`/`limp` use a built-in CiA 402 object dictionary. `pvt-move` requires it.

## Examples

Sniff and decode live traffic on `can0`, with EDS names, saving rotated logs:

```
canopener sniff --interface can0 --eds drive.eds --save
```

Dump a node's RPDO/TPDO communication + mapping parameters (node 2 on `can0`):

```
canopener dump-pdo --interface can0 --node-id 2 --eds drive.eds
```

Read an SDO object. INDEX may be a hex/decimal integer, a CiA 402 name, or any
EDS object name; SUBINDEX defaults to 0:

```
canopener sdo-read --node-id 2 0x6041        # status word
canopener sdo-read --node-id 2 0x1018 1      # vendor ID (subindex 1)
canopener sdo-read --node-id 2 --eds drive.eds statusword
```

Write an SDO object. `--value` accepts decimal or `0x...`; `--size` is the
payload width in bytes:

```
canopener sdo-write --node-id 2 0x6040 --value 0x000F          # control word, 4 bytes
canopener sdo-write --node-id 2 0x6060 --value 7 --size 1      # mode of operation = PVT
```

Execute a trapezoidal PVT move of 100000 counts over 5 s (relative to current
position). `pvt-move` requires `--eds`; add `--dry-run` to print the trajectory
table without touching the bus:

```
canopener pvt-move --interface can0 --node-id 2 --eds drive.eds --to 100000 --time 5000
canopener pvt-move --node-id 2 --eds drive.eds --to 100000 --time 5000 --dry-run
```
