#!/usr/bin/env python3
"""canopener entry point — click commands for sniffer and PVT tools."""

from __future__ import annotations

from collections.abc import Callable
import sys

import click
from loguru import logger

from canopener import canopen_utils
from canopener import pcap2candump
from canopener import pvt_demo
from canopener import sniffer
from canopener import watcher


def setup_logging(verbose: bool) -> None:
    """Configure loguru sink with appropriate level."""
    logger.remove()
    level = "DEBUG" if verbose else "INFO"
    fmt = (
        "<green>{time:HH:mm:ss.SSS}</green> <level>{level.name:.1}</level> "
        "<cyan>{name}</cyan>:"
        "<cyan>{function}</cyan>:"
        "<cyan>{line}</cyan> "
        "| "
        "<level>{message}</level>"
    )
    logger.add(sys.stdout, level=level, format=fmt)


@click.group()
@click.option("-v", "--verbose", is_flag=True, help="Enable debug logging.")
def cli(verbose: bool) -> None:
    """CANopen multitool: sniffer and AMC FlexPro PVT motion tools."""
    setup_logging(verbose)


def node_options(f: Callable) -> Callable:
    """Add --interface and --node-id options to a command."""
    f = click.option(
        "--node-id", default=2, show_default=True, type=int, help="CANopen node ID."
    )(f)
    f = click.option(
        "--interface",
        default="can0",
        show_default=True,
        help="SocketCAN interface name.",
    )(f)
    return f


# ---------------------------------------------------------------------------
# sniff
# ---------------------------------------------------------------------------


@cli.command()
@click.option(
    "-i", "--interface", default="can0", show_default=True, help="CAN interface name."
)
@click.option(
    "--eds",
    default=None,
    type=click.Path(exists=True, dir_okay=False),
    help="EDS file for signal decoding.",
)
@click.option("--save", is_flag=True, help="Save captured frames to disk.")
@click.option(
    "--save-dir",
    default="capture",
    show_default=True,
    type=click.Path(file_okay=False),
    help="Directory for saved log files.",
)
@click.option(
    "--rotate",
    default=300.0,
    show_default=True,
    type=float,
    help="Seconds before rotating the log file.",
)
@click.option(
    "--compress/--no-compress",
    default=True,
    show_default=True,
    help="Bzip2-compress log files after rotation.",
)
def sniff(
    interface: str,
    eds: str | None,
    save: bool,
    save_dir: str,
    rotate: float,
    compress: bool,
) -> None:
    """Sniff and decode CANopen traffic on a SocketCAN interface."""
    sniffer.run(interface, eds, save, save_dir, rotate, compress)


@cli.command("decode-candump")
@click.argument(
    "candump_path",
    type=click.Path(exists=True, dir_okay=False),
    metavar="CANDUMP_LOG",
)
@click.option(
    "--eds",
    default=None,
    type=click.Path(exists=True, dir_okay=False),
    help="EDS file for signal decoding.",
)
@click.option(
    "--pdo-config",
    default=None,
    type=click.Path(exists=True, dir_okay=False),
    help="Path to a pdo.yaml file with discovered PDO mappings.",
)
def decode_candump(
    candump_path: str,
    eds: str | None,
    pdo_config: str | None,
) -> None:
    """Decode a candump log file offline."""
    sniffer.decode_candump(candump_path, eds, pdo_config)


@cli.command("pcap2candump")
@click.argument(
    "pcap_path",
    type=click.Path(exists=True, dir_okay=False),
    metavar="PCAP",
)
@click.option(
    "-o",
    "--output",
    default=None,
    type=click.Path(dir_okay=False),
    help="Output log file.  [default: PCAP with .log suffix]",
)
@click.option(
    "--interface",
    default="can0",
    show_default=True,
    help="Interface name written into each log line.",
)
def pcap2candump_cmd(pcap_path: str, output: str | None, interface: str) -> None:
    """Convert a pcap capture of SocketCAN traffic to candump -ta format.

    Accepts classic pcap files with Linux SLL v1 ("tcpdump -i any"; tx-queue
    duplicates of frames sent by the capturing host are dropped) or raw
    SocketCAN ("tcpdump -i can0") link types.
    """
    pcap2candump.convert(pcap_path, output, interface)


@cli.command("watch")
@click.option(
    "-i", "--interface", default="can0", show_default=True, help="CAN interface name."
)
@click.option(
    "--eds",
    default=None,
    type=click.Path(exists=True, dir_okay=False),
    help="EDS file for signal decoding.",
)
@click.option(
    "--node-id",
    "node_ids",
    multiple=True,
    type=int,
    required=True,
    help="CANopen node ID to watch (repeatable).",
)
@click.option(
    "--foxglove-port",
    default="",
    help="Port for a Foxglove websocket server (requires the 'foxglove' "
    "extra). Empty disables; default is disabled.",
)
def watch(
    interface: str, eds: str | None, node_ids: tuple[int, ...], foxglove_port: str
) -> None:
    """Watch CANopen traffic and log decoded values to one CSV per node."""
    config = watcher.WatchConfig(node_ids=list(node_ids))
    port: int | None = int(foxglove_port) if foxglove_port else None
    watcher.run(interface, eds, config, foxglove_port=port)


# ---------------------------------------------------------------------------
# PVT / SDO commands
# ---------------------------------------------------------------------------


@cli.command("ping")
@node_options
@click.option(
    "-c",
    "--count",
    default=4,
    show_default=True,
    type=int,
    help="Number of pings (0 = infinite until Ctrl+C).",
)
@click.option(
    "-i",
    "--interval",
    default=1.0,
    show_default=True,
    type=float,
    help="Interval between pings in seconds.",
)
@click.option(
    "--timeout",
    default=1.0,
    show_default=True,
    type=float,
    help="SDO response timeout in seconds.",
)
def ping(
    interface: str, node_id: int, count: int, interval: float, timeout: float
) -> None:
    """Ping a CANopen node via SDO read of Device Type (0x1000)."""
    canopen_utils.ping(interface, node_id, count, interval, timeout)


@cli.command("sdo-read")
@node_options
@click.argument("index", type=str, metavar="INDEX")
@click.argument("subindex", default="0", type=lambda x: int(x, 0))
@click.option(
    "--timeout",
    default=1.0,
    show_default=True,
    type=float,
    help="SDO response timeout in seconds.",
)
@click.option(
    "--eds",
    "eds_path",
    default=None,
    help="EDS file for name lookup.",
)
def sdo_read(
    interface: str,
    node_id: int,
    index: str,
    subindex: int,
    timeout: float,
    eds_path: str | None,
) -> None:
    """Read an SDO object from the node (expedited transfer only).

    INDEX may be a decimal/hex integer, a built-in CiA 402 name, or any
    object/sub-object name from the EDS file.  SUBINDEX defaults to 0 and
    is ignored for named lookups.
    """
    canopen_utils.sdo_read(interface, node_id, index, subindex, timeout, eds_path)


@cli.command("sdo-write")
@node_options
@click.argument("index", type=str, metavar="INDEX")
@click.argument("subindex", default="0", type=lambda x: int(x, 0))
@click.option(
    "--value",
    "-v",
    required=True,
    type=lambda x: int(x, 0),
    help="Value to write (decimal or hex 0x...).",
)
@click.option(
    "--size",
    default="4",
    show_default=True,
    type=click.Choice(["1", "2", "4", "6", "8"]),
    help="Payload size in bytes.",
)
@click.option(
    "--timeout",
    default=1.0,
    show_default=True,
    type=float,
    help="SDO response timeout in seconds.",
)
@click.option(
    "--eds",
    "eds_path",
    default=None,
    help="EDS file for name lookup.",
)
def sdo_write(
    interface: str,
    node_id: int,
    index: str,
    subindex: int,
    value: int,
    size: str,
    timeout: float,
    eds_path: str | None,
) -> None:
    """Write an SDO object on the node.

    INDEX may be a decimal/hex integer, a built-in CiA 402 name, or any
    object/sub-object name from the EDS file.  SUBINDEX defaults to 0 and
    is ignored for named lookups.
    """
    canopen_utils.sdo_write(
        interface, node_id, index, subindex, value, size, timeout, eds_path
    )


@cli.command("nmt-reset")
@node_options
def nmt_reset(interface: str, node_id: int) -> None:
    """Send an NMT Reset Node command to the configured node."""
    canopen_utils.nmt_reset(interface, node_id)


@cli.command("dump-pdo")
@node_options
@click.option(
    "--max-rpdo",
    default=32,
    show_default=True,
    type=int,
    help="Highest RPDO number to probe.",
)
@click.option(
    "--max-tpdo",
    default=32,
    show_default=True,
    type=int,
    help="Highest TPDO number to probe.",
)
@click.option(
    "--eds",
    "eds_path",
    default=None,
    help="Path to AMC FlexPro EDS file.",
)
def dump_pdo(
    interface: str, node_id: int, max_rpdo: int, max_tpdo: int, eds_path: str | None
) -> None:
    """Dump RPDO/TPDO communication and mapping parameters via SDO."""
    canopen_utils.dump_pdo(interface, node_id, max_rpdo, max_tpdo, eds_path)


@cli.command("home")
@node_options
@click.option(
    "--eds",
    "eds_path",
    default=None,
    help="Path to AMC FlexPro EDS file.",
)
def home(interface: str, node_id: int, eds_path: str | None) -> None:
    """Zero the position using homing method 35 (current position as home)."""
    canopen_utils.home(interface, node_id, eds_path)


@cli.command("limp")
@node_options
@click.option(
    "--eds",
    "eds_path",
    default=None,
    help="Path to AMC FlexPro EDS file.",
)
def limp(interface: str, node_id: int, eds_path: str | None) -> None:
    """Disable the drive so the motor turns freely."""
    canopen_utils.limp(interface, node_id, eds_path)


@cli.command("pvt-move")
@node_options
@click.option(
    "--to",
    "to_pos",
    required=True,
    type=int,
    help="Target offset in encoder counts (relative to current position).",
)
@click.option(
    "--time",
    "total_ms",
    required=True,
    type=int,
    help="Total travel time in milliseconds.",
)
@click.option(
    "--segment-ms",
    default=250,
    show_default=True,
    type=int,
    help="PVT segment duration (2-255 ms).",
)
@click.option(
    "--accel-frac",
    default=0.25,
    show_default=True,
    type=float,
    help="Fraction of total time for acceleration phase (0-1).",
)
@click.option(
    "--decel-frac",
    default=0.25,
    show_default=True,
    type=float,
    help="Fraction of total time for deceleration phase (0-1).",
)
@click.option(
    "--threshold",
    default=10,
    show_default=True,
    type=int,
    help="Buffer threshold level: TPDO 24 fires when buffer drops below this.",
)
@click.option(
    "--eds",
    "eds_path",
    required=True,
    type=click.Path(exists=True, dir_okay=False),
    help="Path to AMC FlexPro EDS file.",
)
@click.option(
    "--dry-run",
    is_flag=True,
    default=False,
    help="Print trajectory table and exit without connecting to CAN bus.",
)
@click.option(
    "--sync-interval",
    default=0.125,
    show_default=True,
    type=float,
    help="Interval in seconds between SYNC messages sent during the move.",
)
@click.option(
    "--start-cob-id",
    default="0x500",
    show_default=True,
    type=str,
    callback=lambda _c, _p, v: int(v, 0),
    help="COB-ID for the PVT start broadcast (RPDO 26 / 0x1419.01).",
)
def pvt_move(
    interface: str,
    node_id: int,
    to_pos: int,
    total_ms: int,
    segment_ms: int,
    accel_frac: float,
    decel_frac: float,
    threshold: int,
    eds_path: str,
    dry_run: bool,
    sync_interval: float,
    start_cob_id: int,
) -> None:
    """Execute a trapezoidal PVT move."""
    pvt_demo.move(
        interface,
        node_id,
        to_pos,
        total_ms,
        segment_ms,
        accel_frac,
        decel_frac,
        threshold,
        eds_path,
        dry_run,
        sync_interval,
        start_cob_id,
    )


if __name__ == "__main__":
    cli()
