"""CiA 402 signal decoders and annotation helpers for CANopen objects."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

_WordDecoder = Callable[[int], str]

_ACTUAL_VELOCITY_SCALE_FACTOR = 2**17 / 20_000  # DS1
_ACTUAL_CURRENT_SCALE_FACTOR = 2**14 / 12  # DC1: 2^14/K_p

# CiA 402 StatusWord state: check (value & mask) == pattern in priority order.
_STATUSWORD_STATES: list[tuple[int, int, str]] = [
    (0x4F, 0x0F, "Fault reaction active"),
    (0x4F, 0x08, "Fault"),
    (0x6F, 0x07, "Quick stop active"),
    (0x6F, 0x27, "Operation enabled"),
    (0x6F, 0x23, "Switched on"),
    (0x6F, 0x21, "Ready to switch on"),
    (0x4F, 0x40, "Switch on disabled"),
    (0x4F, 0x00, "Not ready to switch on"),
]

_STATUSWORD_FLAGS: list[tuple[int, str]] = [
    (1 << 4, "VoltageEnabled"),
    (1 << 7, "Warning"),
    (1 << 10, "TargetReached"),
    (1 << 11, "InternalLimitActive"),
    (1 << 12, "HomingComplete"),
]

# CiA 402 ControlWord command: check (value & mask) == pattern in priority order.
_CONTROLWORD_CMDS: list[tuple[int, int, str]] = [
    (0x80, 0x80, "Fault reset"),
    (0x8F, 0x0F, "Enable operation"),
    (0x87, 0x07, "Switch on"),
    (0x87, 0x06, "Shutdown"),
    (0x86, 0x02, "Quick stop"),
    (0x82, 0x00, "Disable voltage"),
]

# CiA 402 Modes of Operation values (0x6060 / 0x6061).
_MODES_OF_OPERATION: dict[int, str] = {
    0: "No mode",
    1: "PP",
    2: "VL",
    3: "PV",
    4: "TQ",
    6: "HM",
    7: "IP",
    8: "CSP",
    9: "CSV",
    10: "CST",
}


def decode_statusword(value: int) -> str:
    """Decode a CiA 402 StatusWord integer to a human-readable string."""
    state = next(
        (
            name
            for mask, pattern, name in _STATUSWORD_STATES
            if (value & mask) == pattern
        ),
        f"0x{value & 0x6F:02X}",
    )
    flags = [name for bit, name in _STATUSWORD_FLAGS if value & bit]
    parts = [f"State={state}", *flags]
    return " | ".join(parts)


def decode_controlword(value: int) -> str:
    """Decode a CiA 402 ControlWord integer to a human-readable string."""
    for mask, pattern, name in _CONTROLWORD_CMDS:
        if (value & mask) == pattern:
            return f"Cmd={name}"
    return f"Cmd=0x{value:02X}"


def decode_modes_of_operation(value: int) -> str:
    """Decode a CiA 402 Modes of Operation (signed INT8) to a human-readable string."""
    signed = value if value < 128 else value - 256
    name = _MODES_OF_OPERATION.get(signed)
    if name is None:
        return f"Mode={signed}"
    return f"Mode={signed}({name})"


# CiA 301 EMCY error codes (high byte = error class, low byte = error code).
_EMCY_ERRORS: dict[int, str] = {
    0x0000: "Error reset",
    0x1000: "Generic error",
    0x2000: "Current",
    0x2100: "Current (device input)",
    0x2200: "Current (inside device)",
    0x2300: "Current (device output)",
    0x3000: "Voltage",
    0x3100: "Voltage (mains)",
    0x3200: "Voltage (inside device)",
    0x3300: "Voltage (device output)",
    0x4000: "Temperature",
    0x4100: "Temperature (ambient)",
    0x4200: "Temperature (device)",
    0x5000: "Device hardware",
    0x6000: "Device software",
    0x6100: "Software (internal)",
    0x6200: "Software (user)",
    0x6300: "Data set",
    0x7000: "Additional modules",
    0x8000: "Monitoring",
    0x8100: "Communication",
    0x8110: "CAN overrun",
    0x8120: "CAN in error passive",
    0x8130: "Heartbeat / life guarding",
    0x8140: "Bus off recovered",
    0x8150: "CAN-ID collision",
    0x8200: "Protocol error",
    0x8210: "PDO not processed (length)",
    0x8220: "PDO length exceeded",
    0x8230: "DAM MPDO not processed",
    0x8240: "Sync data length",
    0x8250: "RPDO timeout",
    0x9000: "External error",
    0xF000: "Additional functions",
    0xFF00: "Device specific",
}

# CiA 301 error register bits (object 0x1001).
_ERROR_REGISTER_BITS: list[tuple[int, str]] = [
    (0x01, "Generic"),
    (0x02, "Current"),
    (0x04, "Voltage"),
    (0x08, "Temperature"),
    (0x10, "Communication"),
    (0x20, "DeviceProfile"),
    (0x80, "Manufacturer"),
]


def decode_emcy(data: bytes | bytearray) -> str:
    """Decode a CANopen EMCY frame payload (8 bytes) to a human-readable string.

    Args:
        data: Raw EMCY frame data bytes (at least 3 bytes: error code + error register).

    Returns:
        Formatted string like ``'0x8130 [Heartbeat / life guarding]  reg=0x01 [Generic]  vendor=0000000000'``.
    """
    if len(data) < 3:
        return f"(short) {data.hex()}"
    error_code = int.from_bytes(data[0:2], "little")
    error_register = data[2]
    vendor = data[3:].hex() if len(data) > 3 else ""

    # Match error class (high byte first, then full code).
    error_name = _EMCY_ERRORS.get(error_code) or _EMCY_ERRORS.get(error_code & 0xFF00)
    code_str = f"0x{error_code:04X}"
    if error_name:
        code_str += f" [{error_name}]"

    reg_flags = [name for bit, name in _ERROR_REGISTER_BITS if error_register & bit]
    reg_str = f"0x{error_register:02X}"
    if reg_flags:
        reg_str += f" [{','.join(reg_flags)}]"

    parts = [f"EMCY {code_str}", f"reg={reg_str}"]
    if vendor:
        parts.append(f"vendor={vendor}")
    return "  ".join(parts)


# AMC-specific EMCY error descriptions for RPDO errors (bits 0-3 of error code byte).
_AMC_RPDO_ERRORS: dict[int, str] = {
    0: "RPDO cannot be processed",
    1: "General error",
    2: "Object does not exist",
    3: "Not writable / not readable",
    4: "Access unsupported in present state",
    5: "Not enough space in PDO for object data",
    6: "Data integrity error",
    7: "Internal write error",
}


def decode_emcy_amc(data: bytes | bytearray) -> str:
    """Decode an AMC FlexPro EMCY frame payload to a human-readable string.

    AMC EMCY layout (8 bytes):
      bytes 0-2: always 0x00
      byte 3:    AMC error code
      bytes 4-7: error-specific context

    Error codes:
      0x00  PVT Sequence Counter Error — bytes 4-7: required counter value (uint32 LE)
      0x01  PVT Cannot Be Started — internal use
      0x02  PVT Buffer Underflow
      0x80-0xFF  RPDO Cannot Be Processed:
                   bits 6-4: (value + 1) = subindex of RPDO mapping parameter
                   bits 3-0: error description (see _AMC_RPDO_ERRORS)
                   bytes 4-5: COB-ID of the offending RPDO (uint16 LE)

    Args:
        data: Raw EMCY frame data bytes (at least 4 bytes).

    Returns:
        Formatted human-readable string.
    """
    if len(data) < 4:
        return f"AMC EMCY (short frame): {data.hex()}"

    error_code = data[3]
    context = data[4:] if len(data) > 4 else b""

    if error_code == 0x00:
        counter = int.from_bytes(context[:4], "little") if len(context) >= 4 else None
        suffix = f"  required_counter={counter}" if counter is not None else ""
        return f"AMC EMCY  PVT Sequence Counter Error{suffix}"

    if error_code == 0x01:
        return "AMC EMCY  PVT Cannot Be Started"

    if error_code == 0x02:
        return "AMC EMCY  PVT Buffer Underflow"

    if error_code & 0x80:
        subindex = ((error_code >> 4) & 0x07) + 1
        desc_key = error_code & 0x0F
        desc = _AMC_RPDO_ERRORS.get(desc_key, f"error={desc_key:#x}")
        cob_id = int.from_bytes(context[:2], "little") if len(context) >= 2 else None
        cob_str = f"  rpdo_cob_id=0x{cob_id:03X}" if cob_id is not None else ""
        return f"AMC EMCY  RPDO error  mapping_subindex={subindex}  {desc}{cob_str}"

    return f"AMC EMCY  code=0x{error_code:02X}  context={context.hex()}"


def decode_int32(value: int) -> str:
    """Reinterpret an unsigned 32-bit value as signed INT32 and return as string."""
    signed = value if value < 0x80000000 else value - 0x100000000
    return str(signed)


def decode_int16(value: int) -> str:
    signed = value if value < 0x8000 else value - 0x10000
    return str(signed)


def decode_actual_current(value: int) -> str:
    signed = value if value < 0x8000 else value - 0x10000
    signed /= _ACTUAL_CURRENT_SCALE_FACTOR
    return f"{signed:0.3f}"


def decode_actual_velocity(value: int) -> str:
    signed = value if value < 0x80000000 else value - 0x100000000
    signed /= _ACTUAL_VELOCITY_SCALE_FACTOR
    return f"{signed:.0f}"


# Mapping from well-known lowercase CiA 402 parameter names to (index, subindex).
# Names match the keys in _WORD_DECODERS so that annotation also works by name.
NAME_TO_INDEX: dict[str, tuple[int, int]] = {
    "controlword": (0x6040, 0x00),
    "statusword": (0x6041, 0x00),
    "modes of operation": (0x6060, 0x00),
    "modes of operation display": (0x6061, 0x00),
    "actual position": (0x6064, 0x00),
    "actual velocity": (0x606C, 0x00),
    "target position": (0x607A, 0x00),
    "target velocity": (0x60FF, 0x00),
}


def decode_word(name: str, value: int) -> str:
    hex_val = f"0x{value:X}"
    leaf = name.rsplit(".", 1)[-1].lower()
    decoder = _WORD_DECODERS.get(leaf)
    if decoder is None:
        return hex_val
    return decoder(value)


def annotate_word(name: str, value: int) -> str:
    """Format a named signal value with an optional decoded annotation.

    Matches on the leaf name (after the last ``.'') case-insensitively, so both
    ``ControlWord'' and ``DriveObjects.ControlWord'' resolve to the same decoder.

    Args:
        name: Signal or parameter name.
        value: Raw integer value.

    Returns:
        Formatted string like ``'0x6 [Cmd=Shutdown]'``.
    """
    hex_val = f"0x{value:X}"
    leaf = name.rsplit(".", 1)[-1].lower()
    decoder = _WORD_DECODERS.get(leaf)
    if decoder is None:
        return hex_val
    return f"{hex_val} [{decoder(value)}]"


def annotate_sdo_data(leaf: str | None, data_bytes: bytes) -> str:
    """Format expedited SDO data bytes as a hex integer with optional annotation.

    Args:
        leaf: Leaf parameter name for decoder lookup, or None.
        data_bytes: Raw data bytes (little-endian integer).

    Returns:
        Formatted string like ``'0x637 [State=Operation enabled]'``.
    """
    value = int.from_bytes(data_bytes, "little")
    hex_val = f"0x{value:X}"
    if leaf is None:
        return hex_val
    decoder = _WORD_DECODERS.get(leaf.lower())
    if decoder is None:
        return hex_val
    return f"{hex_val} [{decoder(value)}]"


def annotate_by_index(index: int, subindex: int, data_bytes: bytes) -> str | None:
    """Return a decoded annotation for a known CiA 402 object, or None.

    Uses a built-in table of well-known object indices so no EDS is required.

    Args:
        index: Object dictionary index.
        subindex: Object dictionary sub-index.
        data_bytes: Raw value bytes (little-endian).

    Returns:
        Annotation string (e.g. ``'State=Operation enabled | TargetReached'``)
        or ``None`` if the object is not in the built-in table.
    """
    decoder = _INDEX_DECODERS.get((index, subindex))
    if decoder is None:
        return None
    value = int.from_bytes(data_bytes, "little")
    return decoder(value)


@dataclass
class PvtStatus:
    """Decoded AMC PVT Status word (object 0x201D.01h, 16 bits).

    Attributes:
        raw: Raw 16-bit value.
        buf_full: Buffer is full.
        buf_empty: Buffer is empty.
        buf_threshold: Buffer has dropped to or below the threshold level.
        buf_failure: Problem reading a point from the buffer.
        buf_empty_stop: Buffer empty and last PVT point reached.
        seq_error: PVT point sequence error.
        executing: PVT buffer is presently executing.
    """

    raw: int
    buf_full: bool
    buf_empty: bool
    buf_threshold: bool
    buf_failure: bool
    buf_empty_stop: bool
    seq_error: bool
    executing: bool

    def describe(self) -> str:
        """Describe the active flags as a space-separated list (no hex prefix)."""
        flags = [
            name
            for flag, name in [
                (self.buf_full, "BufFull"),
                (self.buf_empty, "BufEmpty"),
                (self.buf_threshold, "BufThreshold"),
                (self.buf_failure, "BufFailure"),
                (self.buf_empty_stop, "BufEmptyStop"),
                (self.seq_error, "SeqError"),
                (self.executing, "Executing"),
            ]
            if flag
        ]
        return " ".join(flags) or "none"

    def __str__(self) -> str:
        return f"0x{self.raw:04X} [{self.describe()}]"


def decode_pvt_status(data: bytes | bytearray) -> PvtStatus:
    """Decode an AMC PVT Status word from TPDO 24 payload bytes.

    Interprets the first two bytes as object 0x201D.01h (PVT Status, UINT16 LE).

    Args:
        data: Raw TPDO payload (at least 2 bytes).

    Returns:
        Decoded PvtStatus dataclass.
    """
    raw = int.from_bytes(data[:2], "little")
    return PvtStatus(
        raw=raw,
        buf_full=bool(raw & (1 << 0)),
        buf_empty=bool(raw & (1 << 1)),
        buf_threshold=bool(raw & (1 << 2)),
        buf_failure=bool(raw & (1 << 3)),
        buf_empty_stop=bool(raw & (1 << 4)),
        seq_error=bool(raw & (1 << 5)),
        executing=bool(raw & (1 << 6)),
    )


@dataclass
class PvtQuickStatus:
    """Decoded AMC PVT Quick Status word (object 0x200C.01h, 16 bits).

    Attributes:
        raw: Raw 16-bit value.
        points_in_drive: Number of PVT points currently in the drive (bits 0–4).
        zero_speed: Measured speed is within the zero-velocity window.
        at_command: Measured position is within the at-command window.
        homing_active: Homing algorithm is running.
        homing_complete: Homing algorithm has completed.
        bridge_enabled: Motor bridge is active and driving current.
        brake_enabled: Brake action is active.
        stop: Stop action is active.
        pvt_executing: PVT trajectory generator is sequencing through points.
    """

    raw: int
    points_in_drive: int
    zero_speed: bool
    at_command: bool
    homing_active: bool
    homing_complete: bool
    bridge_enabled: bool
    brake_enabled: bool
    stop: bool
    pvt_executing: bool

    def describe(self) -> str:
        """Describe points-in-drive count and active flags (no hex prefix)."""
        flags = [
            name
            for flag, name in [
                (self.zero_speed, "ZeroSpeed"),
                (self.at_command, "AtCommand"),
                (self.homing_active, "HomingActive"),
                (self.homing_complete, "HomingComplete"),
                (self.bridge_enabled, "BridgeEnabled"),
                (self.brake_enabled, "BrakeEnabled"),
                (self.stop, "Stop"),
                (self.pvt_executing, "PvtExecuting"),
            ]
            if flag
        ]
        return f"pts={self.points_in_drive} {' '.join(flags) or 'none'}"

    def __str__(self) -> str:
        return f"0x{self.raw:04X} [{self.describe()}]"


def decode_pvt_quick_status(data: bytes | bytearray) -> PvtQuickStatus:
    """Decode an AMC PVT Quick Status word from raw bytes.

    Interprets the first two bytes as object 0x200C.01h (PVT Quick Status, UINT16 LE).

    Args:
        data: Raw bytes (at least 2 bytes).

    Returns:
        Decoded PvtQuickStatus dataclass.
    """
    raw = int.from_bytes(data[:2], "little")
    return PvtQuickStatus(
        raw=raw,
        points_in_drive=raw & 0x1F,
        zero_speed=bool(raw & (1 << 8)),
        at_command=bool(raw & (1 << 9)),
        homing_active=bool(raw & (1 << 10)),
        homing_complete=bool(raw & (1 << 11)),
        bridge_enabled=bool(raw & (1 << 12)),
        brake_enabled=bool(raw & (1 << 13)),
        stop=bool(raw & (1 << 14)),
        pvt_executing=bool(raw & (1 << 15)),
    )


def decode_pvt_status_word(value: int) -> str:
    """Decode a PVT Status word value (201D.01h) as a flag-list string."""
    return decode_pvt_status(value.to_bytes(2, "little")).describe()


def decode_pvt_quick_status_word(value: int) -> str:
    """Decode a PVT Quick Status word value (200C.01h) as a flag-list string."""
    return decode_pvt_quick_status(value.to_bytes(2, "little")).describe()


# Registry keyed by lowercase parameter name.
_WORD_DECODERS: dict[str, _WordDecoder] = {
    "controlword": decode_controlword,
    "statusword": decode_statusword,
    "modes of operation": decode_modes_of_operation,
    "modes of operation display": decode_modes_of_operation,
    "target position": decode_int32,
    "actual velocity": decode_actual_velocity,
    "actual position": decode_int32,
    "actual current": decode_actual_current,
    "pvt status": decode_pvt_status_word,
    "pvt quick status": decode_pvt_quick_status_word,
}


def _snake_channel(name: str) -> str:
    """Convert a signal name to a snake_case channel base.

    Uses the leaf after the last '.' so dotted record names collapse to their
    leaf, then lowercases and replaces spaces with underscores.
    """
    leaf = name.rsplit(".", 1)[-1]
    return leaf.lower().replace(" ", "_")


def _expand_pvt_status(base: str, value: int) -> dict[str, object]:
    ps = decode_pvt_status(value.to_bytes(2, "little"))
    return {
        f"{base}.raw": ps.raw,
        f"{base}.buf_full": int(ps.buf_full),
        f"{base}.buf_empty": int(ps.buf_empty),
        f"{base}.buf_threshold": int(ps.buf_threshold),
        f"{base}.buf_failure": int(ps.buf_failure),
        f"{base}.buf_empty_stop": int(ps.buf_empty_stop),
        f"{base}.seq_error": int(ps.seq_error),
        f"{base}.executing": int(ps.executing),
    }


def _expand_pvt_quick_status(base: str, value: int) -> dict[str, object]:
    qs = decode_pvt_quick_status(value.to_bytes(2, "little"))
    return {
        f"{base}.raw": qs.raw,
        f"{base}.pts": qs.points_in_drive,
        f"{base}.zero_speed": int(qs.zero_speed),
        f"{base}.at_command": int(qs.at_command),
        f"{base}.homing_active": int(qs.homing_active),
        f"{base}.homing_complete": int(qs.homing_complete),
        f"{base}.bridge_enabled": int(qs.bridge_enabled),
        f"{base}.brake_enabled": int(qs.brake_enabled),
        f"{base}.stop": int(qs.stop),
        f"{base}.pvt_executing": int(qs.pvt_executing),
    }


def _expand_statusword(base: str, value: int) -> dict[str, object]:
    state = next(
        (
            name
            for mask, pattern, name in _STATUSWORD_STATES
            if (value & mask) == pattern
        ),
        f"0x{value & 0x6F:02X}",
    )
    out: dict[str, object] = {f"{base}.raw": value, f"{base}.state": state}
    for bit, fname in _STATUSWORD_FLAGS:
        out[f"{base}.{fname.lower()}"] = int(bool(value & bit))
    return out


def _expand_controlword(base: str, value: int) -> dict[str, object]:
    cmd = next(
        (
            name
            for mask, pattern, name in _CONTROLWORD_CMDS
            if (value & mask) == pattern
        ),
        f"0x{value:02X}",
    )
    return {f"{base}.raw": value, f"{base}.cmd": cmd}


def _decode_numeric(leaf: str, value: int) -> int | float:
    """Return a typed decoded value for known scalar leaves, else raw int."""
    if leaf in ("actual position", "target position", "target velocity"):
        return value if value < 0x80000000 else value - 0x100000000
    if leaf == "actual velocity":
        signed = value if value < 0x80000000 else value - 0x100000000
        return signed / _ACTUAL_VELOCITY_SCALE_FACTOR
    if leaf == "actual current":
        signed = value if value < 0x8000 else value - 0x10000
        return signed / _ACTUAL_CURRENT_SCALE_FACTOR
    if leaf in ("modes of operation", "modes of operation display"):
        return value if value < 128 else value - 256
    return value


def expand_emcy(data: bytes | bytearray) -> dict[str, object]:
    """Fan out an EMCY frame payload into sink channels.

    Always emits ``emcy.error_code`` (CiA 301 uint16, bytes 0–1) and
    ``emcy.error_register`` (uint8, byte 2).  AMC FlexPro overloads byte 3 with
    its own error code, so we expose that as ``emcy.amc_code``.  ``emcy.text``
    holds a human-readable single-line summary; AMC's flavor is preferred when
    the standard error_code is zero (AMC convention), otherwise the CiA decoder.

    Args:
        data: Raw EMCY payload bytes (typically 8).

    Returns:
        Mapping of channel name to value.
    """
    out: dict[str, object] = {"emcy.raw": bytes(data).hex()}
    if len(data) >= 2:
        out["emcy.error_code"] = int.from_bytes(data[0:2], "little")
    if len(data) >= 3:
        out["emcy.error_register"] = data[2]
    if len(data) >= 4:
        out["emcy.amc_code"] = data[3]
    if len(data) >= 3:
        cia_code = int.from_bytes(data[0:2], "little")
        if cia_code == 0 and len(data) >= 4:
            out["emcy.text"] = decode_emcy_amc(data)
        else:
            out["emcy.text"] = decode_emcy(data)
    return out


def expand_to_fields(name: str, value: int) -> dict[str, object]:
    """Fan out a decoded PDO signal into one or more (channel, value) entries.

    Bitfields (StatusWord, ControlWord, PVT Status, PVT Quick Status) expand
    into multiple entries including a ``<base>.raw`` key. Other signals return
    a single ``{base: typed_value}`` entry, where ``typed_value`` is a
    sign-extended int or scaled float for known scalars and the raw int
    otherwise.

    Args:
        name: Signal or parameter name (may include dotted record prefix).
        value: Raw integer value.

    Returns:
        Mapping of channel name to typed value.
    """
    base = _snake_channel(name)
    leaf = name.rsplit(".", 1)[-1].lower()

    if leaf == "pvt status":
        return _expand_pvt_status(base, value)
    if leaf == "pvt quick status":
        return _expand_pvt_quick_status(base, value)
    if leaf == "statusword":
        return _expand_statusword(base, value)
    if leaf == "controlword":
        return _expand_controlword(base, value)
    return {base: _decode_numeric(leaf, value)}


# Registry keyed by (index, subindex) for raw SDO reads without EDS.
_INDEX_DECODERS: dict[tuple[int, int], _WordDecoder] = {
    (0x6040, 0x00): decode_controlword,
    (0x6041, 0x00): decode_statusword,
    (0x6060, 0x00): decode_modes_of_operation,
    (0x6061, 0x00): decode_modes_of_operation,
    (0x6064, 0x00): decode_int32,
    (0x606C, 0x00): decode_int32,
    (0x607A, 0x00): decode_int32,
    (0x60FF, 0x00): decode_int32,
    (0x200C, 0x01): decode_pvt_quick_status_word,
    (0x201D, 0x01): decode_pvt_status_word,
}
