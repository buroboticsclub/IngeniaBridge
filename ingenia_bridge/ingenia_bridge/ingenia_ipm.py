"""Ingenia-specific IPM (Interpolated Position Mode) helpers.

Unlike Copley (one packed 64-bit segment object 0x2010) and Maxon
(0x20C1), Ingenia exposes the IPM segment as three SEPARATE
manufacturer objects in their object dictionary:

  0x2181  Interpolation data record - Position input    (INT32, counts)
  0x2182  Interpolation data record - Velocity input    (FLOAT32, rev/sec)
  0x2183  Interpolation data record - Time input        (UINT32, units of 0x60C2)
  0x2184  Interpolation data record integrity check     (UINT32, write-to-commit)
  0x2185  Interpolation data record status              (UINT32, status word)

Plus the buffer state objects:

  0x21F7  Interpolation buffer size                     (UINT16)
  0x21F8  Interpolation buffer number of elements       (UINT16, RO)
  0x21F9  Interpolation buffer maximum size             (UINT16)
  0x21EF  Interpolation time mantissa                   (FLOAT32, currently 1.0)
  0x21F0  Interpolation time exponent                   (FLOAT32, currently 0.001 = 1ms)

Because the segment payload is 12 bytes total (4+4+4) and a CAN frame
is at most 8 bytes, the bridge needs TWO RPDOs per segment:

  RPDO1  ->  0x2181 (pos, 32) + 0x2183 (time, 32)        = 8 bytes
  RPDO3  ->  0x2182 (vel float, 32) + 0x2184 (commit, 32) = 8 bytes

Send order per segment: RPDO1 (stages pos+time), then RPDO3 (writes
velocity AND commits via the integrity-check write). The integrity
write to 0x2184 is the trigger that pushes the staged segment into
the drive's interpolation buffer.

NOTE: the exact semantics of 0x2184 (does it just need to be written,
or does the drive validate the value somehow?) are not in the public
docs we have. The bridge supports several integrity strategies via
the ``integrity_check_mode`` parameter so we can pick the one that
the drive accepts on the bench:

  "counter"   - monotonic 32-bit counter starting at 1 (default guess)
  "constant"  - always write 1
  "echo_pos"  - write the segment's position value (Ingenia drives
                 from other product lines have used this)

Velocity is FLOAT32 in revolutions per second, packed little-endian
IEEE 754. Position is INT32 in encoder counts. Time is UINT32 in
units of 0x60C2 (currently 1 ms by default).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import Enum


# --- Ingenia OD indices used by the bridge ------------------------------------

IDX_IPM_POSITION_INPUT = 0x2181        # i32 RW
IDX_IPM_VELOCITY_INPUT = 0x2182        # f32 RW
IDX_IPM_TIME_INPUT = 0x2183            # u32 RW
IDX_IPM_INTEGRITY_CHECK = 0x2184       # u32 RW (write-to-commit)
IDX_IPM_RECORD_STATUS = 0x2185         # u32 RO

IDX_IPM_BUFFER_CLEAR = 0x21F6          # u16 WO -- any write clears buffer
IDX_IPM_BUFFER_SIZE = 0x21F7           # u16 RW
IDX_IPM_BUFFER_NUM_ELEMENTS = 0x21F8   # u16 RO
IDX_IPM_BUFFER_MAX_SIZE = 0x21F9       # u16 RO
IDX_IPM_TIME_MANTISSA = 0x21EF         # f32 RW
IDX_IPM_TIME_EXPONENT = 0x21F0         # f32 RW

IDX_INGENIA_MFG_CONTROLWORD = 0x2010   # mirror of 0x6040
IDX_INGENIA_MFG_STATUSWORD = 0x2011    # mirror of 0x6041
IDX_INGENIA_MFG_MODE_SELECT = 0x2014   # Ingenia-specific mode index
IDX_INGENIA_MFG_MODE_DISPLAY = 0x2015


# Default-PDO COB-ID base. Ingenia drives use the standard CiA 301 layout.
def cob_rpdo1(node_id: int) -> int: return 0x200 + node_id
def cob_rpdo2(node_id: int) -> int: return 0x300 + node_id
def cob_rpdo3(node_id: int) -> int: return 0x400 + node_id
def cob_rpdo4(node_id: int) -> int: return 0x500 + node_id
def cob_tpdo1(node_id: int) -> int: return 0x180 + node_id
def cob_tpdo2(node_id: int) -> int: return 0x280 + node_id
def cob_tpdo3(node_id: int) -> int: return 0x380 + node_id
def cob_tpdo4(node_id: int) -> int: return 0x480 + node_id
def cob_emcy(node_id: int) -> int: return 0x080 + node_id
def cob_heartbeat(node_id: int) -> int: return 0x700 + node_id


class IntegrityMode(str, Enum):
    COUNTER = "counter"
    CONSTANT = "constant"
    ECHO_POS = "echo_pos"


@dataclass
class IPMPoint:
    """A single trajectory waypoint in motor-native units.

    position_qc:   absolute position in encoder counts (int32)
    velocity_rev_s: motor velocity in revolutions per second (float32)
    time_units:    segment duration in 0x60C2 units (default 1 ms each)
    """
    position_qc: int
    velocity_rev_s: float
    time_units: int


class IntegrityCounter:
    """Monotonic 32-bit counter for the integrity-check field.

    Starts at 1 (zero is reserved as 'never written' / 'reset' on most
    Ingenia drives we've seen). Wraps modulo 2**32.
    """

    def __init__(self, start: int = 1) -> None:
        self._value = int(start) & 0xFFFF

    @property
    def value(self) -> int:
        return self._value

    def next(self) -> int:
        v = self._value
        self._value = (self._value + 1) & 0xFFFF
        if self._value == 0:
            self._value = 1
        return v

    def reset(self, start: int = 1) -> None:
        self._value = int(start) & 0xFFFF


def to_signed_32(v: int) -> int:
    v &= 0xFFFFFFFF
    if v & 0x80000000:
        return v - (1 << 32)
    return v


def pack_pos_time_frame(point: IPMPoint) -> bytes:
    """Pack RPDO1 payload: position (i32 LE) + time (u32 LE) = 8 bytes.

    Maps:
      bytes 0-3: 0x2181 position input (signed, counts)
      bytes 4-7: 0x2183 time input (unsigned, 0x60C2 units)
    """
    return struct.pack("<iH", int(point.position_qc), int(point.time_units) & 0xFFFF)


def pack_vel_commit_frame(point: IPMPoint, integrity: int) -> bytes:
    """Pack RPDO3 payload: velocity (f32 LE) + integrity (u32 LE) = 8 bytes.

    Maps:
      bytes 0-3: 0x2182 velocity input (IEEE 754 float, rev/sec)
      bytes 4-7: 0x2184 integrity check (write triggers segment commit)
    """
    return struct.pack("<fH", float(point.velocity_rev_s), int(integrity) & 0xFFFF)


def compute_integrity(mode: IntegrityMode, counter: IntegrityCounter, point: IPMPoint) -> int:
    """Pick the integrity-check value per the configured strategy."""
    if mode == IntegrityMode.CONSTANT:
        return 1
    if mode == IntegrityMode.ECHO_POS:
        return int(point.position_qc) & 0xFFFF
    return counter.next()


@dataclass
class IPMRecordStatus:
    """Decoded view of 0x2185 (interpolation data record status).

    The exact bit layout is not in the public docs we have, so this
    struct only stores the raw value plus a derived ``ok`` flag using
    the most common Ingenia convention (0 = accepted, non-zero = error).
    Refine once we observe real values on the bench.
    """
    raw: int

    @property
    def ok(self) -> bool:
        return self.raw == 0


def decode_record_status(value: int) -> IPMRecordStatus:
    return IPMRecordStatus(raw=int(value) & 0xFFFF)
