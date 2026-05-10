"""CiA 402 (DSP-402) state machine constants and helpers.

Vendor-agnostic. Same controlword/statusword/mode codes as epos2_bridge
and copley_bridge. Only the manufacturer-specific OD indices for the
IPM data path differ -- those live in ingenia_ipm.py.
"""

from __future__ import annotations


# --- Standard CiA 402 object indices ------------------------------------------

IDX_CONTROLWORD = 0x6040            # u16 RW
IDX_STATUSWORD = 0x6041             # u16 RO
IDX_MODES_OF_OPERATION = 0x6060     # i8  RW
IDX_MODES_OF_OPERATION_DISPLAY = 0x6061  # i8 RO
IDX_POSITION_ACTUAL = 0x6064        # i32 RO (counts)
IDX_VELOCITY_ACTUAL = 0x606C        # i32 RO (counts/sec on Ingenia)
IDX_TARGET_POSITION = 0x607A        # i32 RW (counts)
IDX_INTERPOLATION_TIME_PERIOD = 0x60C2  # record (subindex 1 = value, 2 = index)


# --- Mode of operation codes (0x6060) -----------------------------------------

MODE_PROFILE_POSITION = 1
MODE_PROFILE_VELOCITY = 3
MODE_PROFILE_TORQUE = 4
MODE_HOMING = 6
MODE_INTERPOLATED_POSITION = 7
MODE_CYCLIC_SYNC_POSITION = 8
MODE_CYCLIC_SYNC_VELOCITY = 9
MODE_CYCLIC_SYNC_TORQUE = 10


# --- Controlword bits / commands (0x6040) -------------------------------------

CW_BIT_SWITCH_ON = 0
CW_BIT_ENABLE_VOLTAGE = 1
CW_BIT_QUICK_STOP = 2
CW_BIT_ENABLE_OPERATION = 3
CW_BIT_NEW_SETPOINT = 4
CW_BIT_RESET_FAULT = 7
CW_BIT_HALT = 8

CW_SHUTDOWN = 0x06
CW_SWITCH_ON = 0x07
CW_ENABLE_OPERATION = 0x0F
CW_DISABLE_VOLTAGE = 0x00
CW_QUICK_STOP = 0x02
CW_FAULT_RESET = 0x80
CW_START_IPM_MOVE = 0x1F     # bit 4 transition starts IPM execution


# --- Statusword bits (0x6041) -------------------------------------------------

SW_BIT_READY_TO_SWITCH_ON = 0
SW_BIT_SWITCHED_ON = 1
SW_BIT_OPERATION_ENABLED = 2
SW_BIT_FAULT = 3
SW_BIT_VOLTAGE_ENABLED = 4
SW_BIT_QUICK_STOP = 5
SW_BIT_SWITCH_ON_DISABLED = 6
SW_BIT_WARNING = 7
SW_BIT_REMOTE = 9
SW_BIT_TARGET_REACHED = 10
SW_BIT_INTERNAL_LIMIT = 11
SW_BIT_IPM_ACTIVE = 12


def sw_faulted(statusword: int) -> bool:
    return bool((statusword >> SW_BIT_FAULT) & 0x1)


def sw_operation_enabled(statusword: int) -> bool:
    # Standard CiA-402 operation-enabled state mask. Accepts vendor bits like 0x4237/0x4637.
    return (int(statusword) & 0x006F) == 0x0027


def sw_ipm_active(statusword: int) -> bool:
    return bool((statusword >> SW_BIT_IPM_ACTIVE) & 0x1)


def sw_target_reached(statusword: int) -> bool:
    return bool((statusword >> SW_BIT_TARGET_REACHED) & 0x1)
