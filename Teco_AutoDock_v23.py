import subprocess
import cv2
import numpy as np
import math
import time
import serial
import threading
from dynamixel_sdk import *

# ==========================================================
# CAMERA / ARUCO
# ==========================================================

APPROACH_TAG_ID = 0
FINAL_TAG_ID = 1

APPROACH_TAG_SIZE_MM = 30.0
FINAL_TAG_SIZE_MM = 14.0

# Small tag is physically mounted about 20 mm to the RIGHT of true docking center.
# So when docking center is aligned, the small tag should appear right of image center.
FINAL_TAG_X_OFFSET_MM = 20.0
FINAL_TAG_X_OFFSET_SIGN = +1.0

# Measured desired final docking observation from real robot placed at docking pose.
# Log: small tag ID 1, size_px=125.7, err_x=205.5, err_y=100.0, yaw=-8.7 deg.
# Docking logic now uses this measured small-tag pixel target instead of only
# calculating the desired X from the physical 20 mm offset.
USE_MEASURED_FINAL_DOCKING_TARGET = True
FINAL_DOCK_TARGET_SIZE_PX = 125.7
FINAL_DOCK_TARGET_ERR_X_PX = 205.5
FINAL_DOCK_TARGET_ERR_Y_PX = 100.0
FINAL_DOCK_TARGET_YAW_DEG = -8.7

WIDTH = 1640
HEIGHT = 1232
FRAMERATE = 10

ARUCO_DICT = cv2.aruco.DICT_4X4_50

DISPLAY_LIVE_CAMERA = False
RECORD_CAMERA_STREAM = True
VIDEO_OUTPUT_FILE = "teco_docking_stream.mp4"
VIDEO_FOURCC = "mp4v"

# ==========================================================
# TAG SIZE THRESHOLDS FROM YOUR MEASUREMENTS AT 1640 x 1232
# ==========================================================

SMALL_TAG_SWITCH_SIZE_PX = 85.0
LARGE_TAG_SWITCH_SIZE_PX = 170.0

SMALL_TAG_DOCK_READY_SIZE_PX = 120.0

# Small tag must also be reasonably visible/centered before we trust it for switching.
SMALL_TAG_SWITCH_MAX_ABS_ERR_X = 500.0
SMALL_TAG_SWITCH_MAX_ABS_ERR_Y = 250.0

# Dock-ready is stricter: size alone is NOT enough.
SMALL_TAG_DOCK_MAX_ABS_CONTROL_ERR_X = 65.0
SMALL_TAG_DOCK_MAX_ABS_ERR_Y = 130.0  # loosened; Y is no longer a hard docking gate unless DOCK_READY_USE_Y_GATE=True

# Close-range policy: mechanically, docking mainly needs X alignment and enough proximity.
# Camera Y can change when a/A bends lift/lower the body, so do not use Y as a hard
# pass/fail gate at pre-extension or final docking.
PRE_EXTENSION_USE_SMALL_Y_GATE = False
DOCK_READY_USE_Y_GATE = False


# Small-tag search behavior
SEARCH_FOR_SMALL_TAG_IF_LOST = True
SMALL_TAG_SEARCH_MAX_RIGHT_STEPS = 6
SMALL_TAG_SEARCH_SETTLE_SEC = 1.0

# Safety logic:
# If both tags are invisible after search, do NOT inch blindly.
ALLOW_BLIND_INCHING_WITH_NO_TAGS = False

# If large tag is already past switch size but small tag cannot be found,
# do NOT inch more. This prevents driving past docking station.
ALLOW_INCHING_WHEN_LARGE_TAG_IS_CLOSE_BUT_SMALL_TAG_LOST = False

# ==========================================================
# SERIAL PORTS
# ==========================================================

ESP32_PORT = "/dev/ttyUSB1"
ESP32_BAUD = 115200

DXL_PORT = "/dev/ttyUSB0"
DXL_BAUD = 57600
DXL_ID = 1

# ==========================================================
# DYNAMIXEL XL330 EXTENDED POSITION SETTINGS
# ==========================================================

PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
EXTENDED_POSITION_CONTROL_MODE = 4

# ----------------------------------------------------------
# CRITICAL DXL HARD SAFETY LIMITS
# ----------------------------------------------------------

DXL_HARD_MIN_POS = -8500
DXL_HARD_MAX_POS = 52000

FRONT_CLAW_HOME_POS = 1500
FRONT_CLAW_CLOSED_POS = 1500
FRONT_CLAW_OPEN_POS = 51000

# Final docking close position.
FRONT_CLAW_DOCKING_POS = -8000

# Main control for final DXL docking close.
# False = align and stop before closing.
# True  = after dock-ready confirmation, move front DXL claw to FRONT_CLAW_DOCKING_POS.
ENABLE_FINAL_DXL_DOCKING_CLOSE = True

DXL_HOME_TOLERANCE = 500
DXL_CLOSED_TOLERANCE = 500
DXL_OPEN_TOLERANCE = 500
DXL_DOCKING_TOLERANCE = 500

DXL_PROFILE_VELOCITY = 300
DXL_PROFILE_ACCELERATION = 50

DXL_HOME_TIMEOUT_SEC = 120.0
DXL_OPEN_TIMEOUT_SEC = 120.0
DXL_CLOSE_TIMEOUT_SEC = 120.0
DXL_DOCKING_TIMEOUT_SEC = 120.0
DXL_POLL_INTERVAL_SEC = 0.4

# ==========================================================
# ESP32 ROBOT COMMANDS
# ==========================================================

DUMMY_FIRST_CHAR = "x"
ROBOT_ID_COMMAND = "4"

ALIGNMENT_SMALL_STEP_COMMAND = "x"
INCHING_STEP_COMMAND = "i"

# Current confirmed cable orientation:
# a = top cable
# left  = a + s
# right = a + d
LEFT_BEND_SEQUENCE = ["a", "s"]
RIGHT_BEND_SEQUENCE = ["a", "d"]

UNDO_LEFT_SEQUENCE = ["A", "S"]
UNDO_RIGHT_SEQUENCE = ["A", "D"]

EXTEND_SEQUENCE = ["S", "D", "A"]
DEFAULT_EXTEND_REPEAT_COUNT = 4
EXTEND_GROUP_WAIT_SEC = 1.0

USE_ADAPTIVE_EXTENSION_REPEAT = True

HOME_COMMAND = "Q"

# ==========================================================
# STARTUP BODY RESET BEFORE NORMAL V20 LOGIC
# ==========================================================
# This version does NOT do camera search. Before starting the normal v20
# adaptive approach/docking logic, it sends Q once to return the body to home,
# then sends x so all normal v20 alignment code starts in small-displacement mode.
HOME_BODY_BEFORE_ADAPTIVE_APPROACH = True
HOME_BODY_BEFORE_ADAPTIVE_WAIT_SEC = 2.0
SEND_X_AFTER_STARTUP_HOME = True

BACK_CLAW_CLOSE = "e"
BACK_CLAW_OPEN = "E"
BODY_COLLAPSE = "q"

BACK_CLAW_WAIT_SEC = 7.0

# ==========================================================
# APPROACH ALIGNMENT SETTINGS, LARGE TAG ID 0
# ==========================================================

APPROACH_DESIRED_ERR_X = 0.0

APPROACH_CENTER_TOLERANCE_PX = 90.0
APPROACH_MAX_ALIGNMENT_STEPS = 10
APPROACH_MAX_PULSES_SAME_DIRECTION = 3

MIN_IMPROVEMENT_PX = 10.0
WORSE_MARGIN_PX = 40.0

INVERT_LEFT_RIGHT = False

# ==========================================================
# INCHING SETTINGS
# ==========================================================

MAX_INCHING_CYCLES = 4
COLLAPSE_WAIT_SEC = 4.0

# ==========================================================
# FINAL SMALL-TAG ALIGNMENT SETTINGS, TAG ID 1
# ==========================================================

FINAL_DESIRED_ERR_Y = 100.0

FINAL_X_TOLERANCE_PX = 60.0
FINAL_Y_TOLERANCE_PX = 60.0

FINAL_MAX_Y_STEPS = 8
FINAL_MAX_X_STEPS = 8

# From previous output, sending 'a' made raw_err_y worse.
INVERT_FINAL_Y = True

FINAL_Y_UP_SEQUENCE = ["a"]
FINAL_Y_DOWN_SEQUENCE = ["A"]

FINAL_SETTLE_SEC = 1.0
FINAL_FLUSH_SEC = 0.6
FINAL_TAG_SEARCH_TIMEOUT_SEC = 4.0

# From previous output, undoing final X 'a' made the tag disappear.
CLEANUP_EXTRA_A_AFTER_FINAL_X = False

# Control whether large-tag X alignment cleanup sends 'A' to undo accumulated top 'a' pulls.
# False = keep the post-alignment body pose and do NOT correct a-pulls with A.
# True  = restore top-cable balance after large-tag alignment.
CLEANUP_A_PULLS_AFTER_LARGE_ALIGNMENT = True

# Before the inching extension phase (S, D, A groups), optionally do a visual
# alignment pass if the robot is already close enough for the small/large tag
# switch zone. This keeps the robot aligned before extension closes the distance.
ALIGN_BEFORE_EVERY_EXTENSION = True
PRE_EXTENSION_ALIGN_USE_FINAL_SMALL_TAG_ALIGNMENT = True
PRE_EXTENSION_ALIGN_RECHECK_AFTER_LARGE_ALIGNMENT = True

# Stricter alignment tolerances used ONLY before inching extension.
# Normal approach can remain looser, but extension should not push forward
# while the robot is visibly offset.
PRE_EXTENSION_LARGE_X_TOLERANCE_PX = 45.0
PRE_EXTENSION_SMALL_X_TOLERANCE_PX = 35.0
PRE_EXTENSION_SMALL_Y_TOLERANCE_PX = 55.0

# Multiplier applied ONLY to pre-extension alignment thresholds.
#   > 1.0  => tighter alignment because effective tolerance = base / multiplier
#   = 1.0  => use base tolerances above
#   < 1.0  => looser alignment
# Examples:
#   1.5 makes 45 px become 30 px
#   0.75 makes 45 px become 60 px
PRE_EXTENSION_ALIGNMENT_STRICTNESS_MULTIPLIER = 0.75

# Timing: after front DXL claw closes, send one S-D-A group first.
# Then run x + alignment corrections before the remaining S-D-A groups.
FIRST_EXTENSION_GROUP_WITHOUT_ALIGNMENT = True

# If pre-extension alignment fails, abort the rest of this inching cycle cleanly.
# Do not keep sending later S-D-A groups, because that breaks the inching sequence.
ABORT_INCHING_ON_PRE_EXTENSION_ALIGNMENT_FAIL = True

# Pre-extension alignment should be a real gate before S-D-A extension.
# The robot switches to small-displacement mode x before every correction pulse,
# sends a+d / a+s, optionally cleans up accumulated a pulls with A, then
# switches back to inching mode i only if alignment is acceptable.
PRE_EXTENSION_MAX_X_ALIGNMENT_PULSES = 12
PRE_EXTENSION_MIN_X_IMPROVEMENT_PX = 3.0
PRE_EXTENSION_TRY_OPPOSITE_IF_NO_IMPROVEMENT = True
PRE_EXTENSION_SEND_X_BEFORE_EVERY_CORRECTION = True
PRE_EXTENSION_CLEANUP_A_PULLS_AFTER_X_ALIGNMENT = False
BLOCK_EXTENSION_IF_PRE_EXTENSION_ALIGNMENT_FAILS = True

# v20 close-range robustness:
# If the small tag is visible and already acceptable in X, use/pass the small tag
# even when the large tag is also visible. This prevents the controller from
# chasing the large tag a few more pixels after the small tag is already good.
PRE_EXTENSION_SMALL_VISIBLE_X_OVERRIDE_ENABLED = True
PRE_EXTENSION_SMALL_VISIBLE_OVERRIDE_MIN_SIZE_PX = 60.0
PRE_EXTENSION_SMALL_VISIBLE_OVERRIDE_REQUIRE_X_IN_TOL = True


# Targeted robustness patch for the working baseline:
# If the robot was just aligned using the large tag, then tags disappear, and a
# single right-search pulse reveals a dock-size small tag, treat that as a
# visibility-reveal docking candidate. Do not chase the small-tag X error, because
# the search bend itself changes the camera geometry and can make a physically
# aligned robot look X-offset.
VISIBILITY_REVEAL_DOCK_CANDIDATE_ENABLED = True
VISIBILITY_REVEAL_MAX_RIGHT_SEARCH_STEPS = 3
VISIBILITY_REVEAL_MIN_SMALL_SIZE_PX = 120.0
VISIBILITY_REVEAL_MAX_ABS_SMALL_CONTROL_Y_PX = 150.0  # soft sanity check only; X/large history matter more here
VISIBILITY_REVEAL_MIN_RECENT_LARGE_SIZE_PX = 190.0
VISIBILITY_REVEAL_MAX_RECENT_LARGE_ABS_X_PX = 75.0
VISIBILITY_REVEAL_RECENT_LARGE_VALID_SEC = 20.0

# Internal bookkeeping for the right-search that revealed the small tag.
last_small_search_right_steps_taken = 0

# Final-stage small-tag recovery / lock-on behavior.
# Problem seen in logs: the small tag can be found during right search, but then
# the large tag reappears and the controller switches back to the large tag.
# Near the final stage, once the small tag is visible, keep using it for the
# final/pre-extension alignment instead of jumping back to the large tag.
LOCK_ON_SMALL_TAG_WHEN_FOUND = False
SMALL_TAG_LOCK_MIN_SIZE_PX = 60.0

# If the large tag is close but the small tag is missing, try a small right-biased
# search for the small tag instead of only aligning with the large tag.
SEARCH_SMALL_TAG_WHEN_LARGE_IS_CLOSE = False
LARGE_SIZE_TO_FORCE_SMALL_SEARCH_PX = 145.0

# Once we have locked onto the small tag, do not let the pre-extension aligner
# switch target back to the large tag even if the large tag reappears.
PRE_EXTENSION_FORCE_SMALL_TARGET_AFTER_SEARCH = False

# ==========================================================
# INCHING TAG MONITORING / OCCLUDED FINAL DOCKING
# ==========================================================

MONITOR_TAGS_DURING_INCHING = True
INTERRUPT_INCHING_IF_DOCK_CANDIDATE = True

ALLOW_FINAL_DXL_CLOSE_FROM_LAST_CLOSE_TAG_WHEN_OCCLUDED = True
LAST_CLOSE_TAG_VALID_SEC = 20.0

SMALL_TAG_CLOSE_MONITOR_SIZE_PX = 115.0
SMALL_TAG_CLOSE_MONITOR_MAX_ABS_CONTROL_ERR_X = 90.0
SMALL_TAG_CLOSE_MONITOR_MAX_ABS_ERR_Y = 90.0

LARGE_TAG_CLOSE_MONITOR_SIZE_PX = 220.0
LARGE_TAG_CLOSE_MONITOR_MAX_ABS_ERR_X = 120.0

# ==========================================================
# CURRENT-THRESHOLD FINAL DXL DOCKING CLOSE
# ==========================================================

USE_CURRENT_AWARE_DXL_DOCKING_CLOSE = True

# New final docking behavior:
#   keep closing the front claw in small position steps until either:
#   1) measured current reaches DXL_DOCK_STOP_CURRENT_A, or
#   2) the commanded safe target/hard minimum is reached, or
#   3) timeout occurs.
#
# For XL330, ROBOTIS documents Goal Current / Present Current unit as about 1 mA.
# If your measured current does not match real current, tune DXL_DOCK_CURRENT_UNIT_A
# or directly set DXL_DOCK_STOP_CURRENT_RAW_OVERRIDE.
DXL_DOCK_CLOSE_UNTIL_CURRENT = True

# Main force/current knob for final docking.
# Suggested first-test value: 0.40 A. If the dock slips or does not latch, try 0.50 A.
# XL330 present-current units are approximately 1 mA/raw count, so 0.40 A ≈ 400 raw.
DXL_DOCK_STOP_CURRENT_A = 0.40
DXL_DOCK_CURRENT_UNIT_A = 0.001  # A/raw count, about 1 mA per count for XL330
DXL_DOCK_STOP_CURRENT_RAW_OVERRIDE = None  # example: 500. Leave None to use amps.
DXL_DOCK_STOP_CURRENT_CONSECUTIVE_SAMPLES = 2

# If True, final current-threshold close aims at DXL_HARD_MIN_POS.
# This lets you lower DXL_HARD_MIN_POS slightly for a force-applying close while
# still preventing the claw from moving past that hard safety limit.
# If False, it aims at FRONT_CLAW_DOCKING_POS.
DXL_DOCK_FORCE_TARGET_USES_HARD_MIN = True

DXL_DOCK_STEP_COUNTS = 500
DXL_DOCK_CURRENT_POLL_SEC = 0.20
DXL_DOCK_HOLD_POSITION_ON_CURRENT_STOP = True

# After docking contact is detected, keep the script alive and keep torque enabled.
# This does NOT actively regulate exact current; it holds the final position and prints the
# measured holding current until Ctrl+C / script exit. On exit, torque is disabled.
DXL_HOLD_AFTER_DOCKING_SUCCESS = True
DXL_HOLD_STATUS_PRINT_SEC = 0.50
DXL_HOLD_WARN_CURRENT_A = 0.65
DXL_DISABLE_TORQUE_ON_EXIT = True

# ==========================================================
# TIMING / CLEANUP
# ==========================================================

CHAR_DELAY_SEC = 0.08
SETTLE_AFTER_PRIMITIVE_SEC = 1.0
FLUSH_AFTER_MOVE_SEC = 0.6
LOST_TAG_WAIT_SEC = 0.30

HOME_AT_START = False
HOME_AT_END = False
HOME_ON_INTERRUPT = False
DISCONNECT_AT_END = True

# If docking never succeeds, send Q before quitting so the body/cables return home.
SEND_Q_ON_DOCKING_FAILURE = False

# Runtime flag set True only after the final DXL docking close confirms contact/position.
docking_successful = False

top_a_pull_balance = 0
last_inching_interrupt_reason = None

last_known_large_tag = None
last_known_small_tag = None
last_known_tag_time = None
last_known_close_dock_candidate = False
last_known_close_reason = None

# ==========================================================
# HARD SAFETY CHECKS
# ==========================================================


def clamp_dxl_goal(goal_pos):
    goal_pos = int(goal_pos)

    if goal_pos < DXL_HARD_MIN_POS:
        print(
            f"WARNING: requested DXL goal {goal_pos} below hard min "
            f"{DXL_HARD_MIN_POS}. Clamping."
        )
        return DXL_HARD_MIN_POS

    if goal_pos > DXL_HARD_MAX_POS:
        print(
            f"WARNING: requested DXL goal {goal_pos} above hard max "
            f"{DXL_HARD_MAX_POS}. Clamping."
        )
        return DXL_HARD_MAX_POS

    return goal_pos


def assert_dxl_constants_safe():
    targets = {
        "FRONT_CLAW_HOME_POS": FRONT_CLAW_HOME_POS,
        "FRONT_CLAW_CLOSED_POS": FRONT_CLAW_CLOSED_POS,
        "FRONT_CLAW_OPEN_POS": FRONT_CLAW_OPEN_POS,
        "FRONT_CLAW_DOCKING_POS": FRONT_CLAW_DOCKING_POS,
    }

    for name, value in targets.items():
        if not (DXL_HARD_MIN_POS <= value <= DXL_HARD_MAX_POS):
            raise RuntimeError(
                f"{name}={value} is outside DXL hard limits "
                f"[{DXL_HARD_MIN_POS}, {DXL_HARD_MAX_POS}]. Refusing to run."
            )


# ==========================================================
# ESP32 SERIAL FUNCTIONS
# ==========================================================


def send_char(ser, ch):
    if len(ch) != 1:
        raise ValueError(f"Command must be exactly one character, got: {repr(ch)}")

    ser.write(ch.encode("ascii"))
    ser.flush()
    print(f"Sent ESP32: {repr(ch)}")


def send_sequence(ser, sequence, track_top_a_balance=True):
    global top_a_pull_balance

    for ch in sequence:
        send_char(ser, ch)

        if track_top_a_balance:
            if ch == "a":
                top_a_pull_balance += 1
                print(f"top_a_pull_balance = {top_a_pull_balance}")
            elif ch == "A":
                top_a_pull_balance -= 1
                if top_a_pull_balance < 0:
                    top_a_pull_balance = 0
                print(f"top_a_pull_balance = {top_a_pull_balance}")

        time.sleep(CHAR_DELAY_SEC)


def initialize_robot(ser):
    print("\nInitializing ESP32 / robot transmitter...")
    time.sleep(2.0)

    send_char(ser, DUMMY_FIRST_CHAR)
    time.sleep(0.2)

    send_char(ser, ROBOT_ID_COMMAND)
    time.sleep(0.2)

    send_char(ser, ALIGNMENT_SMALL_STEP_COMMAND)
    time.sleep(0.2)

    print("ESP32 robot initialization complete.\n")


def disconnect_robot(ser):
    print("\nDisconnecting robot 4...")
    send_char(ser, ROBOT_ID_COMMAND)
    time.sleep(0.2)
    print("Robot 4 disconnected.\n")


def set_small_alignment_mode(ser):
    print("\nSetting robot back to SMALL alignment displacement mode: x")
    send_char(ser, ALIGNMENT_SMALL_STEP_COMMAND)
    time.sleep(0.5)


def home_body_before_normal_v20_logic(ser, camera=None):
    if not HOME_BODY_BEFORE_ADAPTIVE_APPROACH:
        print("HOME_BODY_BEFORE_ADAPTIVE_APPROACH = False; skipping startup body home.")
        return

    print("\n" + "=" * 70)
    print("STARTUP BODY HOME BEFORE NORMAL V20 DOCKING LOGIC")
    print("Sending Q once to reset/home the body before autonomous approach.")
    print("No camera search will be performed in this version.")
    print("=" * 70)

    send_char(ser, HOME_COMMAND)
    print(f"Waiting {HOME_BODY_BEFORE_ADAPTIVE_WAIT_SEC:.1f} sec after Q...")
    time.sleep(HOME_BODY_BEFORE_ADAPTIVE_WAIT_SEC)

    if SEND_X_AFTER_STARTUP_HOME:
        print("Returning to normal small-displacement alignment mode x before v20 logic.")
        send_char(ser, ALIGNMENT_SMALL_STEP_COMMAND)
        time.sleep(0.5)

    if camera is not None:
        camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)


def cleanup_top_a_pulls(ser):
    global top_a_pull_balance

    print("\nCleaning up accumulated top-cable a-pulls...")
    print(f"Current top_a_pull_balance = {top_a_pull_balance}")

    count = top_a_pull_balance

    for i in range(count):
        print(f"Undoing top a pull {i + 1}/{count}: sending A")
        send_char(ser, "A")
        top_a_pull_balance -= 1
        if top_a_pull_balance < 0:
            top_a_pull_balance = 0
        time.sleep(CHAR_DELAY_SEC)

    print(f"Cleanup done. top_a_pull_balance = {top_a_pull_balance}\n")


def cleanup_top_a_pulls_after_large_alignment(ser):
    if CLEANUP_A_PULLS_AFTER_LARGE_ALIGNMENT:
        cleanup_top_a_pulls(ser)
    else:
        print("\nCLEANUP_A_PULLS_AFTER_LARGE_ALIGNMENT = False")
        print("Keeping accumulated top-cable a-pulls after large-tag alignment.")
        print(f"top_a_pull_balance remains = {top_a_pull_balance}\n")


def cleanup_extra_a_from_final_x(ser, balance_before_final_x):
    global top_a_pull_balance

    if not CLEANUP_EXTRA_A_AFTER_FINAL_X:
        print("\nCLEANUP_EXTRA_A_AFTER_FINAL_X is False.")
        print("Keeping extra top a-pulls from FINAL X alignment.")
        return

    extra_a = top_a_pull_balance - balance_before_final_x

    if extra_a <= 0:
        print("\nNo extra top a-pulls from FINAL X to clean up.")
        return

    print("\nCleaning up only extra top a-pulls added during FINAL X...")
    print(f"Balance before FINAL X: {balance_before_final_x}")
    print(f"Current top_a_pull_balance: {top_a_pull_balance}")
    print(f"Extra a-pulls to undo: {extra_a}")

    for i in range(extra_a):
        print(f"Undoing FINAL X extra a-pull {i + 1}/{extra_a}: sending A")
        send_char(ser, "A")
        top_a_pull_balance -= 1
        if top_a_pull_balance < 0:
            top_a_pull_balance = 0
        time.sleep(CHAR_DELAY_SEC)

    print(f"FINAL X cleanup done. top_a_pull_balance = {top_a_pull_balance}\n")


# ==========================================================
# DYNAMIXEL FUNCTIONS
# ==========================================================


def signed_16(value):
    if value > 0x7FFF:
        value -= 0x10000
    return value


def signed_32(value):
    if value > 0x7FFFFFFF:
        value -= 0x100000000
    return value


def int32_to_uint32(value):
    value = int(value)
    if value < 0:
        value += 1 << 32
    return value


class FrontClawDynamixel:
    def __init__(self, port_name, baud, dxl_id):
        self.port_name = port_name
        self.baud = baud
        self.dxl_id = dxl_id
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(PROTOCOL_VERSION)

    def open(self):
        print(f"\nOpening Dynamixel port {self.port_name}...")

        if not self.port.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port {self.port_name}")

        if not self.port.setBaudRate(self.baud):
            raise RuntimeError(f"Failed to set Dynamixel baud {self.baud}")

        print("Dynamixel port opened.")

        self.write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        time.sleep(0.1)

        print("Setting Dynamixel to EXTENDED POSITION CONTROL MODE.")
        self.write1(ADDR_OPERATING_MODE, EXTENDED_POSITION_CONTROL_MODE)
        time.sleep(0.1)

        self.write4_unsigned(ADDR_PROFILE_ACCELERATION, DXL_PROFILE_ACCELERATION)
        self.write4_unsigned(ADDR_PROFILE_VELOCITY, DXL_PROFILE_VELOCITY)

        self.write1(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        time.sleep(0.2)

        self.print_status()

    def disable_torque(self):
        print("Disabling Dynamixel torque so docking current drops to zero.")
        self.write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def close_port(self):
        if DXL_DISABLE_TORQUE_ON_EXIT:
            try:
                self.disable_torque()
            except Exception as e:
                print(f"Warning: failed to disable Dynamixel torque on exit: {e}")
        else:
            print("DXL_DISABLE_TORQUE_ON_EXIT = False; leaving Dynamixel torque state unchanged.")

        try:
            self.port.closePort()
            print("Dynamixel port closed.")
        except Exception:
            pass

    def check_result(self, dxl_comm_result, dxl_error, label):
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"{label}: {self.packet.getTxRxResult(dxl_comm_result)}")
        if dxl_error != 0:
            raise RuntimeError(f"{label}: {self.packet.getRxPacketError(dxl_error)}")

    def write1(self, addr, value):
        dxl_comm_result, dxl_error = self.packet.write1ByteTxRx(
            self.port,
            self.dxl_id,
            addr,
            int(value)
        )
        self.check_result(dxl_comm_result, dxl_error, f"write1 addr {addr}")

    def write4_unsigned(self, addr, value):
        dxl_comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.port,
            self.dxl_id,
            addr,
            int(value)
        )
        self.check_result(dxl_comm_result, dxl_error, f"write4 unsigned addr {addr}")

    def write4_signed(self, addr, value):
        safe_value = clamp_dxl_goal(value)
        packet_value = int32_to_uint32(safe_value)

        dxl_comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.port,
            self.dxl_id,
            addr,
            packet_value
        )
        self.check_result(dxl_comm_result, dxl_error, f"write4 signed addr {addr}")

        return safe_value

    def read1(self, addr):
        value, dxl_comm_result, dxl_error = self.packet.read1ByteTxRx(
            self.port,
            self.dxl_id,
            addr
        )
        self.check_result(dxl_comm_result, dxl_error, f"read1 addr {addr}")
        return value

    def read2(self, addr):
        value, dxl_comm_result, dxl_error = self.packet.read2ByteTxRx(
            self.port,
            self.dxl_id,
            addr
        )
        self.check_result(dxl_comm_result, dxl_error, f"read2 addr {addr}")
        return value

    def read4(self, addr):
        value, dxl_comm_result, dxl_error = self.packet.read4ByteTxRx(
            self.port,
            self.dxl_id,
            addr
        )
        self.check_result(dxl_comm_result, dxl_error, f"read4 addr {addr}")
        return value

    def present_position(self):
        return signed_32(self.read4(ADDR_PRESENT_POSITION))

    def present_current(self):
        return signed_16(self.read2(ADDR_PRESENT_CURRENT))

    def present_velocity(self):
        return signed_32(self.read4(ADDR_PRESENT_VELOCITY))

    def voltage(self):
        return self.read2(ADDR_PRESENT_INPUT_VOLTAGE) / 10.0

    def temperature(self):
        return self.read1(ADDR_PRESENT_TEMPERATURE)

    def print_status(self):
        try:
            pos = self.present_position()
            current = self.present_current()
            vel = self.present_velocity()
            voltage = self.voltage()
            temp = self.temperature()

            print(
                f"Dynamixel status: pos={pos}, current_raw={current}, "
                f"vel={vel}, V={voltage:.1f}, T={temp}C"
            )

            if pos < DXL_HARD_MIN_POS or pos > DXL_HARD_MAX_POS:
                print(
                    f"WARNING: present position {pos} outside configured hard range "
                    f"[{DXL_HARD_MIN_POS}, {DXL_HARD_MAX_POS}]."
                )

        except Exception as e:
            print(f"Could not read full Dynamixel status: {e}")

    def move_to_and_wait(self, goal_pos, label="", timeout_sec=90.0, tolerance=500):
        goal_pos = clamp_dxl_goal(goal_pos)

        print(f"\nDynamixel move {label}: goal_pos={goal_pos}, tolerance=±{tolerance}")
        commanded_goal = self.write4_signed(ADDR_GOAL_POSITION, goal_pos)

        start = time.time()

        while True:
            pos = self.present_position()
            current = self.present_current()
            vel = self.present_velocity()
            error = commanded_goal - pos

            print(
                f"  pos={pos:8d} | goal={commanded_goal:8d} | "
                f"err={error:8d} | current_raw={current:6d} | vel={vel:6d}"
            )

            if abs(error) <= tolerance:
                print(
                    f"Dynamixel reached {label}: "
                    f"pos={pos}, goal={commanded_goal}, within ±{tolerance}"
                )
                return True

            if time.time() - start > timeout_sec:
                print(
                    f"WARNING: Dynamixel timeout during {label}. "
                    f"Last pos={pos}, goal={commanded_goal}, err={error}, tolerance=±{tolerance}"
                )
                return False

            time.sleep(DXL_POLL_INTERVAL_SEC)

    def home_front_claw_blocking(self):
        ok = self.move_to_and_wait(
            FRONT_CLAW_HOME_POS,
            label="HOME FRONT CLAW",
            timeout_sec=DXL_HOME_TIMEOUT_SEC,
            tolerance=DXL_HOME_TOLERANCE
        )
        if not ok:
            raise RuntimeError("Dynamixel did not reach home position. Refusing to continue.")

    def open_front_claw_blocking(self):
        ok = self.move_to_and_wait(
            FRONT_CLAW_OPEN_POS,
            label="OPEN FRONT CLAW",
            timeout_sec=DXL_OPEN_TIMEOUT_SEC,
            tolerance=DXL_OPEN_TOLERANCE
        )
        if not ok:
            raise RuntimeError("Dynamixel did not reach open position.")

    def close_front_claw_blocking(self):
        ok = self.move_to_and_wait(
            FRONT_CLAW_CLOSED_POS,
            label="CLOSE FRONT CLAW",
            timeout_sec=DXL_CLOSE_TIMEOUT_SEC,
            tolerance=DXL_CLOSED_TOLERANCE
        )
        if not ok:
            raise RuntimeError("Dynamixel did not reach closed/home position.")

    def docking_stop_current_raw(self):
        if DXL_DOCK_STOP_CURRENT_RAW_OVERRIDE is not None:
            return int(DXL_DOCK_STOP_CURRENT_RAW_OVERRIDE)
        return int(round(DXL_DOCK_STOP_CURRENT_A / DXL_DOCK_CURRENT_UNIT_A))

    def docking_close_front_claw_current_aware(self):
        if DXL_DOCK_FORCE_TARGET_USES_HARD_MIN:
            target = DXL_HARD_MIN_POS
            target_label = "DXL_HARD_MIN_POS force target"
        else:
            target = clamp_dxl_goal(FRONT_CLAW_DOCKING_POS)
            target_label = "FRONT_CLAW_DOCKING_POS target"

        target = clamp_dxl_goal(target)
        stop_current_raw = self.docking_stop_current_raw()
        start_time = time.time()
        high_current_samples = 0

        print("\nStarting CURRENT-THRESHOLD final DXL docking close.")
        print(f"Target mode: {target_label}")
        print(f"Target close position: {target}")
        print(f"Hard limits: [{DXL_HARD_MIN_POS}, {DXL_HARD_MAX_POS}]")
        print(
            f"Stop current threshold: {DXL_DOCK_STOP_CURRENT_A:.3f} A "
            f"≈ {stop_current_raw} raw counts "
            f"using {DXL_DOCK_CURRENT_UNIT_A:.6f} A/raw"
        )
        print(f"Consecutive high-current samples required: {DXL_DOCK_STOP_CURRENT_CONSECUTIVE_SAMPLES}")
        print(f"Step size: {DXL_DOCK_STEP_COUNTS} counts")

        pos = self.present_position()
        current_goal = max(target, pos - DXL_DOCK_STEP_COUNTS)
        self.write4_signed(ADDR_GOAL_POSITION, current_goal)

        while True:
            pos = self.present_position()
            current = self.present_current()
            vel = self.present_velocity()
            abs_current = abs(current)
            current_a = abs_current * DXL_DOCK_CURRENT_UNIT_A
            distance_to_target = abs(target - pos)

            if abs_current >= stop_current_raw:
                high_current_samples += 1
            else:
                high_current_samples = 0

            print(
                f"  docking pos={pos:8d} | step_goal={current_goal:8d} | "
                f"target={target:8d} | dist={distance_to_target:6d} | "
                f"current_raw={current:6d} | abs_current≈{current_a:.3f}A | "
                f"high_samples={high_current_samples}/{DXL_DOCK_STOP_CURRENT_CONSECUTIVE_SAMPLES} | "
                f"vel={vel:6d}"
            )

            if high_current_samples >= DXL_DOCK_STOP_CURRENT_CONSECUTIVE_SAMPLES:
                print(
                    "Final DXL docking close stopped because current threshold was reached. "
                    "Treating this as docking contact / applied gripping force."
                )
                if DXL_DOCK_HOLD_POSITION_ON_CURRENT_STOP:
                    print(f"Holding current position: {pos}")
                    self.write4_signed(ADDR_GOAL_POSITION, pos)
                return True

            if distance_to_target <= DXL_DOCKING_TOLERANCE:
                print(
                    "Final DXL docking close reached the safe target/min position before current threshold. "
                    "Stopping at the position limit."
                )
                self.write4_signed(ADDR_GOAL_POSITION, target)
                return True

            if time.time() - start_time > DXL_DOCKING_TIMEOUT_SEC:
                print("WARNING: timeout during current-threshold final docking close. Holding current position.")
                self.write4_signed(ADDR_GOAL_POSITION, pos)
                return False

            current_goal = max(target, pos - DXL_DOCK_STEP_COUNTS)
            self.write4_signed(ADDR_GOAL_POSITION, current_goal)
            time.sleep(DXL_DOCK_CURRENT_POLL_SEC)

    def hold_after_successful_docking_until_interrupt(self):
        if not DXL_HOLD_AFTER_DOCKING_SUCCESS:
            print("DXL_HOLD_AFTER_DOCKING_SUCCESS = False; not entering holding loop.")
            return

        print("\n" + "=" * 70)
        print("DOCKING HOLD LOOP ACTIVE")
        print("Holding the final DXL position with torque enabled.")
        print(f"Target/contact current setting was {DXL_DOCK_STOP_CURRENT_A:.3f} A.")
        print("This is position-hold after contact, not exact active current regulation.")
        print("Press Ctrl+C to stop. The cleanup block will disable DXL torque.")
        print("=" * 70)

        # Freeze the current measured position as the hold goal.
        hold_pos = self.present_position()
        hold_pos = clamp_dxl_goal(hold_pos)
        self.write4_signed(ADDR_GOAL_POSITION, hold_pos)

        while True:
            pos = self.present_position()
            current = self.present_current()
            vel = self.present_velocity()
            voltage = self.voltage()
            temp = self.temperature()
            abs_current_a = abs(current) * DXL_DOCK_CURRENT_UNIT_A

            warn = ""
            if abs_current_a >= DXL_HOLD_WARN_CURRENT_A:
                warn = "  WARNING: holding current is high"

            print(
                f"  HOLDING DOCK | hold_goal={hold_pos:8d} | pos={pos:8d} | "
                f"err={hold_pos - pos:7d} | current_raw={current:6d} | "
                f"abs_current≈{abs_current_a:.3f}A | V={voltage:.1f} | T={temp}C | vel={vel:6d}"
                f"{warn}"
            )

            time.sleep(DXL_HOLD_STATUS_PRINT_SEC)

    def docking_close_front_claw_blocking(self):
        if USE_CURRENT_AWARE_DXL_DOCKING_CLOSE:
            ok = self.docking_close_front_claw_current_aware()
        else:
            ok = self.move_to_and_wait(
                FRONT_CLAW_DOCKING_POS,
                label="FINAL DXL DOCKING CLOSE FRONT CLAW",
                timeout_sec=DXL_DOCKING_TIMEOUT_SEC,
                tolerance=DXL_DOCKING_TOLERANCE
            )

        if not ok:
            raise RuntimeError("Dynamixel final docking close stopped before confirmed docking position/contact.")


# ==========================================================
# CAMERA THREAD / ARUCO
# ==========================================================


def start_rpicam_stream():
    cmd = [
        "rpicam-vid",
        "-t", "0",
        "--width", str(WIDTH),
        "--height", str(HEIGHT),
        "--framerate", str(FRAMERATE),
        "--codec", "mjpeg",
        "--inline",
        "-o", "-"
    ]

    print("Starting camera:")
    print(" ".join(cmd))

    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=0
    )


def create_aruco_detector():
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)

    if hasattr(aruco, "ArucoDetector"):
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)
        return detector, None, dictionary

    parameters = aruco.DetectorParameters_create()
    return None, parameters, dictionary


def detect_all_tags(frame, detector, parameters, dictionary):
    aruco = cv2.aruco

    if detector is not None:
        corners, ids, rejected = detector.detectMarkers(frame)
    else:
        corners, ids, rejected = aruco.detectMarkers(
            frame,
            dictionary,
            parameters=parameters
        )

    results = {}

    if ids is None:
        return results

    h, w = frame.shape[:2]
    image_cx = w / 2.0
    image_cy = h / 2.0

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        pts = marker_corners[0]

        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())

        err_x = cx - image_cx
        err_y = cy - image_cy

        side_lengths = []
        for i in range(4):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % 4]
            side_lengths.append(math.hypot(float(x2 - x1), float(y2 - y1)))

        size_px = sum(side_lengths) / 4.0

        dx = float(pts[1][0] - pts[0][0])
        dy = float(pts[1][1] - pts[0][1])
        yaw_deg = math.degrees(math.atan2(dy, dx))

        results[marker_id] = {
            "id": marker_id,
            "cx": cx,
            "cy": cy,
            "err_x": err_x,
            "err_y": err_y,
            "size_px": size_px,
            "yaw_deg": yaw_deg,
            "corners": pts,
        }

    return results


def detect_target(frame, detector, parameters, dictionary, target_id):
    results = detect_all_tags(frame, detector, parameters, dictionary)
    return results.get(int(target_id), None)


class RpiCamFrameReader:
    def __init__(self):
        self.proc = start_rpicam_stream()
        self.jpeg_buffer = bytearray()

        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_frame_id = 0
        self.running = True

        self.video_writer = None
        if RECORD_CAMERA_STREAM:
            fourcc = cv2.VideoWriter_fourcc(*VIDEO_FOURCC)
            self.video_writer = cv2.VideoWriter(
                VIDEO_OUTPUT_FILE,
                fourcc,
                FRAMERATE,
                (WIDTH, HEIGHT)
            )
            print(f"Recording camera stream to: {VIDEO_OUTPUT_FILE}")

        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _extract_jpeg_frame(self):
        while self.running:
            chunk = self.proc.stdout.read(4096)

            if not chunk:
                return None

            self.jpeg_buffer.extend(chunk)

            start = self.jpeg_buffer.find(b"\xff\xd8")
            end = self.jpeg_buffer.find(b"\xff\xd9")

            if start == -1 or end == -1 or end <= start:
                continue

            jpg = self.jpeg_buffer[start:end + 2]
            self.jpeg_buffer = self.jpeg_buffer[end + 2:]

            frame = cv2.imdecode(
                np.frombuffer(jpg, dtype=np.uint8),
                cv2.IMREAD_COLOR
            )
            return frame

        return None

    def _draw_tags(self, frame):
        out = frame.copy()

        h, w = out.shape[:2]
        image_cx = int(w / 2)
        image_cy = int(h / 2)

        cv2.line(out, (image_cx - 20, image_cy), (image_cx + 20, image_cy), (0, 255, 0), 2)
        cv2.line(out, (image_cx, image_cy - 20), (image_cx, image_cy + 20), (0, 255, 0), 2)

        try:
            dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

            if hasattr(cv2.aruco, "ArucoDetector"):
                parameters = cv2.aruco.DetectorParameters()
                detector = cv2.aruco.ArucoDetector(dictionary, parameters)
                results = detect_all_tags(out, detector, None, dictionary)
            else:
                parameters = cv2.aruco.DetectorParameters_create()
                results = detect_all_tags(out, None, parameters, dictionary)

            for marker_id, r in results.items():
                pts = r["corners"].astype(int)
                cv2.polylines(out, [pts], True, (0, 255, 255), 2)

                cx = int(r["cx"])
                cy = int(r["cy"])

                cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)

                label = (
                    f"ID {marker_id} "
                    f"size={r['size_px']:.0f}px "
                    f"ex={r['err_x']:.0f} "
                    f"ey={r['err_y']:.0f}"
                )

                cv2.putText(
                    out,
                    label,
                    (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

        except Exception:
            pass

        return out

    def _reader_loop(self):
        print("Camera reader thread started. Stream will stay live continuously.")

        while self.running:
            frame = self._extract_jpeg_frame()

            if frame is None:
                time.sleep(0.01)
                continue

            with self.lock:
                self.latest_frame = frame
                self.latest_frame_id += 1

            display_frame = self._draw_tags(frame) if (DISPLAY_LIVE_CAMERA or RECORD_CAMERA_STREAM) else frame

            if self.video_writer is not None:
                try:
                    self.video_writer.write(display_frame)
                except Exception as e:
                    print(f"Video writer error: {e}")

            if DISPLAY_LIVE_CAMERA:
                try:
                    cv2.imshow("TeCoBot Live Camera", display_frame)
                    cv2.waitKey(1)
                except Exception as e:
                    print(f"Display error. Disable DISPLAY_LIVE_CAMERA if headless. Error: {e}")

        print("Camera reader thread stopped.")

    def get_frame_id(self):
        with self.lock:
            return self.latest_frame_id

    def wait_for_new_frame(self, after_frame_id=None, timeout_sec=2.0):
        start = time.time()

        while time.time() - start < timeout_sec:
            with self.lock:
                if self.latest_frame is not None:
                    if after_frame_id is None or self.latest_frame_id > after_frame_id:
                        return self.latest_frame.copy(), self.latest_frame_id

            time.sleep(0.02)

        return None, after_frame_id

    def flush_frames(self, duration_sec=0.6):
        print(f"Letting live camera stream update for {duration_sec:.1f} sec...")
        start_id = self.get_frame_id()
        time.sleep(duration_sec)
        end_id = self.get_frame_id()
        print(f"Camera advanced by approximately {end_id - start_id} frames.")

    def stop(self):
        self.running = False

        try:
            if self.proc is not None:
                self.proc.terminate()
        except Exception:
            pass

        try:
            self.thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            if self.video_writer is not None:
                self.video_writer.release()
                print(f"Saved video stream: {VIDEO_OUTPUT_FILE}")
        except Exception:
            pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def get_detection(camera, detector, parameters, dictionary, target_id, timeout_sec=2.0):
    start_time = time.time()
    last_frame_id = None

    while time.time() - start_time < timeout_sec:
        frame, frame_id = camera.wait_for_new_frame(
            after_frame_id=last_frame_id,
            timeout_sec=0.5
        )

        if frame is None:
            continue

        last_frame_id = frame_id

        result = detect_target(frame, detector, parameters, dictionary, target_id)
        if result is not None:
            return result

    return None


def get_both_tag_detections(camera, detector, parameters, dictionary, timeout_sec=2.0):
    start_time = time.time()
    last_frame_id = None

    while time.time() - start_time < timeout_sec:
        frame, frame_id = camera.wait_for_new_frame(
            after_frame_id=last_frame_id,
            timeout_sec=0.5
        )

        if frame is None:
            continue

        last_frame_id = frame_id
        results = detect_all_tags(frame, detector, parameters, dictionary)

        large = results.get(APPROACH_TAG_ID, None)
        small = results.get(FINAL_TAG_ID, None)

        if large is not None or small is not None:
            return large, small

    return None, None


def desired_small_tag_err_x_from_size(size_px):
    if USE_MEASURED_FINAL_DOCKING_TARGET:
        return FINAL_DOCK_TARGET_ERR_X_PX

    px_per_mm = size_px / FINAL_TAG_SIZE_MM
    return FINAL_TAG_X_OFFSET_SIGN * FINAL_TAG_X_OFFSET_MM * px_per_mm


def desired_small_tag_err_x_from_result(result):
    if result is None:
        return 0.0
    return desired_small_tag_err_x_from_size(result["size_px"])


def desired_small_tag_err_y():
    if USE_MEASURED_FINAL_DOCKING_TARGET:
        return FINAL_DOCK_TARGET_ERR_Y_PX
    return FINAL_DESIRED_ERR_Y

def print_detection(prefix, result, desired_x=0.0, desired_y=0.0):
    if result is None:
        print(f"{prefix} None")
        return

    raw_err_x = result["err_x"]
    raw_err_y = result["err_y"]

    control_err_x = raw_err_x - desired_x
    control_err_y = raw_err_y - desired_y

    print(
        f"{prefix} "
        f"id={result['id']} | "
        f"size={result['size_px']:.1f}px | "
        f"raw_err_x={raw_err_x:.1f}px | "
        f"desired_x={desired_x:.1f}px | "
        f"control_err_x={control_err_x:.1f}px | "
        f"raw_err_y={raw_err_y:.1f}px | "
        f"desired_y={desired_y:.1f}px | "
        f"control_err_y={control_err_y:.1f}px | "
        f"yaw={result['yaw_deg']:.1f}deg"
    )


def print_tag_status(prefix, large, small):
    print("\n" + "-" * 70)
    print(prefix)

    if large is None:
        print("Large tag ID 0: NOT VISIBLE")
    else:
        print_detection("Large tag ID 0:", large, desired_x=APPROACH_DESIRED_ERR_X)

    if small is None:
        print("Small tag ID 1: NOT VISIBLE")
    else:
        desired_x = desired_small_tag_err_x_from_result(small)
        print_detection("Small tag ID 1:", small, desired_x=desired_x, desired_y=FINAL_DESIRED_ERR_Y)

    print("-" * 70)


# ==========================================================
# DECISION LOGIC
# ==========================================================


def large_tag_x_is_aligned(large):
    if large is None:
        return False
    return abs(large["err_x"] - APPROACH_DESIRED_ERR_X) <= APPROACH_CENTER_TOLERANCE_PX


def small_tag_is_reasonable_for_switch(small):
    if small is None:
        return False

    if small["size_px"] < SMALL_TAG_SWITCH_SIZE_PX:
        return False

    if abs(small["err_x"]) > SMALL_TAG_SWITCH_MAX_ABS_ERR_X:
        print(
            f"Small tag size is large enough, but X is too far off-center for switch: "
            f"err_x={small['err_x']:.1f}px"
        )
        return False

    if abs(small["err_y"]) > SMALL_TAG_SWITCH_MAX_ABS_ERR_Y:
        print(
            f"Small tag size is large enough, but Y is too far off-center for switch: "
            f"err_y={small['err_y']:.1f}px"
        )
        return False

    return True


def should_switch_to_small_tag(large, small):
    # Simplified visibility-based policy for this version:
    # Use the large tag whenever it is visible. Use the small tag only when the
    # large tag is not visible. Do not switch based on tag size jargon.
    if large is not None:
        print("Large tag is visible, so do NOT switch to small-tag control based on size.")
        return False

    if small is not None:
        print("Large tag is not visible and small tag is visible: use small-tag control.")
        return True

    print("Neither tag is visible: cannot switch to small-tag control yet.")
    return False

def is_dock_ready_from_small_tag(small):
    if small is None:
        print("Not dock-ready: small tag is not visible.")
        return False

    desired_x = desired_small_tag_err_x_from_result(small)
    desired_y = desired_small_tag_err_y()
    control_err_x = small["err_x"] - desired_x
    control_err_y = small["err_y"] - desired_y

    size_ok = small["size_px"] >= SMALL_TAG_DOCK_READY_SIZE_PX
    x_ok = abs(control_err_x) <= SMALL_TAG_DOCK_MAX_ABS_CONTROL_ERR_X
    y_ok = (not DOCK_READY_USE_Y_GATE) or (abs(control_err_y) <= SMALL_TAG_DOCK_MAX_ABS_ERR_Y)

    print(
        f"Dock-ready check against measured small-tag docking target: "
        f"size={small['size_px']:.1f}px >= {SMALL_TAG_DOCK_READY_SIZE_PX:.1f}px "
        f"(measured target {FINAL_DOCK_TARGET_SIZE_PX:.1f}px), "
        f"err_x={small['err_x']:.1f}px target={desired_x:.1f}px "
        f"control_x={control_err_x:.1f}px/{SMALL_TAG_DOCK_MAX_ABS_CONTROL_ERR_X:.1f}, "
        f"err_y={small['err_y']:.1f}px target={desired_y:.1f}px "
        f"control_y={control_err_y:.1f}px/{SMALL_TAG_DOCK_MAX_ABS_ERR_Y:.1f} (Y gate={'ON' if DOCK_READY_USE_Y_GATE else 'OFF'})"
    )

    if size_ok and x_ok and y_ok:
        print("DOCK-READY: small tag is close enough for docking. X/size passed; Y gate is disabled unless configured on.")
        return True

    print("Not dock-ready yet.")
    return False

def small_tag_is_close_monitor_candidate(small):
    if small is None:
        return False

    desired_x = desired_small_tag_err_x_from_result(small)
    control_err_x = small["err_x"] - desired_x
    control_err_y = small["err_y"] - desired_small_tag_err_y()

    return (
        small["size_px"] >= SMALL_TAG_CLOSE_MONITOR_SIZE_PX
        and abs(control_err_x) <= SMALL_TAG_CLOSE_MONITOR_MAX_ABS_CONTROL_ERR_X
        and abs(control_err_y) <= SMALL_TAG_CLOSE_MONITOR_MAX_ABS_ERR_Y
    )


def large_tag_is_close_monitor_candidate(large):
    if large is None:
        return False

    control_err_x = large["err_x"] - APPROACH_DESIRED_ERR_X

    return (
        large["size_px"] >= LARGE_TAG_CLOSE_MONITOR_SIZE_PX
        and abs(control_err_x) <= LARGE_TAG_CLOSE_MONITOR_MAX_ABS_ERR_X
    )


def tag_state_is_close_dock_candidate(large, small):
    if small_tag_is_close_monitor_candidate(small):
        return True, "small tag close/aligned during monitoring"

    if large_tag_is_close_monitor_candidate(large):
        return True, "large tag close/aligned during monitoring"

    return False, None


def remember_tag_state(large, small, reason=""):
    global last_known_large_tag
    global last_known_small_tag
    global last_known_tag_time
    global last_known_close_dock_candidate
    global last_known_close_reason

    if large is None and small is None:
        return

    last_known_large_tag = large.copy() if large is not None else None
    last_known_small_tag = small.copy() if small is not None else None
    last_known_tag_time = time.time()

    close_candidate, close_reason = tag_state_is_close_dock_candidate(large, small)
    last_known_close_dock_candidate = close_candidate
    last_known_close_reason = close_reason if close_candidate else reason

    if close_candidate:
        print(
            f"Remembered close docking candidate from tags: {last_known_close_reason}. "
            f"This can be used if the tags become occluded near docking."
        )


def recent_last_known_close_dock_candidate():
    if not ALLOW_FINAL_DXL_CLOSE_FROM_LAST_CLOSE_TAG_WHEN_OCCLUDED:
        return False

    if not last_known_close_dock_candidate or last_known_tag_time is None:
        return False

    age = time.time() - last_known_tag_time
    if age <= LAST_CLOSE_TAG_VALID_SEC:
        print(
            f"Recent close docking candidate exists from {age:.1f} sec ago: "
            f"{last_known_close_reason}"
        )
        return True

    print(
        f"Last close docking candidate is too old: {age:.1f} sec > "
        f"{LAST_CLOSE_TAG_VALID_SEC:.1f} sec"
    )
    return False



def recent_large_tag_was_pre_extension_aligned_for_visibility_reveal():
    if last_known_large_tag is None or last_known_tag_time is None:
        print("Visibility-reveal check: no recent large-tag state remembered.")
        return False

    age = time.time() - last_known_tag_time
    control_err_x = last_known_large_tag["err_x"] - APPROACH_DESIRED_ERR_X
    size_px = last_known_large_tag["size_px"]

    ok = (
        age <= VISIBILITY_REVEAL_RECENT_LARGE_VALID_SEC
        and size_px >= VISIBILITY_REVEAL_MIN_RECENT_LARGE_SIZE_PX
        and abs(control_err_x) <= VISIBILITY_REVEAL_MAX_RECENT_LARGE_ABS_X_PX
    )

    print(
        "Visibility-reveal recent-large check: "
        f"age={age:.1f}s/{VISIBILITY_REVEAL_RECENT_LARGE_VALID_SEC:.1f}s, "
        f"large_size={size_px:.1f}px/{VISIBILITY_REVEAL_MIN_RECENT_LARGE_SIZE_PX:.1f}px, "
        f"large_control_x={control_err_x:.1f}px/±{VISIBILITY_REVEAL_MAX_RECENT_LARGE_ABS_X_PX:.1f}px, "
        f"ok={ok}"
    )
    return ok


def small_tag_is_visibility_reveal_dock_candidate(small, acquisition_status):
    if not VISIBILITY_REVEAL_DOCK_CANDIDATE_ENABLED:
        return False

    if small is None:
        return False

    if acquisition_status != "small_found_during_blind_right_search":
        return False

    if last_small_search_right_steps_taken > VISIBILITY_REVEAL_MAX_RIGHT_SEARCH_STEPS:
        print(
            "Visibility-reveal check: small tag was found, but only after too many right-search steps: "
            f"{last_small_search_right_steps_taken} > {VISIBILITY_REVEAL_MAX_RIGHT_SEARCH_STEPS}"
        )
        return False

    desired_y = desired_small_tag_err_y()
    control_y = small["err_y"] - desired_y

    size_ok = small["size_px"] >= VISIBILITY_REVEAL_MIN_SMALL_SIZE_PX
    y_ok = abs(control_y) <= VISIBILITY_REVEAL_MAX_ABS_SMALL_CONTROL_Y_PX
    recent_large_ok = recent_large_tag_was_pre_extension_aligned_for_visibility_reveal()

    ok = size_ok and y_ok and recent_large_ok

    print(
        "Visibility-reveal small-tag check: "
        f"right_steps={last_small_search_right_steps_taken}/{VISIBILITY_REVEAL_MAX_RIGHT_SEARCH_STEPS}, "
        f"small_size={small['size_px']:.1f}px/{VISIBILITY_REVEAL_MIN_SMALL_SIZE_PX:.1f}px, "
        f"small_control_y={control_y:.1f}px/±{VISIBILITY_REVEAL_MAX_ABS_SMALL_CONTROL_Y_PX:.1f}px, "
        "small X intentionally ignored in this special case, "
        f"ok={ok}"
    )
    return ok


def remember_visibility_reveal_dock_candidate(small, large=None):
    global last_known_large_tag
    global last_known_small_tag
    global last_known_tag_time
    global last_known_close_dock_candidate
    global last_known_close_reason

    last_known_large_tag = large.copy() if large is not None else last_known_large_tag
    last_known_small_tag = small.copy() if small is not None else None
    last_known_tag_time = time.time()
    last_known_close_dock_candidate = True
    last_known_close_reason = (
        "visibility-reveal docking candidate: recent large tag was aligned, "
        "then a short right-search revealed a dock-size small tag"
    )
    print(
        "Remembered visibility-reveal docking candidate. "
        "Skipping strict small-tag correction because the search pose itself can make "
        "the small tag appear X-offset even when the robot is physically aligned."
    )

def monitor_tags_after_inching_stage(camera, detector, parameters, dictionary, stage_label):
    if not MONITOR_TAGS_DURING_INCHING:
        return None, None, False

    camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

    large, small = get_both_tag_detections(
        camera,
        detector,
        parameters,
        dictionary,
        timeout_sec=2.0
    )

    print_tag_status(f"Inching monitor after {stage_label}:", large, small)

    if large is not None or small is not None:
        remember_tag_state(large, small, reason=f"after {stage_label}")
        close_candidate, close_reason = tag_state_is_close_dock_candidate(large, small)

        if INTERRUPT_INCHING_IF_DOCK_CANDIDATE and close_candidate:
            print(
                f"Inching interrupt: {close_reason}. "
                "Stop remaining inching steps and go to final alignment/docking."
            )
            return large, small, True

        return large, small, False

    print("No tags visible during inching monitor.")

    if INTERRUPT_INCHING_IF_DOCK_CANDIDATE and recent_last_known_close_dock_candidate():
        print(
            "Tags are now occluded/missing, but the last known tag state was close enough. "
            "Stop remaining inching steps and allow final docking attempt."
        )
        return None, None, True

    return None, None, False


def choose_extension_repeat_count(large, small):
    # Keep inching predictable: do the standard number of S-D-A extension groups.
    # Tag size is now used for docking readiness/monitoring, not for switching
    # between large and small tag controllers.
    return DEFAULT_EXTEND_REPEAT_COUNT

def should_inch_more(large, small):
    if small is not None and is_dock_ready_from_small_tag(small):
        print("Should not inch more: small tag is at the measured docking pose.")
        return False

    if large is not None:
        print("Large tag visible: continue controlled inching after alignment.")
        return True

    if small is not None:
        print("Large tag missing but small tag visible: continue cautious inching after small-tag alignment.")
        return True

    print("No tags visible.")
    if recent_last_known_close_dock_candidate():
        print("No inching: last known tag state was close enough; try docking instead of blind inching.")
        return False

    return ALLOW_BLIND_INCHING_WITH_NO_TAGS

def maybe_final_dxl_docking_close(dxl, final_small_tag_result):
    global docking_successful

    print("\nChecking whether final DXL docking close should run...")

    if not ENABLE_FINAL_DXL_DOCKING_CLOSE:
        print("ENABLE_FINAL_DXL_DOCKING_CLOSE = False")
        print("Skipping final DXL docking close.")
        return False

    if final_small_tag_result is not None:
        if not is_dock_ready_from_small_tag(final_small_tag_result):
            print("Final DXL docking close blocked because dock-ready check failed.")
            return False
    else:
        print("No final visible small-tag confirmation available.")
        if not recent_last_known_close_dock_candidate():
            print("Final DXL docking close blocked: no recent close/aligned tag state to trust.")
            return False
        print("Allowing final DXL docking close from recent close/aligned tag state because tags may be occluded.")

    print("\n" + "!" * 70)
    print("FINAL DXL DOCKING CLOSE ENABLED")
    print(f"Commanding front claw to docking position: {FRONT_CLAW_DOCKING_POS}")
    print(f"Hard limits: [{DXL_HARD_MIN_POS}, {DXL_HARD_MAX_POS}]")
    print("!" * 70)

    dxl.docking_close_front_claw_blocking()

    # Mark success immediately after the final close succeeds. If the user
    # interrupts during the hold loop, we should NOT send Q because docking
    # was already achieved.
    docking_successful = True

    print("Final DXL docking close complete.")
    dxl.hold_after_successful_docking_until_interrupt()
    return True


# ==========================================================
# X ALIGNMENT
# ==========================================================


def apply_move(ser, move_name):
    if move_name == "LEFT":
        print("Applying LEFT bend: a + s")
        send_sequence(ser, LEFT_BEND_SEQUENCE, track_top_a_balance=True)

    elif move_name == "RIGHT":
        print("Applying RIGHT bend: a + d")
        send_sequence(ser, RIGHT_BEND_SEQUENCE, track_top_a_balance=True)

    else:
        raise ValueError(f"Unknown move: {move_name}")


def undo_move(ser, move_name):
    if move_name == "LEFT":
        print("Undoing LEFT bend: A + S")
        send_sequence(ser, UNDO_LEFT_SEQUENCE, track_top_a_balance=True)

    elif move_name == "RIGHT":
        print("Undoing RIGHT bend: A + D")
        send_sequence(ser, UNDO_RIGHT_SEQUENCE, track_top_a_balance=True)

    else:
        raise ValueError(f"Unknown move: {move_name}")


def preferred_move_from_error(control_err_x):
    if control_err_x < 0:
        move = "LEFT"
    else:
        move = "RIGHT"

    if INVERT_LEFT_RIGHT:
        move = "RIGHT" if move == "LEFT" else "LEFT"

    return move


def align_x_to_large_tag(ser, camera, detector, parameters, dictionary):
    latest_detection = None

    print("\nStarting LARGE TAG X alignment phase...")
    print(f"Target tag ID: {APPROACH_TAG_ID}")
    print(f"X tolerance: +/- {APPROACH_CENTER_TOLERANCE_PX} px")

    for step in range(1, APPROACH_MAX_ALIGNMENT_STEPS + 1):
        print("\n" + "=" * 60)
        print(f"Large-tag X alignment step {step}/{APPROACH_MAX_ALIGNMENT_STEPS}")

        before = get_detection(
            camera,
            detector,
            parameters,
            dictionary,
            target_id=APPROACH_TAG_ID,
            timeout_sec=2.0
        )

        if before is None:
            print("Large tag not visible. Waiting...")
            time.sleep(LOST_TAG_WAIT_SEC)
            camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)
            continue

        latest_detection = before
        print_detection("Before large-tag X:", before, desired_x=APPROACH_DESIRED_ERR_X)

        before_err = before["err_x"] - APPROACH_DESIRED_ERR_X

        if abs(before_err) <= APPROACH_CENTER_TOLERANCE_PX:
            print("Large-tag X aligned.")
            return before

        best_abs = abs(before_err)
        move = preferred_move_from_error(before_err)
        accepted_any = False

        for pulse in range(1, APPROACH_MAX_PULSES_SAME_DIRECTION + 1):
            print(f"\nLarge-tag X pulse {pulse}/{APPROACH_MAX_PULSES_SAME_DIRECTION}: {move}")

            apply_move(ser, move)
            time.sleep(SETTLE_AFTER_PRIMITIVE_SEC)
            camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

            after = get_detection(
                camera,
                detector,
                parameters,
                dictionary,
                target_id=APPROACH_TAG_ID,
                timeout_sec=2.0
            )

            if after is None:
                print("Large tag not detected after X move. Undoing pulse.")
                undo_move(ser, move)
                time.sleep(SETTLE_AFTER_PRIMITIVE_SEC)
                camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)
                return latest_detection

            print_detection("After large-tag X:", after, desired_x=APPROACH_DESIRED_ERR_X)

            after_err = after["err_x"] - APPROACH_DESIRED_ERR_X
            after_abs = abs(after_err)
            improvement = best_abs - after_abs

            print(f"Large-tag X improvement = {improvement:.1f} px")

            if after_abs <= APPROACH_CENTER_TOLERANCE_PX:
                print("Large-tag X alignment complete.")
                return after

            if improvement >= MIN_IMPROVEMENT_PX:
                best_abs = after_abs
                latest_detection = after
                accepted_any = True
                continue

            if after_abs > best_abs + WORSE_MARGIN_PX:
                print("Large-tag X got much worse. Undoing pulse.")
                undo_move(ser, move)
                time.sleep(SETTLE_AFTER_PRIMITIVE_SEC)
                camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)
                return latest_detection

            latest_detection = after

        if not accepted_any:
            print("No strong large-tag X improvement. Stopping large alignment.")
            return latest_detection

    return latest_detection


def large_tag_alignment_if_visible(ser, camera, detector, parameters, dictionary):
    large, small = get_both_tag_detections(
        camera,
        detector,
        parameters,
        dictionary,
        timeout_sec=2.0
    )

    print_tag_status("Large-tag helper check:", large, small)

    if large is None:
        print("Large tag not visible. Cannot use large-tag alignment.")
        return large, small

    if large_tag_x_is_aligned(large):
        print("Large tag is already X aligned.")
        return large, small

    print("Large tag is visible and offset. Aligning with large tag BEFORE small-tag search.")
    align_x_to_large_tag(ser, camera, detector, parameters, dictionary)
    cleanup_top_a_pulls_after_large_alignment(ser)
    camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

    large, small = get_both_tag_detections(
        camera,
        detector,
        parameters,
        dictionary,
        timeout_sec=2.0
    )

    print_tag_status("After large-tag alignment helper:", large, small)
    return large, small


def align_x_to_small_tag_dynamic_offset(ser, camera, detector, parameters, dictionary):
    latest_detection = None

    print("\nStarting FINAL small-tag X alignment to measured docking X target...")
    print(f"X tolerance: +/- {FINAL_X_TOLERANCE_PX} px")

    for step in range(1, FINAL_MAX_X_STEPS + 1):
        print("\n" + "=" * 60)
        print(f"FINAL small-tag X alignment step {step}/{FINAL_MAX_X_STEPS}")

        before = get_detection(
            camera,
            detector,
            parameters,
            dictionary,
            target_id=FINAL_TAG_ID,
            timeout_sec=FINAL_TAG_SEARCH_TIMEOUT_SEC
        )

        if before is None:
            print("Small tag not visible during final X alignment.")
            time.sleep(LOST_TAG_WAIT_SEC)
            camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)
            continue

        latest_detection = before
        desired_x = desired_small_tag_err_x_from_result(before)
        before_err = before["err_x"] - desired_x

        print_detection("Before FINAL X:", before, desired_x=desired_x, desired_y=FINAL_DESIRED_ERR_Y)

        if abs(before_err) <= FINAL_X_TOLERANCE_PX:
            print("FINAL small-tag X aligned.")
            return before

        move = preferred_move_from_error(before_err)
        before_abs = abs(before_err)

        apply_move(ser, move)
        time.sleep(FINAL_SETTLE_SEC)
        camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)

        after = get_detection(
            camera,
            detector,
            parameters,
            dictionary,
            target_id=FINAL_TAG_ID,
            timeout_sec=FINAL_TAG_SEARCH_TIMEOUT_SEC
        )

        if after is None:
            print("Small tag lost after FINAL X move. Undoing pulse.")
            undo_move(ser, move)
            time.sleep(FINAL_SETTLE_SEC)
            camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)
            return latest_detection

        desired_x_after = desired_small_tag_err_x_from_result(after)
        after_err = after["err_x"] - desired_x_after
        after_abs = abs(after_err)
        improvement = before_abs - after_abs

        print_detection("After FINAL X:", after, desired_x=desired_x_after, desired_y=FINAL_DESIRED_ERR_Y)
        print(f"FINAL X improvement = {improvement:.1f} px")

        latest_detection = after

        if after_abs <= FINAL_X_TOLERANCE_PX:
            print("FINAL small-tag X alignment complete.")
            return after

        if improvement < -WORSE_MARGIN_PX:
            print("FINAL X got much worse. Undoing pulse.")
            undo_move(ser, move)
            time.sleep(FINAL_SETTLE_SEC)
            camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)
            return latest_detection

    print("Reached maximum FINAL X alignment steps.")
    return latest_detection


# ==========================================================
# FINAL Y ALIGNMENT
# ==========================================================


def apply_y_move(ser, move_name):
    if move_name == "UP":
        print("Applying FINAL Y UP command: a")
        send_sequence(ser, FINAL_Y_UP_SEQUENCE, track_top_a_balance=True)

    elif move_name == "DOWN":
        print("Applying FINAL Y DOWN command: A")
        send_sequence(ser, FINAL_Y_DOWN_SEQUENCE, track_top_a_balance=True)

    else:
        raise ValueError(f"Unknown Y move: {move_name}")


def preferred_y_move_from_error(control_err_y):
    if control_err_y > 0:
        move = "UP"
    else:
        move = "DOWN"

    if INVERT_FINAL_Y:
        move = "DOWN" if move == "UP" else "UP"

    return move


def align_y_to_final_tag(ser, camera, detector, parameters, dictionary):
    latest_detection = None

    print("\nStarting FINAL Y alignment using small tag ID 1...")
    print(f"Y tolerance: +/- {FINAL_Y_TOLERANCE_PX} px")
    print(f"INVERT_FINAL_Y = {INVERT_FINAL_Y}")

    for step in range(1, FINAL_MAX_Y_STEPS + 1):
        print("\n" + "=" * 60)
        print(f"FINAL Y alignment step {step}/{FINAL_MAX_Y_STEPS}")

        before = get_detection(
            camera,
            detector,
            parameters,
            dictionary,
            target_id=FINAL_TAG_ID,
            timeout_sec=FINAL_TAG_SEARCH_TIMEOUT_SEC
        )

        if before is None:
            print("Small tag ID 1 not visible during Y alignment.")
            time.sleep(LOST_TAG_WAIT_SEC)
            camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)
            continue

        latest_detection = before
        desired_x = desired_small_tag_err_x_from_result(before)

        print_detection(
            "Before FINAL Y:",
            before,
            desired_x=desired_x,
            desired_y=FINAL_DESIRED_ERR_Y
        )

        control_err_y = before["err_y"] - FINAL_DESIRED_ERR_Y

        if abs(control_err_y) <= FINAL_Y_TOLERANCE_PX:
            print("FINAL Y aligned.")
            return before

        move = preferred_y_move_from_error(control_err_y)
        before_abs = abs(control_err_y)

        apply_y_move(ser, move)
        time.sleep(FINAL_SETTLE_SEC)
        camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)

        after = get_detection(
            camera,
            detector,
            parameters,
            dictionary,
            target_id=FINAL_TAG_ID,
            timeout_sec=FINAL_TAG_SEARCH_TIMEOUT_SEC
        )

        if after is None:
            print("Small tag lost after Y move. Continuing cautiously.")
            continue

        desired_x_after = desired_small_tag_err_x_from_result(after)

        print_detection(
            "After FINAL Y:",
            after,
            desired_x=desired_x_after,
            desired_y=FINAL_DESIRED_ERR_Y
        )

        after_err_y = after["err_y"] - FINAL_DESIRED_ERR_Y
        after_abs = abs(after_err_y)
        improvement = before_abs - after_abs

        print(f"FINAL Y improvement = {improvement:.1f} px")

        latest_detection = after

        if after_abs <= FINAL_Y_TOLERANCE_PX:
            print("FINAL Y alignment completed.")
            return after

        if improvement < -WORSE_MARGIN_PX:
            print("FINAL Y got much worse. Reversing that Y pulse.")
            reverse_move = "DOWN" if move == "UP" else "UP"
            apply_y_move(ser, reverse_move)
            time.sleep(FINAL_SETTLE_SEC)
            camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)
            return latest_detection

    print("Reached maximum FINAL Y alignment steps.")
    return latest_detection


def search_for_small_tag_bias_right_only_when_large_missing(ser, camera, detector, parameters, dictionary):
    global last_small_search_right_steps_taken
    last_small_search_right_steps_taken = 0

    if not SEARCH_FOR_SMALL_TAG_IF_LOST:
        return None, None, None

    print("\nSmall-tag acquisition requested.")
    print("Rule: use large tag first if visible. Only blind-search right if large tag is missing.")

    large, small = large_tag_alignment_if_visible(
        ser,
        camera,
        detector,
        parameters,
        dictionary
    )

    if small is not None:
        print("Small tag found after using/checking large tag.")
        return small, large, "small_found_after_large_help"

    if large is not None:
        print("Large tag is visible, but small tag is still not visible.")
        print("Not doing blind right-search yet because large tag is available.")
        print("Caller should use large tag state to decide alignment/inching/stop.")
        return None, large, "large_visible_small_missing"

    print("\nLarge tag is NOT visible. Starting blind right-biased search.")
    print(f"Max right steps: {SMALL_TAG_SEARCH_MAX_RIGHT_STEPS}")
    print("During search, checking BOTH small and large tags after every right step.")

    accepted_steps = 0
    any_large_seen = None

    for step in range(1, SMALL_TAG_SEARCH_MAX_RIGHT_STEPS + 1):
        print(f"\nSmall-tag blind right search step {step}/{SMALL_TAG_SEARCH_MAX_RIGHT_STEPS}")
        apply_move(ser, "RIGHT")
        accepted_steps += 1
        last_small_search_right_steps_taken = accepted_steps

        time.sleep(SMALL_TAG_SEARCH_SETTLE_SEC)
        camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)

        large, small = get_both_tag_detections(
            camera,
            detector,
            parameters,
            dictionary,
            timeout_sec=FINAL_TAG_SEARCH_TIMEOUT_SEC
        )

        print_tag_status("During blind right search:", large, small)

        if small is not None:
            desired_x = desired_small_tag_err_x_from_result(small)
            print_detection("Small tag found during blind right search:", small, desired_x=desired_x)
            return small, large, "small_found_during_blind_right_search"

        if large is not None:
            any_large_seen = large
            print("Large tag became visible during blind right search.")
            print("Stopping blind search and returning control to large-tag logic.")
            return None, large, "large_found_during_blind_right_search"

    print("Small tag not found after blind right search.")

    if any_large_seen is None:
        print("No large tag found either. Undoing blind right search steps.")
        for i in range(accepted_steps):
            undo_move(ser, "RIGHT")
            time.sleep(CHAR_DELAY_SEC)

        return None, None, "no_tags_after_blind_search"

    return None, any_large_seen, "large_seen_after_blind_search"


def final_align_small_tag_y_then_x(ser, camera, detector, parameters, dictionary):
    print("\n" + "#" * 70)
    print("STARTING FINAL CLOSE-RANGE ALIGNMENT USING SMALL TAG ID 1")
    print("#" * 70)

    set_small_alignment_mode(ser)
    camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)

    # Important corrected logic:
    # Try large tag first if visible before blind-searching for the small tag.
    small, large, acquisition_status = search_for_small_tag_bias_right_only_when_large_missing(
        ser,
        camera,
        detector,
        parameters,
        dictionary
    )

    print(f"Small-tag acquisition status: {acquisition_status}")

    if small is None:
        print("Could not acquire small tag ID 1 for final alignment.")
        return None

    desired_x = desired_small_tag_err_x_from_result(small)

    print_detection(
        "Initial small tag detection:",
        small,
        desired_x=desired_x,
        desired_y=FINAL_DESIRED_ERR_Y
    )

    result_y = align_y_to_final_tag(
        ser,
        camera,
        detector,
        parameters,
        dictionary
    )

    if result_y is None:
        print("FINAL Y alignment failed: small tag not visible.")
        return None

    balance_before_final_x = top_a_pull_balance

    print("\nSaving top_a_pull_balance before FINAL X.")
    print(f"balance_before_final_x = {balance_before_final_x}")

    result_x = align_x_to_small_tag_dynamic_offset(
        ser,
        camera,
        detector,
        parameters,
        dictionary
    )

    cleanup_extra_a_from_final_x(ser, balance_before_final_x)

    time.sleep(FINAL_SETTLE_SEC)
    camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)

    final_result = get_detection(
        camera,
        detector,
        parameters,
        dictionary,
        target_id=FINAL_TAG_ID,
        timeout_sec=FINAL_TAG_SEARCH_TIMEOUT_SEC
    )

    if final_result is None:
        print("Small tag lost after FINAL alignment/cleanup.")
        print("Not dock-ready because final visible confirmation failed.")
        return None

    desired_x_final = desired_small_tag_err_x_from_result(final_result)

    print_detection(
        "Final visible confirmation:",
        final_result,
        desired_x=desired_x_final,
        desired_y=FINAL_DESIRED_ERR_Y
    )

    print("\nFINAL SMALL-TAG ALIGNMENT DONE.")

    is_dock_ready_from_small_tag(final_result)

    return final_result


# ==========================================================
# INCHING WITH DYNAMIXEL
# ==========================================================



def set_inching_interrupt_reason(reason):
    global last_inching_interrupt_reason
    last_inching_interrupt_reason = reason
    print(f"Inching interrupt reason set to: {reason}")


def clear_inching_interrupt_reason():
    global last_inching_interrupt_reason
    last_inching_interrupt_reason = None


def get_pre_extension_strictness_multiplier():
    m = float(PRE_EXTENSION_ALIGNMENT_STRICTNESS_MULTIPLIER)
    if m <= 0.0:
        print(
            f"WARNING: PRE_EXTENSION_ALIGNMENT_STRICTNESS_MULTIPLIER={m} is invalid. "
            "Using 1.0 instead."
        )
        return 1.0
    return m


def effective_pre_extension_large_x_tolerance():
    return PRE_EXTENSION_LARGE_X_TOLERANCE_PX / get_pre_extension_strictness_multiplier()


def effective_pre_extension_small_x_tolerance():
    return PRE_EXTENSION_SMALL_X_TOLERANCE_PX / get_pre_extension_strictness_multiplier()


def effective_pre_extension_small_y_tolerance():
    return PRE_EXTENSION_SMALL_Y_TOLERANCE_PX / get_pre_extension_strictness_multiplier()


def print_pre_extension_effective_tolerances():
    print(
        "Pre-extension effective tolerances: "
        f"multiplier={get_pre_extension_strictness_multiplier():.2f}, "
        f"large_X=±{effective_pre_extension_large_x_tolerance():.1f}px, "
        f"small_X=±{effective_pre_extension_small_x_tolerance():.1f}px, "
        f"small_Y=±{effective_pre_extension_small_y_tolerance():.1f}px "
        f"(Y gate={'ON' if PRE_EXTENSION_USE_SMALL_Y_GATE else 'OFF'})"
    )

def large_tag_x_is_pre_extension_aligned(large):
    if large is None:
        return False
    control_err_x = large["err_x"] - APPROACH_DESIRED_ERR_X
    ok = abs(control_err_x) <= effective_pre_extension_large_x_tolerance()
    print(
        f"Pre-extension large-tag X check: "
        f"control_err_x={control_err_x:.1f}px, "
        f"tolerance=±{effective_pre_extension_large_x_tolerance():.1f}px, ok={ok}"
    )
    return ok


def small_tag_is_pre_extension_aligned(small):
    if small is None:
        return False

    desired_x = desired_small_tag_err_x_from_result(small)
    desired_y = desired_small_tag_err_y()
    control_err_x = small["err_x"] - desired_x
    control_err_y = small["err_y"] - desired_y

    x_ok = abs(control_err_x) <= effective_pre_extension_small_x_tolerance()
    y_ok = (not PRE_EXTENSION_USE_SMALL_Y_GATE) or (abs(control_err_y) <= effective_pre_extension_small_y_tolerance())

    print(
        f"Pre-extension small-tag alignment check: "
        f"size={small['size_px']:.1f}px, "
        f"err_x={small['err_x']:.1f}px target={desired_x:.1f}px "
        f"control_x={control_err_x:.1f}px/±{effective_pre_extension_small_x_tolerance():.1f}px, "
        f"err_y={small['err_y']:.1f}px target={desired_y:.1f}px "
        f"control_y={control_err_y:.1f}px/±{effective_pre_extension_small_y_tolerance():.1f}px "
        f"(Y gate={'ON' if PRE_EXTENSION_USE_SMALL_Y_GATE else 'OFF'}), "
        f"ok={x_ok and y_ok}"
    )

    return x_ok and y_ok



def small_tag_x_is_pre_extension_aligned_for_override(small):
    """
    v20 close-range target-selection helper.

    This checks only the small tag X target, not Y. Y is not a hard close-range
    docking gate in this version because the camera Y can change when the body
    bends, while the mechanical docking mainly needs X/entry alignment.
    """
    if not PRE_EXTENSION_SMALL_VISIBLE_X_OVERRIDE_ENABLED:
        return False

    if small is None:
        return False

    if small["size_px"] < PRE_EXTENSION_SMALL_VISIBLE_OVERRIDE_MIN_SIZE_PX:
        return False

    desired_x = desired_small_tag_err_x_from_result(small)
    control_err_x = small["err_x"] - desired_x
    x_tol = effective_pre_extension_small_x_tolerance()
    ok = abs(control_err_x) <= x_tol

    print(
        "Pre-extension small-visible X override check: "
        f"size={small['size_px']:.1f}px/{PRE_EXTENSION_SMALL_VISIBLE_OVERRIDE_MIN_SIZE_PX:.1f}px, "
        f"control_x={control_err_x:.1f}px/±{x_tol:.1f}px, "
        f"Y ignored, ok={ok}"
    )

    return ok

def align_large_tag_for_pre_extension(ser, camera, detector, parameters, dictionary):
    """
    Run large-tag alignment using stricter pre-extension X tolerance.
    The normal large-tag tolerance remains unchanged elsewhere.
    """
    global APPROACH_CENTER_TOLERANCE_PX

    old_tol = APPROACH_CENTER_TOLERANCE_PX
    APPROACH_CENTER_TOLERANCE_PX = effective_pre_extension_large_x_tolerance()

    print(
        "\nRunning STRICT pre-extension large-tag X alignment: "
        f"temporary tolerance ±{effective_pre_extension_large_x_tolerance():.1f}px "
        f"instead of normal ±{old_tol:.1f}px."
    )

    try:
        return align_x_to_large_tag(ser, camera, detector, parameters, dictionary)
    finally:
        APPROACH_CENTER_TOLERANCE_PX = old_tol
        print(f"Restored normal large-tag X tolerance to ±{APPROACH_CENTER_TOLERANCE_PX:.1f}px.")


def align_small_tag_for_pre_extension(ser, camera, detector, parameters, dictionary):
    """
    Run measured-pose small-tag alignment using stricter pre-extension X/Y tolerances.
    The normal final alignment tolerances remain unchanged elsewhere.
    """
    global FINAL_X_TOLERANCE_PX, FINAL_Y_TOLERANCE_PX

    old_x_tol = FINAL_X_TOLERANCE_PX
    old_y_tol = FINAL_Y_TOLERANCE_PX

    FINAL_X_TOLERANCE_PX = effective_pre_extension_small_x_tolerance()
    FINAL_Y_TOLERANCE_PX = effective_pre_extension_small_y_tolerance()

    print(
        "\nRunning STRICT pre-extension small-tag measured-pose alignment: "
        f"temporary X tolerance ±{effective_pre_extension_small_x_tolerance():.1f}px "
        f"instead of normal ±{old_x_tol:.1f}px; "
        f"temporary Y tolerance ±{effective_pre_extension_small_y_tolerance():.1f}px "
        f"instead of normal ±{old_y_tol:.1f}px."
    )

    try:
        return final_align_small_tag_y_then_x(ser, camera, detector, parameters, dictionary)
    finally:
        FINAL_X_TOLERANCE_PX = old_x_tol
        FINAL_Y_TOLERANCE_PX = old_y_tol
        print(
            f"Restored normal final small-tag tolerances: "
            f"X ±{FINAL_X_TOLERANCE_PX:.1f}px, Y ±{FINAL_Y_TOLERANCE_PX:.1f}px."
        )


def in_switch_zone_for_pre_extension_alignment(large, small):
    """
    Returns True when the robot is close enough that extension should not happen
    until the visual alignment has been refreshed.

    Uses the same two switch-distance variables:
      - SMALL_TAG_SWITCH_SIZE_PX for tag ID 1
      - LARGE_TAG_SWITCH_SIZE_PX for tag ID 0
    """
    if small is not None and small["size_px"] >= SMALL_TAG_SWITCH_SIZE_PX:
        print(
            f"Pre-extension switch zone: small tag size "
            f"{small['size_px']:.1f}px >= {SMALL_TAG_SWITCH_SIZE_PX:.1f}px"
        )
        return True

    if large is not None and large["size_px"] >= LARGE_TAG_SWITCH_SIZE_PX:
        print(
            f"Pre-extension switch zone: large tag size "
            f"{large['size_px']:.1f}px >= {LARGE_TAG_SWITCH_SIZE_PX:.1f}px"
        )
        return True

    return False


def get_current_pre_extension_target(
    camera,
    detector,
    parameters,
    dictionary,
    prefer_large=True,
    timeout_sec=2.0,
    force_small=False,
):
    """
    Get the current tag used for pre-extension alignment.

    Normal policy:
        - If prefer_large=True and the large tag is visible, use the large tag.
        - Otherwise use the small tag if visible.

    Final-stage lock-on policy:
        - If force_small=True, use ONLY the small tag when it is visible.
        - This prevents the controller from finding the small tag during right-search
          and then immediately switching back to large-tag alignment when the large
          tag reappears.
    """
    large, small = get_both_tag_detections(
        camera,
        detector,
        parameters,
        dictionary,
        timeout_sec=timeout_sec
    )

    if force_small:
        if small is not None:
            return "small", small, desired_small_tag_err_x_from_result(small), effective_pre_extension_small_x_tolerance(), large, small
        print("Pre-extension target is locked to SMALL tag, but small tag is not visible right now.")
        return None, None, 0.0, 0.0, large, small

    # v20: At close range, do not chase the large tag if the small tag is already
    # acceptable in X. This exact failure mode happened when small control_x was
    # inside tolerance but large was slightly outside, so the controller kept
    # correcting large and lost the useful small-tag pose.
    if small_tag_x_is_pre_extension_aligned_for_override(small):
        print("Pre-extension target override: SMALL tag X is already acceptable; using SMALL instead of LARGE.")
        return "small", small, desired_small_tag_err_x_from_result(small), effective_pre_extension_small_x_tolerance(), large, small

    if prefer_large and large is not None:
        return "large", large, APPROACH_DESIRED_ERR_X, effective_pre_extension_large_x_tolerance(), large, small

    if small is not None:
        return "small", small, desired_small_tag_err_x_from_result(small), effective_pre_extension_small_x_tolerance(), large, small

    if large is not None:
        return "large", large, APPROACH_DESIRED_ERR_X, effective_pre_extension_large_x_tolerance(), large, small

    return None, None, 0.0, 0.0, large, small


def pre_extension_x_error(result, desired_x):
    if result is None:
        return None
    return result["err_x"] - desired_x


def pre_extension_send_alignment_pulse(ser, move):
    """
    Important hardware behavior:
        x  -> small displacement alignment mode
        a+d or a+s -> actual small alignment correction

    This makes the pre-extension alignment visible in the log and prevents the
    robot from accidentally applying the correction in inching displacement mode.
    """
    if PRE_EXTENSION_SEND_X_BEFORE_EVERY_CORRECTION:
        print("Pre-extension correction: forcing SMALL alignment mode x before pulse.")
        send_char(ser, ALIGNMENT_SMALL_STEP_COMMAND)
        time.sleep(0.2)

    apply_move(ser, move)


def strict_pre_extension_x_align_loop(ser, camera, detector, parameters, dictionary, prefer_large=True, force_small=False):
    """
    Actively align X before extension. This replaces the earlier weak behavior
    where the code could attempt alignment once and then still send S-D-A even
    though the tag remained badly offset.

    Returns:
        large, small, aligned_ok
    """
    print("\n" + "=" * 70)
    print("STRICT PRE-EXTENSION X ALIGNMENT LOOP")
    print("This loop sends x before each a+d/a+s correction and blocks S-D-A if still offset.")
    print(f"force_small target lock = {force_small}")
    print("=" * 70)

    last_large = None
    last_small = None

    for pulse in range(1, PRE_EXTENSION_MAX_X_ALIGNMENT_PULSES + 1):
        tag_kind, before, desired_x, x_tol, large, small = get_current_pre_extension_target(
            camera,
            detector,
            parameters,
            dictionary,
            prefer_large=prefer_large,
            timeout_sec=2.0,
            force_small=force_small
        )
        last_large, last_small = large, small

        print_tag_status(f"Pre-extension X loop detection {pulse}/{PRE_EXTENSION_MAX_X_ALIGNMENT_PULSES}:", large, small)

        if tag_kind is None or before is None:
            print("Pre-extension X alignment: no tags visible.")
            return large, small, False

        if tag_kind == "large":
            print("Pre-extension X target: LARGE tag ID 0, desired err_x = 0 px.")
            print_detection("Before pre-extension LARGE X:", before, desired_x=desired_x)
        else:
            print("Pre-extension X target: SMALL tag ID 1, desired measured docking err_x.")
            print_detection("Before pre-extension SMALL X:", before, desired_x=desired_x, desired_y=desired_small_tag_err_y())

        before_err = pre_extension_x_error(before, desired_x)
        before_abs = abs(before_err)

        print(
            f"Pre-extension X error = {before_err:.1f}px, "
            f"abs={before_abs:.1f}px, tolerance=±{x_tol:.1f}px"
        )

        if before_abs <= x_tol:
            print("Pre-extension X is aligned tightly enough. No more correction needed.")
            return large, small, True

        preferred = preferred_move_from_error(before_err)
        candidate_moves = [preferred]
        opposite = "LEFT" if preferred == "RIGHT" else "RIGHT"
        if PRE_EXTENSION_TRY_OPPOSITE_IF_NO_IMPROVEMENT:
            candidate_moves.append(opposite)

        accepted = False

        for move_index, move in enumerate(candidate_moves):
            if move_index == 0:
                print(f"Pre-extension X correction pulse {pulse}: trying preferred move {move}.")
            else:
                print(f"Preferred move did not improve enough. Trying opposite move {move}.")

            pre_extension_send_alignment_pulse(ser, move)
            time.sleep(SETTLE_AFTER_PRIMITIVE_SEC)
            camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

            tag_kind_after, after, desired_x_after, x_tol_after, large_after, small_after = get_current_pre_extension_target(
                camera,
                detector,
                parameters,
                dictionary,
                prefer_large=prefer_large,
                timeout_sec=2.0,
                force_small=force_small
            )
            last_large, last_small = large_after, small_after

            if after is None or tag_kind_after is None:
                print("Tag lost after pre-extension X correction. Undoing that pulse and stopping alignment.")
                undo_move(ser, move)
                time.sleep(SETTLE_AFTER_PRIMITIVE_SEC)
                camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)
                return last_large, last_small, False

            # v20: Even if this pulse was aiming at the large tag, immediately stop
            # if the small tag became good enough in X. This prevents extra correction
            # pulses after the small tag is already in a usable close-range pose.
            if small_tag_x_is_pre_extension_aligned_for_override(small_after):
                print("Pre-extension X alignment passed via SMALL tag override after correction.")
                return large_after, small_after, True

            # If the visible target switched from large to small because large disappeared,
            # continue with whatever is visible, but compute error using the current target.
            after_err = pre_extension_x_error(after, desired_x_after)
            after_abs = abs(after_err)
            improvement = before_abs - after_abs

            if tag_kind_after == "large":
                print_detection("After pre-extension LARGE X:", after, desired_x=desired_x_after)
            else:
                print_detection("After pre-extension SMALL X:", after, desired_x=desired_x_after, desired_y=desired_small_tag_err_y())

            print(
                f"Pre-extension X improvement = {improvement:.1f}px "
                f"after move {move}; new abs error={after_abs:.1f}px, "
                f"tolerance=±{x_tol_after:.1f}px"
            )

            if after_abs <= x_tol_after:
                print("Pre-extension X alignment reached strict tolerance.")
                return large_after, small_after, True

            if improvement >= PRE_EXTENSION_MIN_X_IMPROVEMENT_PX:
                print("Pre-extension X pulse accepted; continuing in same alignment loop if needed.")
                accepted = True
                break

            print("Pre-extension X pulse did not improve enough. Undoing pulse.")
            undo_move(ser, move)
            time.sleep(SETTLE_AFTER_PRIMITIVE_SEC)
            camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

        if not accepted:
            print("Neither preferred nor opposite pre-extension X pulse improved enough.")
            return last_large, last_small, False

    print("Reached maximum pre-extension X alignment pulses without satisfying strict tolerance.")
    return last_large, last_small, False


def align_before_extension_if_needed(ser, dxl, camera, detector, parameters, dictionary, large, small):
    """
    Perform real visual alignment immediately before one S-D-A inching extension group.

    IMPORTANT TIMING:
        This function should only be called after the body compression/repositioning
        and front DXL claw close/home phase are complete. It must not be called at
        the beginning of the approach loop or right after the back claw closes.

    Corrected behavior from log test:
        1. send x before correction pulses
        2. send a+d / a+s repeatedly until strict X tolerance is reached
        3. optionally undo accumulated a pulls using A
        4. only then send i before S-D-A extension
        5. if alignment is still bad, block the extension instead of pushing forward
    """
    if not ALIGN_BEFORE_EVERY_EXTENSION:
        return large, small, False

    print("\n" + "=" * 70)
    print("TIMED PRE-EXTENSION ALIGNMENT - HARD GATE")
    print("This should run only immediately before one S-D-A extension group.")
    print("Will not send S-D-A unless the selected tag is inside strict pre-extension X tolerance.")
    print("Policy: use large tag if visible; otherwise use/search small tag.")
    print_pre_extension_effective_tolerances()
    print("=" * 70)

    set_small_alignment_mode(ser)
    camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)

    large, small = get_both_tag_detections(
        camera,
        detector,
        parameters,
        dictionary,
        timeout_sec=2.0
    )
    print_tag_status("Pre-extension alignment starting tag status:", large, small)

    force_small_target = False

    if (
        LOCK_ON_SMALL_TAG_WHEN_FOUND
        and small is not None
        and small["size_px"] >= SMALL_TAG_LOCK_MIN_SIZE_PX
    ):
        print(
            "Pre-extension: small tag is already visible and big enough. "
            "Locking alignment target to SMALL tag instead of large tag."
        )
        force_small_target = True

    # If the large tag is close but the small tag is missing, actively try to find
    # the small tag with the existing right-biased search. This is the final-stage
    # behavior you wanted: when small should be nearby, look for it instead of only
    # continuing large-tag alignment.
    if (
        SEARCH_SMALL_TAG_WHEN_LARGE_IS_CLOSE
        and small is None
        and large is not None
        and large["size_px"] >= LARGE_SIZE_TO_FORCE_SMALL_SEARCH_PX
    ):
        print(
            "Pre-extension: large tag is close but small tag is missing. "
            "Trying right-biased search to recover small tag before more extension."
        )
        small_found, large_found, acquisition_status = search_for_small_tag_bias_right_only_when_large_missing(
            ser, camera, detector, parameters, dictionary
        )
        print(f"Pre-extension close-large small-search status: {acquisition_status}")

        if small_found is not None:
            small = small_found
            large = large_found
            if PRE_EXTENSION_FORCE_SMALL_TARGET_AFTER_SEARCH:
                print("Small tag found. Locking pre-extension alignment target to SMALL tag.")
                force_small_target = True
        elif large_found is not None:
            large = large_found
            small = None

    # If no tags are visible, first use the existing right-biased search to recover a tag.
    if large is None and small is None:
        print("Pre-extension: no tags visible. Trying right-biased small-tag search before extension.")
        small_found, large_found, acquisition_status = search_for_small_tag_bias_right_only_when_large_missing(
            ser, camera, detector, parameters, dictionary
        )
        print(f"Pre-extension acquisition status: {acquisition_status}")

        if small_found is not None:
            if small_tag_is_visibility_reveal_dock_candidate(small_found, acquisition_status):
                print(
                    "Pre-extension: a short right-search revealed a close small tag after a recent aligned large tag. "
                    "Treating this as a visibility-reveal docking candidate instead of chasing small-tag X."
                )
                remember_visibility_reveal_dock_candidate(small_found, large_found)
                set_inching_interrupt_reason("dock_candidate")
                # Return no visible small tag to the caller so it does not run final small-tag alignment
                # and undo the good physical pose. maybe_final_dxl_docking_close(dxl, None) will use
                # the remembered close candidate.
                return None, None, True

            small = small_found
            large = large_found
            if PRE_EXTENSION_FORCE_SMALL_TARGET_AFTER_SEARCH:
                print("Small tag found from no-tag search. Locking target to SMALL tag.")
                force_small_target = True
        elif large_found is not None:
            large = large_found
            small = None
        elif recent_last_known_close_dock_candidate():
            print("Pre-extension: tags missing, but last known state was close. Interrupt for docking attempt.")
            set_inching_interrupt_reason("dock_candidate")
            return None, None, True
        else:
            print("Pre-extension: no tag recovered. Aborting this inching cycle to avoid blind S-D-A push.")
            set_inching_interrupt_reason("alignment_failed")
            return None, None, ABORT_INCHING_ON_PRE_EXTENSION_ALIGNMENT_FAIL

    # Run the actual X correction loop. Near final stage, if the small tag has
    # been found, force small-target tracking so the large tag cannot steal focus.
    large, small, aligned_ok = strict_pre_extension_x_align_loop(
        ser,
        camera,
        detector,
        parameters,
        dictionary,
        prefer_large=(not force_small_target),
        force_small=force_small_target
    )

    if PRE_EXTENSION_CLEANUP_A_PULLS_AFTER_X_ALIGNMENT:
        print("Pre-extension: optional cleanup of accumulated a pulls after X alignment.")
        cleanup_top_a_pulls_after_large_alignment(ser)
        camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

    large, small = get_both_tag_detections(
        camera,
        detector,
        parameters,
        dictionary,
        timeout_sec=2.0
    )
    print_tag_status("Pre-extension final tag status after X alignment/cleanup:", large, small)

    if large is not None or small is not None:
        remember_tag_state(large, small, reason="strict pre-extension alignment")

    if small is not None and is_dock_ready_from_small_tag(small):
        print("Pre-extension: small tag is at measured docking pose. Interrupt before extension for DXL close.")
        set_inching_interrupt_reason("dock_candidate")
        return large, small, True

    # Re-evaluate after cleanup/camera refresh. v20: if the small tag is visible
    # and X-acceptable, it is allowed to pass even if the large tag is also visible
    # but slightly outside tolerance. Close-range docking should not chase the large
    # tag after the small tag is already good.
    final_ok = False
    if small is not None and small_tag_x_is_pre_extension_aligned_for_override(small):
        print("Pre-extension final check passed via SMALL tag X override.")
        final_ok = True
    elif large is not None:
        final_ok = large_tag_x_is_pre_extension_aligned(large)
    elif small is not None:
        final_ok = small_tag_is_pre_extension_aligned(small)

    if not aligned_ok or not final_ok:
        print("\nPRE-EXTENSION ALIGNMENT FAILED STRICT CHECK.")
        print("S-D-A extension will be blocked so the robot does not keep pushing while offset.")
        print("Aborting the rest of this inching cycle cleanly so the inching sequence does not get messed up.")
        if ABORT_INCHING_ON_PRE_EXTENSION_ALIGNMENT_FAIL:
            set_inching_interrupt_reason("alignment_failed")
            return large, small, True

    print("Pre-extension alignment passed. Returning to inching mode before S-D-A extension.")
    send_char(ser, INCHING_STEP_COMMAND)
    time.sleep(0.2)
    camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)

    return large, small, False

def extend_body_repeated(ser, repeat_count):
    print("\nExtending body with repeated S, D, A groups...")
    print(f"Extension repeat count = {repeat_count}")

    for i in range(1, repeat_count + 1):
        print(f"Extension group {i}/{repeat_count}: S, D, A")
        send_sequence(ser, EXTEND_SEQUENCE, track_top_a_balance=False)
        print(f"Waiting {EXTEND_GROUP_WAIT_SEC:.1f} sec after extension group...")
        time.sleep(EXTEND_GROUP_WAIT_SEC)

    print("Extension phase complete.")


def inching_cycle_with_dynamixel(ser, dxl, camera, detector, parameters, dictionary, extend_repeat_count):
    clear_inching_interrupt_reason()
    print("\nStarting one inching cycle WITH Dynamixel...")
    print("This version monitors tag size/alignment after every inching sub-step.")

    def check_stage(stage_label):
        large_c, small_c, interrupt_c = monitor_tags_after_inching_stage(
            camera,
            detector,
            parameters,
            dictionary,
            stage_label
        )
        if interrupt_c:
            set_inching_interrupt_reason("dock_candidate")
        return large_c, small_c, interrupt_c

    print("Setting inching displacement: i")
    send_char(ser, INCHING_STEP_COMMAND)
    time.sleep(0.2)

    large, small, interrupt = check_stage("selecting inching mode i")
    if interrupt:
        return large, small, True

    print("Closing back claw: e")
    send_char(ser, BACK_CLAW_CLOSE)
    print(f"Waiting {BACK_CLAW_WAIT_SEC:.1f} sec for back claw close...")
    time.sleep(BACK_CLAW_WAIT_SEC)

    large, small, interrupt = check_stage("back claw close e")
    if interrupt:
        return large, small, True

    # Do NOT run strict pre-extension alignment here.
    # The body has not completed compression/repositioning yet, so aligning now can
    # over-correct from the wrong mechanical state. Alignment is delayed until
    # after collapse/open/close are complete, immediately before each S-D-A group.

    print("Opening front Dynamixel claw and waiting until reached.")
    dxl.open_front_claw_blocking()

    large, small, interrupt = check_stage("front DXL claw open")
    if interrupt:
        return large, small, True

    print("Collapsing body: q")
    send_char(ser, BODY_COLLAPSE)
    print(f"Waiting {COLLAPSE_WAIT_SEC:.1f} sec for collapse...")
    time.sleep(COLLAPSE_WAIT_SEC)

    large, small, interrupt = check_stage("body collapse q")
    if interrupt:
        return large, small, True

    print("Opening back claw: E")
    send_char(ser, BACK_CLAW_OPEN)
    print(f"Waiting {BACK_CLAW_WAIT_SEC:.1f} sec for back claw open...")
    time.sleep(BACK_CLAW_WAIT_SEC)

    large, small, interrupt = check_stage("back claw open E")
    if interrupt:
        return large, small, True

    print("Closing front Dynamixel claw back to HOME/CLOSED and waiting until reached.")
    dxl.close_front_claw_blocking()

    large, small, interrupt = check_stage("front DXL claw close/home")
    if interrupt:
        return large, small, True

    print("\nExtending body with repeated S, D, A groups...")
    print(f"Extension repeat count = {extend_repeat_count}")
    print("Timing policy:")
    print("  - After front DXL claw close/home, send the FIRST S-D-A group immediately.")
    print("  - Before every later S-D-A group, switch to x, align, then switch back to i.")

    for i in range(1, extend_repeat_count + 1):
        if FIRST_EXTENSION_GROUP_WITHOUT_ALIGNMENT and i == 1:
            print("\n" + "=" * 70)
            print(f"EXTENSION GROUP {i}/{extend_repeat_count}: first push after DXL close, no pre-alignment")
            print("Reason: use one extension cycle to settle/reposition before doing visual correction.")
            print("=" * 70)
            send_char(ser, INCHING_STEP_COMMAND)
            time.sleep(0.2)
        else:
            print("\n" + "=" * 70)
            print(f"PRE-EXTENSION ALIGNMENT BEFORE EXTENSION GROUP {i}/{extend_repeat_count}")
            print("Timing: at least one S-D-A group has already happened after DXL close/home.")
            print("Now align with small-displacement x before sending this S-D-A push.")
            print("=" * 70)

            large, small, interrupt = align_before_extension_if_needed(
                ser,
                dxl,
                camera,
                detector,
                parameters,
                dictionary,
                large,
                small
            )
            if interrupt:
                print("Pre-extension alignment interrupted/failed or docking is close. Not sending this S-D-A group.")
                return large, small, True

        print(f"Extension group {i}/{extend_repeat_count}: S, D, A")
        send_sequence(ser, EXTEND_SEQUENCE, track_top_a_balance=False)
        print(f"Waiting {EXTEND_GROUP_WAIT_SEC:.1f} sec after extension group...")
        time.sleep(EXTEND_GROUP_WAIT_SEC)

        large, small, interrupt = check_stage(f"extension group {i} S-D-A")
        if interrupt:
            print("Stopping extension early because tag monitor says docking is close enough.")
            return large, small, True

    print("Extension phase complete.")

    camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

    large, small = get_both_tag_detections(
        camera,
        detector,
        parameters,
        dictionary,
        timeout_sec=2.0
    )

    print_tag_status("After inching tag status:", large, small)
    remember_tag_state(large, small, reason="after full inching cycle")

    return large, small, False

def handle_confirmed_dock_ready(dxl, final_result):
    if final_result is None:
        if recent_last_known_close_dock_candidate():
            print("\nDock-ready inferred from recent close/aligned tag state after tag occlusion.")
            maybe_final_dxl_docking_close(dxl, None)
            return True
        return False

    if not is_dock_ready_from_small_tag(final_result):
        return False

    print("\nDock-ready confirmed from visible small tag.")
    maybe_final_dxl_docking_close(dxl, final_result)
    return True


def adaptive_approach_and_final_align(ser, dxl, camera, detector, parameters, dictionary):
    print("\nStarting simplified visibility-based adaptive approach + measured-pose docking.")
    print(f"Max inching cycles = {MAX_INCHING_CYCLES}")
    print("Policy: use large tag whenever visible; use small tag when large tag disappears.")
    print("Switching is no longer based on large/small tag size thresholds.")

    large, small = get_both_tag_detections(
        camera, detector, parameters, dictionary, timeout_sec=2.0
    )
    print_tag_status("Initial tag status before adaptive approach:", large, small)
    if large is not None or small is not None:
        remember_tag_state(large, small, reason="initial detection")

    # Do NOT run the strict pre-extension alignment here.
    # That hard gate is only for the inching extension timing:
    # after body compression/front-claw-close is complete, immediately before each S-D-A group.

    for cycle in range(1, MAX_INCHING_CYCLES + 1):
        print("\n" + "#" * 70)
        print(f"VISIBILITY-BASED ADAPTIVE LOOP {cycle}/{MAX_INCHING_CYCLES}")
        print("#" * 70)

        large, small = get_both_tag_detections(
            camera, detector, parameters, dictionary, timeout_sec=2.0
        )
        print_tag_status("Before deciding next action:", large, small)
        if large is not None or small is not None:
            remember_tag_state(large, small, reason=f"loop {cycle} start")

        if small is not None and is_dock_ready_from_small_tag(small):
            print("Small tag is already at the measured docking pose. Running docking close.")
            if handle_confirmed_dock_ready(dxl, small):
                return

        if large is not None:
            print("Large tag visible: align with large tag first.")
            if not large_tag_x_is_aligned(large):
                align_x_to_large_tag(ser, camera, detector, parameters, dictionary)
                cleanup_top_a_pulls_after_large_alignment(ser)
                camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

            large, small = get_both_tag_detections(
                camera, detector, parameters, dictionary, timeout_sec=2.0
            )
            print_tag_status("After large-tag alignment/check:", large, small)
            if large is not None or small is not None:
                remember_tag_state(large, small, reason="after large-tag alignment")

            if small is not None and is_dock_ready_from_small_tag(small):
                print("Small tag reached measured docking pose after large-tag alignment.")
                if handle_confirmed_dock_ready(dxl, small):
                    return

        elif small is not None:
            print("Large tag missing, small tag visible: use small tag measured-pose alignment.")
            result = final_align_small_tag_y_then_x(
                ser, camera, detector, parameters, dictionary
            )
            if result is not None:
                remember_tag_state(None, result, reason="main-loop small-tag alignment")
                if handle_confirmed_dock_ready(dxl, result):
                    return

        else:
            print("No tags visible at loop start. Trying right-biased small-tag search.")
            small_found, large_found, acquisition_status = search_for_small_tag_bias_right_only_when_large_missing(
                ser, camera, detector, parameters, dictionary
            )
            print(f"Acquisition status after no-tag search: {acquisition_status}")

            if small_found is not None:
                result = final_align_small_tag_y_then_x(
                    ser, camera, detector, parameters, dictionary
                )
                if result is not None:
                    remember_tag_state(None, result, reason="no-tag search found small")
                    if handle_confirmed_dock_ready(dxl, result):
                        return

            elif large_found is not None:
                print("Large tag found during blind search. Aligning large tag.")
                align_x_to_large_tag(ser, camera, detector, parameters, dictionary)
                cleanup_top_a_pulls_after_large_alignment(ser)
                camera.flush_frames(duration_sec=FLUSH_AFTER_MOVE_SEC)

            elif recent_last_known_close_dock_candidate():
                print("No tags visible, but last known state was close. Trying final DXL docking close.")
                maybe_final_dxl_docking_close(dxl, None)
                return

            elif not ALLOW_BLIND_INCHING_WITH_NO_TAGS:
                print("Blind inching is disabled and no recent close tag state exists. Stopping.")
                return

        large, small = get_both_tag_detections(
            camera, detector, parameters, dictionary, timeout_sec=2.0
        )
        print_tag_status("Before inching decision:", large, small)

        if not should_inch_more(large, small):
            print("Decision: do not inch more.")
            if small is not None:
                handle_confirmed_dock_ready(dxl, small)
            elif recent_last_known_close_dock_candidate():
                maybe_final_dxl_docking_close(dxl, None)
            return

        extend_repeat_count = choose_extension_repeat_count(large, small)
        print(f"Extension repeat count selected: {extend_repeat_count}")

        large, small, inching_interrupted = inching_cycle_with_dynamixel(
            ser, dxl, camera, detector, parameters, dictionary,
            extend_repeat_count=extend_repeat_count
        )

        set_small_alignment_mode(ser)
        camera.flush_frames(duration_sec=FINAL_FLUSH_SEC)

        if inching_interrupted:
            print(f"Inching was interrupted. Reason: {last_inching_interrupt_reason}")

            if last_inching_interrupt_reason == "alignment_failed":
                print("Pre-extension alignment failed, so the rest of the inching cycle was aborted cleanly.")
                print("Checking whether docking is still possible before stopping.")
                if small is not None and is_dock_ready_from_small_tag(small):
                    maybe_final_dxl_docking_close(dxl, small)
                    return
                if recent_last_known_close_dock_candidate():
                    print("Trying final DXL docking close from recent close tag state.")
                    maybe_final_dxl_docking_close(dxl, None)
                    return
                print("Docking is not safe/ready. Stopping instead of continuing a messed-up inching sequence.")
                return

            print("Inching was interrupted by close/dock candidate or occlusion.")
            if small is not None:
                result = final_align_small_tag_y_then_x(
                    ser, camera, detector, parameters, dictionary
                )
                if handle_confirmed_dock_ready(dxl, result):
                    return

            if recent_last_known_close_dock_candidate():
                print("Trying final DXL docking close from recent close tag state.")
                maybe_final_dxl_docking_close(dxl, None)
                return

    print("\nReached maximum inching cycles.")
    print("Trying final small-tag alignment/docking one last time if possible.")

    large, small = get_both_tag_detections(
        camera, detector, parameters, dictionary, timeout_sec=2.0
    )
    if small is not None:
        result = final_align_small_tag_y_then_x(ser, camera, detector, parameters, dictionary)
        handle_confirmed_dock_ready(dxl, result)
    elif recent_last_known_close_dock_candidate():
        maybe_final_dxl_docking_close(dxl, None)
    else:
        print("No small tag and no recent close tag state. Not docking.")


# ==========================================================
# MAIN
# ==========================================================


def main():
    ser = None
    camera = None
    dxl = None
    robot_connected = False

    try:
        assert_dxl_constants_safe()

        print("\n" + "=" * 80)
        print("STEP 1: OPEN DYNAMIXEL AND HOME FRONT CLAW BEFORE ANYTHING ELSE")
        print("=" * 80)
        print(f"DXL hard limits: [{DXL_HARD_MIN_POS}, {DXL_HARD_MAX_POS}]")
        print(f"DXL home/closed pos: {FRONT_CLAW_HOME_POS}")
        print(f"DXL open pos: {FRONT_CLAW_OPEN_POS}")
        print(f"DXL docking pos: {FRONT_CLAW_DOCKING_POS}")
        print(f"ENABLE_FINAL_DXL_DOCKING_CLOSE = {ENABLE_FINAL_DXL_DOCKING_CLOSE}")

        dxl = FrontClawDynamixel(DXL_PORT, DXL_BAUD, DXL_ID)
        dxl.open()

        print("\nHOMING FRONT CLAW NOW.")
        print("No ESP32, no camera alignment, and no inching will happen until this finishes.")
        dxl.home_front_claw_blocking()
        print("FRONT CLAW HOMED SUCCESSFULLY. Continuing.\n")

        print("\n" + "=" * 80)
        print("STEP 2: CONNECT ESP32 ROBOT ONLY AFTER DXL HOME")
        print("=" * 80)

        print("Opening ESP32 serial...")
        ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
        print(f"Opened ESP32 on {ESP32_PORT} at {ESP32_BAUD} baud.")

        initialize_robot(ser)
        robot_connected = True

        if HOME_AT_START:
            print("Sending body home command Q at start.")
            send_char(ser, HOME_COMMAND)
            time.sleep(1.0)
            send_char(ser, ALIGNMENT_SMALL_STEP_COMMAND)
            time.sleep(0.2)

        print("\n" + "=" * 80)
        print("STEP 3: START CAMERA AND RUN ALIGNMENT/INCHING")
        print("=" * 80)

        detector, parameters, dictionary = create_aruco_detector()
        camera = RpiCamFrameReader()

        time.sleep(1.0)
        camera.flush_frames(duration_sec=1.0)

        print("\nController started.")
        print(f"ESP32 port: {ESP32_PORT}")
        print(f"Dynamixel port: {DXL_PORT}")
        print("Dynamixel mode: EXTENDED POSITION CONTROL")
        print(f"DXL HARD LIMITS: [{DXL_HARD_MIN_POS}, {DXL_HARD_MAX_POS}]")
        print(f"Front claw home/closed pos: {FRONT_CLAW_CLOSED_POS}")
        print(f"Front claw open pos: {FRONT_CLAW_OPEN_POS}")
        print(f"Final docking close pos: {FRONT_CLAW_DOCKING_POS}")
        print(f"ENABLE_FINAL_DXL_DOCKING_CLOSE = {ENABLE_FINAL_DXL_DOCKING_CLOSE}")
        print(f"Large tag ID {APPROACH_TAG_ID}: {APPROACH_TAG_SIZE_MM} mm")
        print(f"Small tag ID {FINAL_TAG_ID}: {FINAL_TAG_SIZE_MM} mm")
        print(f"Small tag physical X offset: {FINAL_TAG_X_OFFSET_SIGN * FINAL_TAG_X_OFFSET_MM} mm")
        print("Switch policy: visibility-based. Use large tag when visible; use small tag when large tag is missing.")
        print(f"Measured final docking target: small tag size≈{FINAL_DOCK_TARGET_SIZE_PX}px, err_x≈{FINAL_DOCK_TARGET_ERR_X_PX}px, err_y≈{FINAL_DOCK_TARGET_ERR_Y_PX}px")
        print(f"Dock-ready tolerance: size >= {SMALL_TAG_DOCK_READY_SIZE_PX}, |x-target| <= {SMALL_TAG_DOCK_MAX_ABS_CONTROL_ERR_X}, |y-target| <= {SMALL_TAG_DOCK_MAX_ABS_ERR_Y}")
        print(f"DOCK_READY_USE_Y_GATE = {DOCK_READY_USE_Y_GATE}")
        print(f"PRE_EXTENSION_USE_SMALL_Y_GATE = {PRE_EXTENSION_USE_SMALL_Y_GATE}")
        print(f"Image: {WIDTH} x {HEIGHT}, center x: {WIDTH / 2:.1f}, center y: {HEIGHT / 2:.1f}")
        print(f"Max inching cycles: {MAX_INCHING_CYCLES}")
        print(f"DISPLAY_LIVE_CAMERA = {DISPLAY_LIVE_CAMERA}")
        print(f"RECORD_CAMERA_STREAM = {RECORD_CAMERA_STREAM}")
        print(f"INVERT_FINAL_Y = {INVERT_FINAL_Y}")
        print(f"CLEANUP_EXTRA_A_AFTER_FINAL_X = {CLEANUP_EXTRA_A_AFTER_FINAL_X}")
        print(f"CLEANUP_A_PULLS_AFTER_LARGE_ALIGNMENT = {CLEANUP_A_PULLS_AFTER_LARGE_ALIGNMENT}")
        print(f"ALIGN_BEFORE_EVERY_EXTENSION = {ALIGN_BEFORE_EVERY_EXTENSION}")
        print(f"PRE_EXTENSION_ALIGN_USE_FINAL_SMALL_TAG_ALIGNMENT = {PRE_EXTENSION_ALIGN_USE_FINAL_SMALL_TAG_ALIGNMENT}")
        print(f"PRE_EXTENSION_ALIGN_RECHECK_AFTER_LARGE_ALIGNMENT = {PRE_EXTENSION_ALIGN_RECHECK_AFTER_LARGE_ALIGNMENT}")
        print(f"PRE_EXTENSION_LARGE_X_TOLERANCE_PX = {PRE_EXTENSION_LARGE_X_TOLERANCE_PX}")
        print(f"PRE_EXTENSION_SMALL_X_TOLERANCE_PX = {PRE_EXTENSION_SMALL_X_TOLERANCE_PX}")
        print(f"PRE_EXTENSION_SMALL_Y_TOLERANCE_PX = {PRE_EXTENSION_SMALL_Y_TOLERANCE_PX}")
        print(f"PRE_EXTENSION_ALIGNMENT_STRICTNESS_MULTIPLIER = {PRE_EXTENSION_ALIGNMENT_STRICTNESS_MULTIPLIER}")
        print(f"FIRST_EXTENSION_GROUP_WITHOUT_ALIGNMENT = {FIRST_EXTENSION_GROUP_WITHOUT_ALIGNMENT}")
        print(f"ABORT_INCHING_ON_PRE_EXTENSION_ALIGNMENT_FAIL = {ABORT_INCHING_ON_PRE_EXTENSION_ALIGNMENT_FAIL}")
        print(f"PRE_EXTENSION_MAX_X_ALIGNMENT_PULSES = {PRE_EXTENSION_MAX_X_ALIGNMENT_PULSES}")
        print(f"PRE_EXTENSION_MIN_X_IMPROVEMENT_PX = {PRE_EXTENSION_MIN_X_IMPROVEMENT_PX}")
        print(f"PRE_EXTENSION_SEND_X_BEFORE_EVERY_CORRECTION = {PRE_EXTENSION_SEND_X_BEFORE_EVERY_CORRECTION}")
        print(f"PRE_EXTENSION_CLEANUP_A_PULLS_AFTER_X_ALIGNMENT = {PRE_EXTENSION_CLEANUP_A_PULLS_AFTER_X_ALIGNMENT}")
        print(f"BLOCK_EXTENSION_IF_PRE_EXTENSION_ALIGNMENT_FAILS = {BLOCK_EXTENSION_IF_PRE_EXTENSION_ALIGNMENT_FAILS}")
        print(f"PRE_EXTENSION_SMALL_VISIBLE_X_OVERRIDE_ENABLED = {PRE_EXTENSION_SMALL_VISIBLE_X_OVERRIDE_ENABLED}")
        print(f"PRE_EXTENSION_SMALL_VISIBLE_OVERRIDE_MIN_SIZE_PX = {PRE_EXTENSION_SMALL_VISIBLE_OVERRIDE_MIN_SIZE_PX}")
        print(f"LOCK_ON_SMALL_TAG_WHEN_FOUND = {LOCK_ON_SMALL_TAG_WHEN_FOUND}")
        print(f"SMALL_TAG_LOCK_MIN_SIZE_PX = {SMALL_TAG_LOCK_MIN_SIZE_PX}")
        print(f"SEARCH_SMALL_TAG_WHEN_LARGE_IS_CLOSE = {SEARCH_SMALL_TAG_WHEN_LARGE_IS_CLOSE}")
        print(f"LARGE_SIZE_TO_FORCE_SMALL_SEARCH_PX = {LARGE_SIZE_TO_FORCE_SMALL_SEARCH_PX}")
        print(f"PRE_EXTENSION_FORCE_SMALL_TARGET_AFTER_SEARCH = {PRE_EXTENSION_FORCE_SMALL_TARGET_AFTER_SEARCH}")
        print_pre_extension_effective_tolerances()
        print(f"USE_ADAPTIVE_EXTENSION_REPEAT = {USE_ADAPTIVE_EXTENSION_REPEAT}")
        print(f"MONITOR_TAGS_DURING_INCHING = {MONITOR_TAGS_DURING_INCHING}")
        print(f"INTERRUPT_INCHING_IF_DOCK_CANDIDATE = {INTERRUPT_INCHING_IF_DOCK_CANDIDATE}")
        print(f"ALLOW_FINAL_DXL_CLOSE_FROM_LAST_CLOSE_TAG_WHEN_OCCLUDED = {ALLOW_FINAL_DXL_CLOSE_FROM_LAST_CLOSE_TAG_WHEN_OCCLUDED}")
        print(f"USE_CURRENT_AWARE_DXL_DOCKING_CLOSE = {USE_CURRENT_AWARE_DXL_DOCKING_CLOSE}")
        print(f"DXL_DOCK_CLOSE_UNTIL_CURRENT = {DXL_DOCK_CLOSE_UNTIL_CURRENT}")
        print(f"DXL_DOCK_STOP_CURRENT_A = {DXL_DOCK_STOP_CURRENT_A}")
        print(f"DXL_DOCK_FORCE_TARGET_USES_HARD_MIN = {DXL_DOCK_FORCE_TARGET_USES_HARD_MIN}")
        print(f"DXL_HOLD_AFTER_DOCKING_SUCCESS = {DXL_HOLD_AFTER_DOCKING_SUCCESS}")
        print(f"DXL_DISABLE_TORQUE_ON_EXIT = {DXL_DISABLE_TORQUE_ON_EXIT}")
        print(f"SEND_Q_ON_DOCKING_FAILURE = {SEND_Q_ON_DOCKING_FAILURE}")
        print(f"SMALL_TAG_SEARCH_MAX_RIGHT_STEPS = {SMALL_TAG_SEARCH_MAX_RIGHT_STEPS}")
        print(f"ALLOW_BLIND_INCHING_WITH_NO_TAGS = {ALLOW_BLIND_INCHING_WITH_NO_TAGS}")
        print(f"ALLOW_INCHING_WHEN_LARGE_TAG_IS_CLOSE_BUT_SMALL_TAG_LOST = {ALLOW_INCHING_WHEN_LARGE_TAG_IS_CLOSE_BUT_SMALL_TAG_LOST}")
        print(f"VISIBILITY_REVEAL_DOCK_CANDIDATE_ENABLED = {VISIBILITY_REVEAL_DOCK_CANDIDATE_ENABLED}")
        print(f"VISIBILITY_REVEAL_MAX_RIGHT_SEARCH_STEPS = {VISIBILITY_REVEAL_MAX_RIGHT_SEARCH_STEPS}")
        print(f"VISIBILITY_REVEAL_MIN_SMALL_SIZE_PX = {VISIBILITY_REVEAL_MIN_SMALL_SIZE_PX}")
        print(f"VISIBILITY_REVEAL_MAX_ABS_SMALL_CONTROL_Y_PX = {VISIBILITY_REVEAL_MAX_ABS_SMALL_CONTROL_Y_PX}")
        print(f"VISIBILITY_REVEAL_MIN_RECENT_LARGE_SIZE_PX = {VISIBILITY_REVEAL_MIN_RECENT_LARGE_SIZE_PX}")
        print(f"VISIBILITY_REVEAL_MAX_RECENT_LARGE_ABS_X_PX = {VISIBILITY_REVEAL_MAX_RECENT_LARGE_ABS_X_PX}")
        print(f"HOME_BODY_BEFORE_ADAPTIVE_APPROACH = {HOME_BODY_BEFORE_ADAPTIVE_APPROACH}")
        print(f"HOME_BODY_BEFORE_ADAPTIVE_WAIT_SEC = {HOME_BODY_BEFORE_ADAPTIVE_WAIT_SEC}")
        print(f"SEND_X_AFTER_STARTUP_HOME = {SEND_X_AFTER_STARTUP_HOME}")

        home_body_before_normal_v20_logic(ser, camera)

        adaptive_approach_and_final_align(
            ser,
            dxl,
            camera,
            detector,
            parameters,
            dictionary
        )

        print("\nAdaptive alignment phase complete.")

        if ENABLE_FINAL_DXL_DOCKING_CLOSE:
            print("Final DXL docking close was enabled. Check log above to confirm whether it ran.")
        else:
            print("Final DXL docking close was disabled. Robot stopped after alignment.")

        if HOME_AT_END:
            print("Sending body home command Q at end.")
            send_char(ser, HOME_COMMAND)
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

        if ser is not None and HOME_ON_INTERRUPT:
            print("Sending body home command Q.")
            send_char(ser, HOME_COMMAND)
            time.sleep(0.5)

    finally:
        if ser is not None and robot_connected and SEND_Q_ON_DOCKING_FAILURE and not docking_successful:
            try:
                print("\nDocking was not confirmed successful. Sending body home command Q before quitting.")
                send_char(ser, HOME_COMMAND)
                time.sleep(1.0)
            except Exception as e:
                print(f"Warning: failed to send Q after docking failure: {e}")

        if ser is not None and robot_connected and DISCONNECT_AT_END:
            try:
                disconnect_robot(ser)
            except Exception as e:
                print(f"Warning: failed to disconnect robot cleanly: {e}")

        if camera is not None:
            try:
                camera.stop()
            except Exception as e:
                print(f"Warning: failed to stop camera cleanly: {e}")

        if dxl is not None:
            try:
                dxl.close_port()
            except Exception as e:
                print(f"Warning: failed to close Dynamixel cleanly: {e}")

        if ser is not None:
            try:
                ser.close()
            except Exception as e:
                print(f"Warning: failed to close ESP32 serial cleanly: {e}")

        print("Done.")


if __name__ == "__main__":
    main()