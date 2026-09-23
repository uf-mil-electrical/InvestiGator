from pymavlink.dialects.v20 import ardupilotmega as mavlink

# Commands for COMMAND_LONG message
MIL_MISSION_CMD = mavlink.MAV_CMD_USER_1
MIL_SYSTEM_CMD = mavlink.MAV_CMD_USER_4

# System Status for heartbeat (uint8_t)
MIL_STATE_CONNECTING = 9
MIL_STATE_STANDBY = 10
MIL_STATE_OVERRIDE = 11
MIL_STATE_MISSION = 12
MIL_STATE_INITIAL_OVERRIDE = 13

MIL_STATES = {
    MIL_STATE_CONNECTING: "MIL_STATE_CONNECTING",
    MIL_STATE_STANDBY: "MIL_STATE_STANDBY",
    MIL_STATE_OVERRIDE: "MIL_STATE_OVERRIDE",
    MIL_STATE_MISSION: "MIL_STATE_MISSION",
    MIL_STATE_INITIAL_OVERRIDE: "MIL_STATE_INITIAL_OVERRIDE"
}

# System Commands
MIL_SYSTEM_PING = 0
MIL_SYSTEM_GUIDED = 1
MIL_SYSTEM_OVERRIDE = 2
MIL_SYSTEM_CANCEL = 3

MIL_SYSTEM_CMDS = {
    MIL_SYSTEM_PING: "MIL_SYSTEM_PING",
    MIL_SYSTEM_GUIDED: "MIL_SYSTEM_GUIDED",
    MIL_SYSTEM_OVERRIDE: "MIL_SYSTEM_OVERRIDE",
    MIL_SYSTEM_CANCEL: "MIL_SYSTEM_CANCEL"
}

# Mirrors RxTask in robotx/rx_common.proto. Keep values in sync.
# Published by the companion computer in HEARTBEAT.custom_mode.
RX_TASK_UNKNOWN = 0
RX_TASK_NONE = 1
RX_TASK_SAFE_PASSAGE = 2
RX_TASK_INFRA_SURVEY_REPAIR = 3
RX_TASK_COORDINATED_LOGISTICS = 4
RX_TASK_DYNAMIC_INCIDENT = 5

# Mirrors FlightPhase in robotx/rx_common.proto. Keep values in sync.
FLIGHT_PHASE_UNKNOWN = 0
FLIGHT_PHASE_GROUNDED = 1
FLIGHT_PHASE_AIRBORNE = 2

# Mirrors RobotState in robocommand/common.proto. Keep values in sync.
# Only STATE_AUTO is mapped for the drone so far. KILLED and MANUAL are not defined yet.
ROBOT_STATE_UNKNOWN = 0;
ROBOT_STATE_KILLED = 1;
ROBOT_STATE_MANUAL = 2;
ROBOT_STATE_AUTO = 3;

# Mission numbers for MIL_MISSION_CMD param1. Append only. Never renumber.
MISSION_NUMBERS = {
    "ARUCO_LANDING": 0,
    "ARM": 1,
    "SQUARE_TEST": 2,
    "HOUR_GLASS": 3,
    "PIROUETTE": 4,
    "TEST_DOWN_YAW": 5,
    "WAIT_FOR_CANCEL": 6,
    "UPS_AND_DOWNS": 7,
    "RTL_BATT_TEST": 8,
    "GPS_TEST": 9,
}
MISSION_NUMBERS_REVERSE = {v: k for k, v in MISSION_NUMBERS.items()}

if len(MISSION_NUMBERS_REVERSE) != len(MISSION_NUMBERS):
    raise ImportError("MISSION_NUMBERS contains duplicate mission numbers.")
# ArduCopter fence parameter values. Confirmed against libraries/AC_Fence/AC_Fence.cpp in the local
# ArduPilot tree at the firmware actually running: ArduCopter V4.8.0-dev (4891432f).
# FENCE_ACTION and FENCE_TYPE are vehicle- and version-specific. Re-confirm if the Cube is reflashed.
FENCE_ACTION_RTL = 1  # @Values{Copter}: 1:RTL or Land

FENCE_TYPE_MAX_ALT = 1 << 0  # @Bitmask{Copter}: 0:Max altitude
FENCE_TYPE_CIRCLE = 1 << 1  # 1:Circle Centered on Home
FENCE_TYPE_POLYGON = 1 << 2  # 2:Inclusion/Exclusion Circles+Polygons
FENCE_TYPE_MIN_ALT = 1 << 3  # 3:Min altitude
# The course boundary polygon plus a ceiling. No circle fence, no min-altitude fence.
FENCE_TYPE_UAV = FENCE_TYPE_POLYGON | FENCE_TYPE_MAX_ALT

# FENCE_ALT_MAX_TP selects the altitude frame. It defaults to 1 (above home), so it must be set
# explicitly for the ceiling to mean AMSL.
FENCE_ALT_FRAME_AMSL = 0  # @Values: 0:Above sea level
FENCE_ALT_MAX_M = 60.0

# The fence is the course boundary itself. FENCE_MARGIN is how far inside it the autopilot warns and,
# with the fence bit in AVOID_ENABLE, stops the vehicle. FENCE_ACTION still fires at the boundary.
FENCE_MARGIN_M = 2.0
