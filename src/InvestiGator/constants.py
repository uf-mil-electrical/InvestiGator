from pymavlink.dialects.v20 import ardupilotmega as mavlink

# Commands for COMMAND_LONG message
MIL_MISSION_CMD = mavlink.MAV_CMD_USER_1
MIL_SYSTEM_CMD = mavlink.MAV_CMD_USER_4

# System Status for heartbeat (uint8_t)
MIL_STATE_CONNECTING = 9
MIL_STATE_STANDBY = 10
MIL_STATE_OVERRIDE = 11
MIL_STATE_MISSION = 12