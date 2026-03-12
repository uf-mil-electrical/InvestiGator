from pymavlink.dialects.v20 import ardupilotmega as mavlink

MIL_MISSION_CMD = mavlink.MAV_CMD_USER_1
MIL_MISSION_CANCEL = mavlink.MAV_CMD_USER_2
MIL_MISSION_ABORT = mavlink.MAV_CMD_USER_3
MIL_SYSTEM_CMD = mavlink.MAV_CMD_USER_4