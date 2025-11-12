from dataclasses import dataclass
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from collections import namedtuple
from threading import Lock

MavFrameGlobal = namedtuple("MavFrameGlobal", ["lattitude_int", "longitude_int", "altitude_m"])
MavFrameLocalNed = namedtuple("LocalNED", ["x_north_m", "y_east_m", "z_down_m"])
MavFrameGlobalRel = namedtuple("GlobalRelative",["lattitude_int", "longitude_int", "altitude_rel_m"])
MavFrameLocalENU = namedtuple("LocalENU", ["x_east_m", "y_north_m", "z_up_m"])
MavFrameLocalOffsetNED = namedtuple("LocalOffsetNED", ["x_north_m", "y_east_m", "z_down_m"])
MavFrameBodyFRD = namedtuple("BodyFRD", ["x_forward_m", "y_right_m", "z_down_m"])
MavFrameLocalFRD = namedtuple("LocalFRD", ["x_forward_m", "y_right_m", "z_down_m"])
MavFrameLocalFLU = namedtuple("LocalFLU", ["x_forward_m", "y_left_m", "z_up_m"])
Attitude = namedtuple("Attitude", ["roll_rad", "pitch_rad", "yaw_rad", "rollspeed_rad_s", "pitchspeed_rad_s", "yawspeed_rad_s"])

class Location(object):
    """
    Represents location of the vehicle and provides methods to return location wrapped in different location types.
    """

    def __init__(self, vehicle):
        self.lock = Lock()

        self.lat_int = None
        self.lon_int = None
        self.alt_m = None
        self.relative_alt_m = None

        self.x_north_m = None
        self.y_east_m = None
        self.z_down_m = None

        self.roll_rad = None
        self.pitch_rad = None
        self.yaw_rad = None
        self.rollspeed_rad_s = None
        self.pitchspeed_rad_s = None
        self.yawspeed_rad_s = None

        @vehicle.subscribe(mavlink.mavlink_map[mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT].msgname)
        def update_global_position(message: mavlink.MAVLink_global_position_int_message):
            with self.lock:
                self.lat_int = message.lat # Divide by 1E7 to convert to degrees
                self.lon_int = message.lon # Divide by 1E7 to convert to degrees
                self.alt_m = message.alt / 1E3 # Given in mm
                self.relative_alt_m = message.relative_alt / 1E3  # Given in mm

        @vehicle.subscribe(mavlink.MAVLink_local_position_ned_message.msgname)
        def update_local_position(message: mavlink.MAVLink_local_position_ned_message):
            with self.lock:
                self.x_north_m = message.x
                self.y_east_m = message.y
                self.z_down_m = message.z  # Negative altitude

        @vehicle.subscribe(mavlink.MAVLink_attitude_message.msgname)
        def update_attitude(message: mavlink.MAVLink_attitude_message):
            with self.lock:
                self.roll_rad = message.roll
                self.pitch_rad = message.pitch
                self.yaw_rad = message.yaw
                self.rollspeed_rad_s = message.rollspeed
                self.pitchspeed_rad_s = message.pitchspeed
                self.yawspeed_rad_s = message.yawspeed

    @property
    def global_frame(self):
        """
        Returns location as a MAV_FRAME_GLOBAL frame. Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL).
        """
        with self.lock:
            return MavFrameGlobal(self.lat_int, self.lon_int, self.alt_m)
        
    @property
    def global_frame_relative(self):
        """
        Returns location as a MAV_FRAME_GLOBAL frame. Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL).
        """
        with self.lock:
            return MavFrameGlobalRel(self.lat_int, self.lon_int, self.relative_alt_m)

    @property
    def local_ned(self):
        """"
        NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
        """
        with self.lock:
            return MavFrameLocalNed(self.x_north_m, self.y_east_m, self.z_down_m)
    
    @property
    def body_frd(self):
        """
        FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle.
        """
        #TODO: Create a subscriber for attitude. Calculate FRD position from Local NED and Orientatio.
        pass

    @property
    def attitude(self):
        """
        The attitude in the aeronautical frame (right-handed, Z-down, Y-right, X-front, ZYX, intrinsic)
        Roll, Pitch, Yaw in (-pi, pi)
        """
        with self.lock:
            return Attitude(self.roll_rad, self.pitch_rad, self.yaw_rad, self.rollspeed_rad_s, self.pitchspeed_rad_s, self.yawspeed_rad_s)


class Status(object):
    """
    Information received from the vehicle's heartbeat.
    """

    def __init__(self, vehicle):
        self.lock = Lock()
        self.mode_dict = mavlink.enums["COPTER_MODE"]
        self.mode = None
        self.type = None
        self.autopilot = None
        self.base_mode = None
        self.custom_mode = None
        self.system_status = None

        @vehicle.subscribe("HEARTBEAT")
        def subscription_update(message: mavlink.MAVLink_heartbeat_message):
            with self.lock:
                self.type = message.type
                self.autopilot = message.autopilot
                self.base_mode = message.base_mode
                self.custom_mode = message.custom_mode
                self.system_status = message.system_status
    
    @property
    def armed(self):
        with self.lock:
            if self.base_mode is None:
                return False
            return bool(self.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    # @property
    # def mode(self):
    #     return self.mode_dict[self.custom_mode]

    # @mode.setter
    # def mode(self, mode: mavlink.enums["COPTER_MODE"]):
    #     if mode not in mavlink.enums["COPTER_MODE"]:
    #         print("Invalid mode")
    #         return


