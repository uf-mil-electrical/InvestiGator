from dataclasses import dataclass
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from collections import namedtuple
from threading import Lock
from pymavlink import mavutil
import math

from . import constants

MavFrameGlobal = namedtuple("MavFrameGlobal", ["lattitude_int", "longitude_int", "altitude_m"])
MavFrameLocalNed = namedtuple("LocalNED", ["x_north_m", "y_east_m", "z_down_m"])
MavFrameGlobalRel = namedtuple("GlobalRelative",["lattitude_int", "longitude_int", "altitude_rel_m"])
MavFrameLocalENU = namedtuple("LocalENU", ["x_east_m", "y_north_m", "z_up_m"])
MavFrameLocalOffsetNED = namedtuple("LocalOffsetNED", ["x_north_m", "y_east_m", "z_down_m"])
MavFrameBodyFRD = namedtuple("BodyFRD", ["x_forward_m", "y_right_m", "z_down_m"])
MavFrameLocalFRD = namedtuple("LocalFRD", ["x_forward_m", "y_right_m", "z_down_m"])
MavFrameLocalFLU = namedtuple("LocalFLU", ["x_forward_m", "y_left_m", "z_up_m"])
Attitude = namedtuple("Attitude", ["roll_rad", "pitch_rad", "yaw_rad", "rollspeed_rad_s", "pitchspeed_rad_s", "yawspeed_rad_s"])

class Location:
    """
    Represents location of the vehicle and provides methods to return location wrapped in different location types.
    """

    def __init__(self, vehicle):
        self.lock = Lock()

        self.lat_int = None
        self.lon_int = None
        self.alt_m = None
        self.relative_alt_m = None
        self.vx_cm_s = None
        self.vy_cm_s = None
        self.vz_cm_s = None
        self.hdg_cdeg = None
        self.alt_ellipsoid_mm = None

        self.x_north_m = None
        self.y_east_m = None
        self.z_down_m = None

        self.roll_rad = None
        self.pitch_rad = None
        self.yaw_rad = None
        self.rollspeed_rad_s = None
        self.pitchspeed_rad_s = None
        self.yawspeed_rad_s = None

        @vehicle.subscribe(mavlink.MAVLink_global_position_int_message.msgname)
        def update_global_position(message: mavlink.MAVLink_global_position_int_message):
            with self.lock:
                self.lat_int = message.lat # Divide by 1E7 to convert to degrees
                self.lon_int = message.lon # Divide by 1E7 to convert to degrees
                self.alt_m = message.alt / 1E3 # Given in mm
                self.relative_alt_m = message.relative_alt / 1E3  # Given in mm
                self.vx_cm_s = message.vx  # Ground X speed (latitude, positive north)
                self.vy_cm_s = message.vy  # Ground Y speed (longitude, positive east)
                self.vz_cm_s = message.vz  # Ground Z speed (altitude, positive down)
                self.hdg_cdeg = message.hdg  # Centidegrees, 65535 if unknown

        @vehicle.subscribe(mavlink.MAVLink_gps_raw_int_message.msgname)
        def update_gps_raw(message: mavlink.MAVLink_gps_raw_int_message):
            if message.get_srcSystem() != 1:
                return
            with self.lock:
                # Height above WGS84 ellipsoid in mm. MAVLink2 extension field, 0 on MAVLink1.
                self.alt_ellipsoid_mm = message.alt_ellipsoid

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
    def ground_speed_mps(self):
        """
        Horizontal speed over ground in m/s from GLOBAL_POSITION_INT velocity.
        """
        with self.lock:
            if self.vx_cm_s is None or self.vy_cm_s is None:
                return None
            return math.hypot(self.vx_cm_s, self.vy_cm_s) / 100.0

    @property
    def heading_deg(self):
        """
        Vehicle heading in degrees [0, 360) from GLOBAL_POSITION_INT. None if unknown.
        """
        with self.lock:
            if self.hdg_cdeg is None or self.hdg_cdeg == 65535:  # 65535 is the MAVLink "unknown" sentinel
                return None
            return self.hdg_cdeg / 100.0

    @property
    def altitude_hae_m(self):
        """
        Altitude above the WGS84 ellipsoid (HAE) in meters from GPS_RAW_INT. None if not reported.
        Deliberately no fallback to GLOBAL_POSITION_INT.alt, which is AMSL (a different datum).
        """
        with self.lock:
            if not self.alt_ellipsoid_mm:
                return None
            return self.alt_ellipsoid_mm / 1000.0

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


class Status:
    """
    Information received from the vehicle's heartbeat and system status messages.
    """

    def __init__(self, vehicle):
        self.lock = Lock()

        # Heartbeat Attributes
        self.mode_map_bynumber = mavutil.mode_mapping_bynumber(mavlink.MAV_TYPE_QUADROTOR)
        self.mode_map_byname = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)
        self.type = None
        self.autopilot = None
        self.base_mode = None
        self.custom_mode = None
        self.system_status = None

        # System Status Attributes
        self.onboard_control_sensors_present = None
        self.onboard_control_sensors_enabled = None
        self.onboard_control_sensors_health = None
        self.load = None
        self.voltage_battery = None
        self.current_battery = None
        self.battery_remaining = None
        self.drop_rate_comm = None
        self.errors_comm = None
        self.errors_count1 = None
        self.errors_count2 = None
        self.errors_count3 = None
        self.errors_count4 = None

        # Extended System State Attributes
        self.landed_state = None

        @vehicle.subscribe(mavlink.MAVLink_heartbeat_message.msgname)
        def subscription_update(message: mavlink.MAVLink_heartbeat_message):
            # TODO: Make this reflect the configurable source system from vehiclemanager
            if message.get_srcSystem() != 1:
                return
            
            with self.lock:
                self.type = message.type
                self.autopilot = message.autopilot
                self.base_mode = message.base_mode
                self.custom_mode = message.custom_mode
                self.system_status = message.system_status

        @vehicle.subscribe(mavlink.MAVLink_sys_status_message.msgname)
        def on_sys_status(message: mavlink.MAVLink_sys_status_message):
            if message.get_srcSystem() != 1:
                return
            with self.lock:
                self.onboard_control_sensors_present = message.onboard_control_sensors_present
                self.onboard_control_sensors_enabled = message.onboard_control_sensors_enabled
                self.onboard_control_sensors_health = message.onboard_control_sensors_health
                self.load = message.load
                self.voltage_battery = message.voltage_battery
                self.current_battery = message.current_battery
                self.battery_remaining = message.battery_remaining
                self.drop_rate_comm = message.drop_rate_comm
                self.errors_comm = message.errors_comm
                self.errors_count1 = message.errors_count1
                self.errors_count2 = message.errors_count2
                self.errors_count3 = message.errors_count3
                self.errors_count4 = message.errors_count4

        @vehicle.subscribe(mavlink.MAVLink_extended_sys_state_message.msgname)
        def update_landed_state(message: mavlink.MAVLink_extended_sys_state_message):
            if message.get_srcSystem() != 1:
                return
            with self.lock:
                self.landed_state = message.landed_state


    @property
    def armed(self):
        with self.lock:
            if self.base_mode is None:
                return False
            return bool(self.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def prearmed(self):
        with self.lock:
            if self.onboard_control_sensors_health is None:
                return False
            return bool(self.onboard_control_sensors_health & mavlink.MAV_SYS_STATUS_PREARM_CHECK)
        
    @property
    def mode_string(self):
        with self.lock:
            if self.custom_mode is None or self.mode_map_bynumber is None:
                return None
            return self.mode_map_bynumber.get(self.custom_mode)

    @property
    def flight_phase(self):
        """
        Map MAV_LANDED_STATE to FlightPhase (constants.FLIGHT_PHASE_*).
        """
        with self.lock:
            match self.landed_state:
                case mavlink.MAV_LANDED_STATE_ON_GROUND:
                    return constants.FLIGHT_PHASE_GROUNDED
                case mavlink.MAV_LANDED_STATE_IN_AIR | mavlink.MAV_LANDED_STATE_TAKEOFF | mavlink.MAV_LANDED_STATE_LANDING:
                    return constants.FLIGHT_PHASE_AIRBORNE
                case _:
                    return constants.FLIGHT_PHASE_UNKNOWN


