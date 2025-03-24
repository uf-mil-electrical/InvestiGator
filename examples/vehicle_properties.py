from pymavlink.dialects.v20 import ardupilotmega as mavlink
#from vehicle import VehicleManager
from dataclasses import dataclass

@dataclass
class MAV_FRAME_GLOBAL:
    """
    Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL).
    """
    latitude_deg:  float
    longitude_deg:  float
    altitude_m:   float

@dataclass
class MAV_FRAME_LOCAL_NED:
    """
    NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
    """
    x_north_m:    float
    y_east_m:     float
    z_down_m:     float

@dataclass
class MAV_FRAME_GLOBAL_RELATIVE_ALT:
    """
    Global (WGS84) coordinate frame + altitude relative to the home position.
    """
    latitude_deg: float
    longitude_deg: float
    altitude_relative_m: float

@dataclass
class MAV_FRAME_LOCAL_ENU:
    """
    ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
    """
    x_east_m:     float
    y_north_m:    float
    z_up_m:       float

@dataclass
class MAV_FRAME_LOCAL_OFFSET_NED:
    """
    NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
    """
    x_north_m:    float
    y_east_m:     float
    z_down_m:     float

@dataclass
class MAV_FRAME_BODY_FRD:
    """
    FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle.
    """
    x_forward_m:  float
    y_right_m:    float
    z_down_m:     float

@dataclass
class MAV_FRAME_LOCAL_FRD:
    """
    FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
    """
    x_forward_m:  float
    y_right_m:    float
    z_down_m:     float

@dataclass
class MAV_FRAME_LOCAL_FLU:
    """
    FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
    """
    x_forward_m:  float
    y_left_m:     float
    z_up_m:       float

class Location:
    """
    Represents location of the vehicle and provides methods to return location wrapped in different location types.
    """
    def __init__(self, vehicle):
        self.lat_deg = None
        self.lon_deg = None
        self.alt_m = None
        self.relative_alt_m = None

        self.x_north_m = None
        self.y_east_m = None
        self.z_down_m = None

        @vehicle.subscribe(mavlink.mavlink_map[mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT].msgname)
        def subscription_update(message_type, message: mavlink.MAVLink_global_position_int_message):
            self.lat_deg = message.lat / 1E7
            self.lon_deg = message.lon / 1E7
            self.alt_m = message.alt / 1E3                     # Given in mm
            self.relative_alt_m = message.relative_alt / 1E3   # Given in mm

        @vehicle.subscribe(mavlink.MAVLink_local_position_ned_message.msgname)
        def subscription_update(message_type, message: mavlink.MAVLink_local_position_ned_message):
            self.x_north_m = message.x
            self.y_east_m = message.y
            self.z_down_m = message.z   # Negative altitude

    @property
    def global_frame(self):
        """
        Returns location as a MAV_FRAME_GLOBAL frame. Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL).
        """
        return MAV_FRAME_GLOBAL(self.lat_deg, self.lon_deg, self.alt_m)
    
    @property
    def local_ned(self):
        """"
        NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
        """
        return MAV_FRAME_LOCAL_NED(self.x_north_m, self.y_east_m, self.z_down_m)

    @property
    def global_relative_alt(self):
        pass

    @property
    def local_offset_ned(self):
        pass

    @property
    def body_frd(self):
        pass

    @property
    def local_frd(self):
        pass
