import time
import math
from multiprocessing import Queue

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from mavconnection import MAVConnection
from vehicle_properties import Location, Status, MavFrameLocalNed
from camera import Camera

radio = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0"
simulation = 'udp:127.0.0.1:14550'


class VehicleManager:
    """
    Represents properties of a vehicle and handles communication with it.
    """

    def __init__(self, address, baud=115200):
        self.mav_connection = MAVConnection(address, baud)
        self.mode_map = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)

        self.detection_queue: Queue = Queue()
        self.camera = Camera(self.detection_queue)

        self.publish = self.mav_connection.publish
        self.publish_function = self.mav_connection.publish_function
        self.unpublish = self.mav_connection.unpublish
        self.subscribe = self.mav_connection.subscribe

        self.mav = self.mav_connection.mav_connection.mav

        self.location = Location(self)
        self.status = Status(self)

        @self.publish('HEARTBEAT', 1)
        def publish_heartbeat():
            self.mav.heartbeat_send(
                type=mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=0
            )

    def takeoff(self, alt_m):
        """
        Wait for vehicle to arm and take off to alt_m meters.
        """
        if not self.status.armed:
            # TODO: Log this as an error and raise an exception. Remove this auto arming.
            self.wait_for_armed()

        self.mav.command_long_send(
            target_system=0,
            target_component=0,
            command=mavlink.MAV_CMD_NAV_TAKEOFF,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=alt_m
        )

    def land(self):
        """
        Land the vehicle.
        """
        self.mav.command_long_send(
            target_system=0,
            target_component=0,
            command=mavlink.MAV_CMD_NAV_LAND,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0 
        )

    def wait_for_armed(self):
        """
        Wait for vehicle to be armed.
        """
        #TODO: Implement timeout 
        while not self.status.armed:
            self.arm()
            time.sleep(0.5)

    def arm(self):
        """
        Arm vehicle.
        """
        self.mav.command_long_send(
            target_system=0,
            target_component=0,
            command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=1.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0 
        )

    def move_body_frd_position_and_wait(self, forward_m, right_m, down_m=0, timeout=5):
        """
        Move relative to vehicle's FRD frame by Forward/Right. Wait for target to be reached.
        Will maintain current altitude by default.
        """
        ONLY_XY = ~mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE & ~mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE & ~mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        target_ned = self.convert_frd_to_ned(forward_m, right_m, down_m)

        #TODO: Implement timeout handling
        start_s = time.time()
        while time.time() - start_s < timeout:

            self.mav.set_position_target_local_ned_send(
                time_boot_ms=int(time.time() * 1000),
                target_system=0,
                target_component=0,
                coordinate_frame=mavlink.MAV_FRAME_BODY_FRD,
                type_mask=ONLY_XY,
                x = forward_m,
                y = right_m,
                z = down_m,
                vx = 0,
                vy = 0,
                vz = 0,
                afx = 0,
                afy = 0,
                afz = 0,
                yaw = 0,
                yaw_rate = 0
            )

            if not self.target_ned_reached(target_ned):
                time.sleep(0.1)


    def move_global_gps(self):
        pass

    def set_mode(self, mode):
        pass

    def target_ned_reached(self, target_ned: MavFrameLocalNed, threshold_m=0.1) -> bool:
        """
        Check if target NED position has been reached within threshold_m meters.
        """
        current_ned = self.location.local_ned

        dn = current_ned.x_north_m - target_ned.x_north_m
        de = current_ned.y_east_m - target_ned.y_east_m
        dd = current_ned.z_down_m - target_ned.z_down_m

        distance_m = math.sqrt(dn*dn + de*de + dd*dd)

        return distance_m < threshold_m
    
    def convert_frd_to_ned(self, forward_m, right_m, down_m):
        """
        Convert FRD target frame to NED target frame.
        """
        yaw_rad = self.location.attitude.yaw_rad

        x_north_m = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        y_east_m = forward_m * math.sin(yaw_rad) + right_m * math.sin(yaw_rad)
        z_down_m = down_m

        return MavFrameLocalNed(x_north_m, y_east_m, z_down_m)
    
    def center_on_marker(self):
        """
        Center on aruco marker based on detections from detection queue.
        """
        #TODO: Timeout if no detections made for a given time. Move back to last marker detection and see if it is redetected
        # Otherwise, abort the landing and return to launch (RTL)

        pass

    def close(self):
        self.mav_connection.close()
        # TODO: Check that threads in mavconnection are closed correctly
        # TODO: Close camera here


if __name__ == "__main__":
    vehicle = VehicleManager("udp:127.0.0.1:14550")
    vehicle.close()
