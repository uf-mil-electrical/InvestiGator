import time
import math
from multiprocessing import Queue
from queue import Empty

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

    def wait_for_armed(self, timeout=5):
        """
        Wait for vehicle to be armed.
        """
        #TODO: Implement timeout handling
        start_s = time.time()
        while not self.status.armed:
            if time.time() - start_s < timeout:
                # Exception
                print("Was not able to arm.")
            self.arm()
            time.sleep(0.1)

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

    def move_body_frd_position(self, forward_m, right_m, down_m=0.0, timeout_s=None):
        """
        Move relative to vehicle's FRD frame by Forward/Right. Optionally wait for target to be reached within timeout_s seconds.
        Will maintain current altitude by default.
        """
        ONLY_XYZ = ~mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE & ~mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE & ~mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        target_ned = self.convert_frd_to_ned(forward_m, right_m, down_m)

        self.mav.set_position_target_local_ned_send(
            time_boot_ms=int(time.time() * 1000),
            target_system=0,
            target_component=0,
            coordinate_frame=mavlink.MAV_FRAME_BODY_FRD,
            type_mask=ONLY_XYZ,
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

        if timeout_s is not None:
            start_s = time.time()
            while True:

                if self.target_ned_reached(target_ned):
                    break

                if time.time() - start_s < timeout_s:
                    self.move_body_frd_position(0,0,0)
                    # TODO: Return error code or exception
                    break

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
    
    def clear_detection_queue(self):
        """
        Clear detection queue by reading all items non-blocking.
        """
        while True:
            try:
                self.detection_queue.get_nowait()
            except Empty:
                break
    
    def center_on_marker(self, timeout_s=5, target_distance_m=2.0, rover=False):
        """
        Center on aruco marker based on detections from detection queue.
        """
        # Otherwise, abort the landing and return to launch (RTL)
        # 0. While detection are available within the timeout
        # 1. Get detection
        # 2. Send target position
        # 3. Wait for the position to be reached
        # 4. Descend by some amount
        # 5. Clear queue before re-looping. Save last detection in case we lose vision of marker.
        # 6. Refresh timeout, re-loop.
        # 7. If timeout event, send target back to last detection and height. New timeout.
        # 8. If second timeout: abort mission, raise exception, return to launch in caller.
        
        last_detection_ned_position = None
        start_s = time.time()

        while time.time() - start_s < timeout_s:
            try:
                detection = self.detection_queue.get(timeout=0.5)
                last_detection_ned_position = self.location.local_ned
            except Empty:
                if last_detection_ned_position is not None:
                    #self.move_global_ned_position(last_detection_ned_position, timeout_s=timeout_s-1)
                    last_detection_ned_position = None
                continue

            self.move_body_frd_position(forward_m=detection[0], right_m=detection[1])
            if detection[2] >= target_distance_m + 0.2:
                self.move_body_frd_position(forward_m=0, right_m=0, down_m=0.2)
            else: 
                break
            
            self.clear_detection_queue()
            start_s = time.time()
        

    def close(self):
        self.mav_connection.close()
        # TODO: Check that threads in mavconnection are closed correctly
        self.camera.stop()

if __name__ == "__main__":
    vehicle = VehicleManager("udp:127.0.0.1:14550")
    vehicle.close()
