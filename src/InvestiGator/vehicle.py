import time
import math
from multiprocessing import Queue
from queue import Empty
from enum import Enum
from threading import Event

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from mavconnection import MAVConnection
from vehicle_properties import Location, Status, MavFrameLocalNed, MavFrameGlobal
from camera import Camera, MarkerDetection
from constants import Radio, Robot


class VehicleManager:
    """
    Represents properties of a vehicle and handles communication with it.
    """

    def __init__(self, mav_connection: MAVConnection, baud=115200):
        self.mav_connection = mav_connection
        self.mode_map = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)

        self.detection_queue: Queue[MarkerDetection] = Queue()
        self.camera = Camera(self.detection_queue, preview=True)

        self.publish = self.mav_connection.publish
        self.publish_function = self.mav_connection.publish_function
        self.unpublish = self.mav_connection.unpublish
        self.subscribe = self.mav_connection.subscribe
        self.unsubscribe = self.mav_connection.sub_manager.unsubscribe

        self.mav = self.mav_connection.mav

        self.location = Location(self)
        self.status = Status(self)

    def send_command(self, command: int, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0, target_system=1, target_component=0, retries:int = 3, timeout_s: float = 2.0):
        """
        Send a MAVLink COMMAND_LONG message. Wait for COMMAND_ACK to be received. Retry up to retries times if not received within timeout_s seconds. 
        Return True if command acknowledged, False otherwise.
        """
        ack_event = Event()
        ack_result = None

        def on_ack(message: mavlink.MAVLink_command_ack_message):
            if message.command == command:
                nonlocal ack_result
                ack_result = message.result
                ack_event.set()

        self.subscribe(mavlink.MAVLink_command_ack_message.msgname)(on_ack)

        for attempt in range(retries):
            self.mav.command_long_send(
                target_system=target_system,
                target_component=target_component,
                command=command,
                confirmation=0,
                param1=param1,
                param2=param2,
                param3=param3,
                param4=param4,
                param5=param5,
                param6=param6,
                param7=param7
            )
            if ack_event.wait(timeout=timeout_s):
                self.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
                return ack_result == mavlink.MAV_RESULT_ACCEPTED
            
            # TODO: Log retry attempt
        self.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)

    def takeoff(self, alt_m, timeout_s=15, threshold_m=0.1):
        """
        Wait for vehicle to arm and take off to alt_m meters.
        """
        if not self.status.armed:
            # TODO: Log this as an error
            self.wait_for_armed()

        self.mav.command_long_send(
            target_system=1,
            target_component=1,
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

        if timeout_s is not None:
            start_s = time.time()
            while True: 
                print("Distance to takeoff altitude: ")
                print(self.location.relative_alt_m - alt_m)
                if time.time() - start_s > timeout_s:
                    return False
                if abs(self.location.relative_alt_m - alt_m) < threshold_m:
                    return True
                time.sleep(1)

    def land(self):
        """
        Land the vehicle.
        """
        print("Start Land Mode")
        self.mav.command_long_send(
            target_system=1,
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

    def wait_for_armed(self, timeout_s):
        """
        Wait for vehicle to be armed.
        """
        start_s = time.monotonic()
        while time.monotonic() - start_s < timeout_s:
            if self.status.armed:
                return True
            time.sleep(0.1)
            # TODO: Add abort event waiting here

        return False
    
    def wait_for_prearm(self, timeout_s):
        """
        Wait for vehicle to be prearmed.
        """
        start_s = time.monotonic()
        while time.monotonic() - start_s < timeout_s:
            if self.status.prearmed:
                return True
            time.sleep(0.1)
            # TODO: Add abort event waiting here

        return False

    def arm(self, timeout_s=30.0):
        """
        Arm vehicle.
        """
        start_s = time.monotonic()

        if not self.wait_for_prearm(timeout_s=timeout_s):
            #TODO: Log failed prearm
            return False
        
        if not self.send_command(command=mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1.0):
            #TODO: Log failed arm
            return False
        
        if not self.wait_for_armed(timeout_s= timeout_s - (time.monotonic() - start_s)):
            #TODO: Log failed arming
            return False
        
        return True


    def move_body_frd_position(self, forward_m, right_m, down_m=0.0, maintain_heading=True, timeout_s=None):
        """
        Move relative to vehicle's FRD frame by Forward/Right. Optionally wait for target to be reached within timeout_s seconds.
        Will maintain current altitude by default.
        """
        XYZ_POS = mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE & \
        mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE & \
        mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE

        XYZ_POS_YAW = XYZ_POS & mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

        typemask = XYZ_POS
        if not maintain_heading:
            typemask = XYZ_POS_YAW

        target_ned = self.convert_frd_to_ned(forward_m, right_m, down_m)

        self.mav.set_position_target_local_ned_send(
            time_boot_ms=0,
            target_system=1,
            target_component=0,
            coordinate_frame=mavlink.MAV_FRAME_BODY_NED,
            type_mask=typemask,
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

                if time.time() - start_s > timeout_s:
                    self.mav.set_position_target_local_ned_send(
                        time_boot_ms=0,
                        target_system=1,
                        target_component=0,
                        coordinate_frame=mavlink.MAV_FRAME_BODY_FRD,
                        type_mask=0b110111111000,
                        x = 0,
                        y = 0,
                        z = 0,
                        vx = 0,
                        vy = 0,
                        vz = 0,
                        afx = 0,
                        afy = 0,
                        afz = 0,
                        yaw = 0,
                        yaw_rate = 0
                    )
                    # TODO: Return error code or exception
                    break

            time.sleep(0.1)

    def move_global_gps(self, lat_int: int, lon_int: int, alt_m: int):
        """
        Move to the given GPS WGS84 coordinates.
        """
        #TODO: Test if heading is maintained or not during only lat/lon movement.
        # If heading is not maintained, use current heading or default to point northward (OR! Home heading?)
        self.mav.set_position_target_global_int_send(
            time_boot_ms=0,
            target_system=1,
            target_component=0,
            coordinate_frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            type_mask=0,
            lat_int=lat_int,
            lon_int=lon_int,
            alt=alt_m,
            vx=0,
            vy=0,
            vz=0,
            afx=0,
            afy=0,
            afz=0,
            yaw=0,
            yaw_rate=0
        )

    def set_mode(self, target_mode: str, timeout_s=None):
        """
        Change mode of the flight controller. See: https://ardupilot.org/copter/docs/parameters.html#fltmode1
        Common modes: 'GUIDED', 'LAND', 'CIRCLE'
        """
        # TODO: If timeout, return false
        if self.mode_map is None:
            return False
        
        target_mode_int = self.mode_map[target_mode]

        while not self.check_mode(target_mode=target_mode):
            self.mav.command_long_send(
                target_system=1,
                target_component=0,
                command=mavlink.MAV_CMD_DO_SET_MODE,
                confirmation=0,
                param1=mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
                param2=target_mode_int,
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0 
            )
            time.sleep(1)

    def check_mode(self, target_mode: str) -> bool:
        """
        Return if current mode is equal to target_mode. 
        """
        if self.status.custom_mode is not None and self.mode_map is not None:
            return self.status.custom_mode == self.mode_map[target_mode]
        return False

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
            
            threshold_m = 0.5
            forward_m = max(-threshold_m, min(detection.X_Offset_m, threshold_m))
            right_m = max(-threshold_m, min(detection.Y_Offset_m, threshold_m))

            if abs(detection.X_Offset_m) < threshold_m:
                forward_m = 0.0                

            if abs(detection.Y_Offset_m) < threshold_m:
                right_m = 0.0
            
            self.move_body_frd_position(forward_m, right_m)

            if abs(detection.X_Offset_m) < threshold_m and abs(detection.Y_Offset_m) < threshold_m:
                self.move_body_frd_position(0.0,0.0,0.0)

                if detection.Z_Offset_m >= target_distance_m:
                    self.move_body_frd_position(forward_m=0.0, right_m=0.0, down_m=0.5)
                else:
                    break
            print(forward_m, right_m, detection.Z_Offset_m)
            self.clear_detection_queue()
            time.sleep(0.1)
            start_s = time.time()
        
    def detect_multiple_objects(self):
        """
        For testing detection of several of the same objects close to each other. Report unique GPS coordinates of each object.
        """
        pass

    def search_for_detection(self, object_to_detect: str):
        """
        Perform a spiral search of the given object.
        """

        self.camera.switch_mode(object_to_detect)
        
        center_gps = self.location.global_frame

        print("Starting first Circle")

        self.circle(radius_m=0.1, center_gps=center_gps)
        start_s = time.time()
        while self.detection_queue.empty():
            if time.time() - start_s > 16:
                break
            time.sleep(0.5)

        self.set_mode("GUIDED")
        
        if not self.detection_queue.empty():
            return self.location.global_frame_relative
        
        print("Starting second Circle")

        self.circle(radius_m=0.5, center_gps=center_gps)
        start_s = time.time()
        while self.detection_queue.empty():
            if time.time() - start_s > 16:
                break
            time.sleep(0.5)

        self.set_mode("GUIDED")
        
        if not self.detection_queue.empty():
            return self.location.global_frame_relative
        
        return None

    def circle(self, radius_m, center_gps: MavFrameGlobal):
        """
        Perform a circle with given radius and center.
        """

        self.move_global_gps(center_gps.lattitude_int, center_gps.longitude_int, center_gps.altitude_m)
        time.sleep(4)

        self.mav.param_set_send(
            target_system=1,
            target_component=0,
            param_id="CIRCLE_RADIUS".encode('utf-8'),
            param_value=radius_m * 1E2, # cm
            param_type=10
        )
        
        self.set_mode("CIRCLE")

    def close(self):
        #self.mav_connection.close()
        # TODO: Check that threads in mavconnection are closed correctly
        self.camera.stop()

# if __name__ == "__main__":
    
    # vehicle = VehicleManager("udp:127.0.0.1:14550", source_system=System.INVESTIGATOR)
    # vehicle.camera.switch_mode("UAV Recovery")
    # vehicle.set_mode(target_mode="GUIDED")
    # vehicle.wait_for_armed()
    # vehicle.takeoff(alt_m=10)

    # vehicle.move_body_frd_position(forward_m=5, right_m=4, down_m=-10, timeout_s=20)
    # print("Positioned for search.")
    # detection_gps = vehicle.search_for_detection("UAV Recovery")
    # print("Search Complete")
    # if detection_gps is not None:
    #     vehicle.center_on_marker(timeout_s=100, target_distance_m=0.25)

    # vehicle.set_mode("LAND")
    # print("DONE")

    # vehicle.close()
