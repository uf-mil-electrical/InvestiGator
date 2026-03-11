import time
import math
from multiprocessing import Queue
from queue import Empty
from enum import Enum
from threading import Event

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from .mavconnection import MAVConnection
from .vehicle_properties import Location, MavFrameGlobalRel, MavFrameLocalOffsetNED, Status, MavFrameLocalNed, MavFrameGlobal
from .camera import Camera, MarkerDetection


class VehicleManager:
    """
    Represents properties of a vehicle and handles communication with it.
    """

    def __init__(self, mav_connection: MAVConnection, baud=115200):
        self.mav_connection = mav_connection

        self.cancel_mission_event = Event()

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

        self.configure_messages()
        self.wait_for_condition(self.properties_populated)


    def configure_messages(self):
        """
        Send requests for messages from autopilot to ensure required data is being sent.
        """
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, param2=10000)
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, param2=10000)
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_SYS_STATUS, param2=10000)
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_ATTITUDE, param2=10000)


    def wait_for_condition(self, condition_function, timeout_s=30.0, interval_s=0.1):
        """
        Wait for condition function to be True within timeout_s seconds, checking every interval_s seconds.
        If mission_cancel_event is set, returns False.
        Return True if condition met, False if timeout reached.
        """
        start_s = time.monotonic()
        while time.monotonic() - start_s < timeout_s:
            if self.cancel_mission_event.wait(timeout=interval_s):
                return False
            if condition_function():
                return True
        return False

    
    def properties_populated(self):
        """
        Return True if vehicle properties have been populated. False otherwise.
        """
        if self.location.lat_int is not None and self.location.x_north_m is not None and self.status.onboard_control_sensors_health is not None and self.location.roll_rad is not None:
                return True
        return False
    

    def wait_for_messages(self, timeout_s = 30.0):
        """
        Wait to confirm that required messages are being received from autopilot before beginning missions.
        """
        start_s = time.monotonic()
        while time.monotonic() - start_s < timeout_s:
            if self.location.lat_int is not None and self.location.x_north_m is not None and self.status.onboard_control_sensors_health is not None and self.location.roll_rad is not None:
                return True
            time.sleep(0.1)
        
        return False


    def send_command(self, command: int, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0, target_system=1, target_component=0, retries:int = 3, retry_timeout_s: float=1.0):
        """
        Send a MAVLink COMMAND_LONG message. Wait for COMMAND_ACK to be received.
        Retry up to retries times if not received within retry_timeout_s seconds. Maximum wait is retries * retry_timeout_s seconds.
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
            ack_event.clear()
            ack_result = None

            self.mav.command_long_send(
                target_system=target_system,
                target_component=target_component,
                command=command,
                confirmation=attempt,
                param1=param1,
                param2=param2,
                param3=param3,
                param4=param4,
                param5=param5,
                param6=param6,
                param7=param7
            )
            
            if ack_event.wait(timeout=retry_timeout_s):
                self.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
                return ack_result == mavlink.MAV_RESULT_ACCEPTED
            # TODO: Refactor for abort event
            # TODO: Log retry attempt
        self.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
        return False

    def takeoff(self, alt_m, timeout_s=30, threshold_m=0.5):
        """
        Wait for vehicle to arm and take off to alt_m meters.
        """
        start_s = time.monotonic()

        if not self.status.armed:
            # TODO: Log waiting for arm
            if not self.arm(timeout_s=timeout_s):
                return False

        if not self.send_command(command=mavlink.MAV_CMD_NAV_TAKEOFF, param7=alt_m):
            # TODO: Log failed takeoff command ack
            return False
        
        remaining_s = timeout_s - (time.monotonic() - start_s)
        if not self.wait_for_condition(lambda: self.altitude_reached(alt_m), timeout_s=remaining_s):
            # TODO: Log failed altitude, altitude reached, and aborting to RTL
            self.send_command(command=mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            return False
        
        return True
    

    def altitude_reached(self, alt_m, threshold_m=0.5):
        """
        Return True if altitude alt_m is within threshold_m meters.
        """
        altitude_rel = self.location.global_frame_relative.altitude_rel_m
        if altitude_rel is not None and abs(altitude_rel - alt_m) <= threshold_m:
            return True
        return False


    def land(self, timeout_s=30.0):
        """
        Land the vehicle.
        """
        start_s = time.monotonic()

        if not self.send_command(command=mavlink.MAV_CMD_NAV_LAND):
            # TODO: Log
            return False
        
        if not self.wait_for_disarmed(timeout_s = timeout_s - (time.monotonic() - start_s)):
            altitude_rel = self.location.global_frame_relative.altitude_rel_m

            if altitude_rel is not None and altitude_rel > 0.5:
                # TODO: Log failed landing, abort to RTL
                self.send_command(command=mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
                return False
            
            return True

        return True
    
    def wait_for_disarmed(self, timeout_s):
        """
        Wait for vehicle to disarm.
        """
        start_s = time.monotonic()
        while time.monotonic() - start_s < timeout_s:
            if not self.status.armed:
                return True
            # TODO: Add abort event waiting here
            time.sleep(0.1)
        
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
        
        remaining_s = timeout_s - (time.monotonic() - start_s)
        if not self.wait_for_condition(lambda: self.status.armed, timeout_s=remaining_s):
            #TODO: Log failed arming
            return False
        return True


    def move_body_frd_position(self, forward_m, right_m, down_m=0.0, maintain_heading=True, timeout_s=30.0):
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

        target_ned = self.convert_frd_target_to_local_ned_target(forward_m, right_m, down_m)
        if target_ned is None:
            # TODO: Log
            return False

        self.mav.set_position_target_local_ned_send(
            time_boot_ms=0,
            target_system=1,
            target_component=0,
            coordinate_frame=mavlink.MAV_FRAME_BODY_OFFSET_NED,
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

        if not self.wait_for_ned_position(target_ned, timeout_s=timeout_s):
            # Stop movement
            self.mav.set_position_target_local_ned_send(
                time_boot_ms=0,
                target_system=1,
                target_component=0,
                coordinate_frame=mavlink.MAV_FRAME_BODY_OFFSET_NED,
                type_mask=typemask,
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

            return False

        return True


    def wait_for_ned_position(self, target_ned: MavFrameLocalNed, timeout_s=30.0, threshold_m=0.5):
        """
        Wait for vehicle to reach target NED position within threshold_m meters within timeout_s seconds.
        Returns True if position reached, False otherwise.
        """
        start_s = time.monotonic()
        while time.monotonic() - start_s < timeout_s:
            if self.target_ned_reached(target_ned, threshold_m=threshold_m):
                return True
            time.sleep(0.1)
            # TODO: Add abort event
        return False


    def move_global_gps_relative_alt(self, lat_int: int, lon_int: int, alt_m: int):
        """
        Move to the given GPS WGS84 coordinates. Altitude is relative to home position. Maintain current heading.
        """
        
        typemask = mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE & \
        mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE & \
        mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE

        target_global_rel_alt = MavFrameGlobalRel(lat_int, lon_int, alt_m)

        self.mav.set_position_target_global_int_send(
            time_boot_ms=0,
            target_system=1,
            target_component=0,
            coordinate_frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            type_mask=typemask,
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

    def set_mode(self, target_mode: str | int, timeout_s=5.0):
        """
        Change mode of the flight controller. See: https://ardupilot.org/copter/docs/parameters.html#fltmode1
        Common modes: 'GUIDED', 'LAND', 'CIRCLE'
        """
        if self.status.mode_map_byname is None or self.status.mode_map_bynumber is None:
            # TODO: Log None maps
            return False
        
        if isinstance(target_mode, str):
            target_mode_int = self.status.mode_map_byname.get(target_mode)
            if target_mode_int is None:
                # TODO: Log unsupported mode
                return False
        else:
            target_mode_int = target_mode
            if target_mode_int not in self.status.mode_map_bynumber:
                # TODO: Log unsupported mode
                return False

        if not self.send_command(command=mavlink.MAV_CMD_DO_SET_MODE, param1=mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2=target_mode_int):
            # TODO: Log failure
            return False
        
        if not self.wait_for_mode(target_mode, timeout_s=timeout_s):
            # TODO: Log failure
            return False
        
        return True

    def wait_for_mode(self, target_mode: str | int, timeout_s):
        """
        Wait for current mode to be target_mode. Return True if target_mode is detected within timeout_s seconds, False otherwise.
        """
        start_s = time.monotonic()
        while time.monotonic() - start_s < timeout_s:
            if self.check_mode(target_mode):
                return True
            # TODO: Add abort event waiting here
            time.sleep(0.1)

        return False

    def check_mode(self, target_mode: str | int) -> bool:
        """
        Return if current mode is equal to target_mode. 
        """
        if isinstance(target_mode, int):
            return self.status.custom_mode == target_mode
        
        return self.status.mode_string == target_mode

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
    
    def convert_frd_target_to_local_ned_target(self, forward_m, right_m, down_m):
        """
        Convert FRD target frame to Local NED target frame, with origin fixed relative to earth.
        """
        yaw_rad = self.location.attitude.yaw_rad
        current_local_ned = self.location.local_ned

        if yaw_rad is None or current_local_ned is None:
            return None

        dx_north_m = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        dy_east_m = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
        dz_down_m = down_m

        target_x_north_m = current_local_ned.x_north_m + dx_north_m
        target_y_east_m = current_local_ned.y_east_m + dy_east_m
        target_z_down_m = current_local_ned.z_down_m + dz_down_m

        return MavFrameLocalNed(target_x_north_m, target_y_east_m, target_z_down_m)
    
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

        self.move_global_gps_relative_alt(center_gps.lattitude_int, center_gps.longitude_int, center_gps.altitude_m)
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