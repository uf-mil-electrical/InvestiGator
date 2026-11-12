import time
import math
from multiprocessing import Queue
from queue import Empty, SimpleQueue
from enum import Enum
from threading import Event, Lock, Thread

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from .mavconnection import MAVConnection
from .vehicle_properties import Location, MavFrameGlobalRel, MavFrameLocalOffsetNED, Status, MavFrameLocalNed, MavFrameGlobal
from .geofence import Geofence, LatLng, MIN_POLYGON_VERTICES, MAX_POLYGON_VERTICES, inset_polygon
from .camera import Camera, MarkerDetection
from . import constants

M_PER_DEG_LAT = 111_320.0


class VehicleManager:
    """
    Represents properties of a vehicle and handles communication with it.
    """

    def __init__(self, mav_connection: MAVConnection, baud=115200, sim=False):
        self.mav_connection = mav_connection

        self.cancel_mission_event = Event()
        self.uncontrolled_event = Event()
        self.uncontrolled_event.set()

        #self.lastknown_state = constants.MIL_STATE_CONNECTING
        #self.lastknown_pos = self.location.global_frame_relative #should be in lat, lon, rel alt order

        self.run_started = False

        # Goal of the current EM_RESPOND/EM_RETURN thread, None when no incident. Threads compare it by identity.
        self.incident_lock = Lock()
        self.incident_goal: MavFrameGlobalRel | None = None
        # Set while a keep-out thread flies the drone out of a new circle. Emergency threads hold their goal until it clears.
        self.keep_out_leaving = Event()

        # Course fence from the OCS. fence_lock serializes fence writes to the Cube.
        self.fence_inbox = SimpleQueue()
        self.fence_live = Event()
        self.fence_lock = Lock()
        # Inclusion polygon read back from the Cube after the last upload, and the copy each OCS is downloading, by (sysid, compid).
        self.cube_fence: list[LatLng] = []
        self.fence_downloads: dict[tuple[int, int], tuple[list[LatLng], float]] = {}
        # FENCE_ALT_MAX as read back from the Cube (m above home).
        self.cube_ceiling_m: float | None = None

        self.detection_queue: Queue[MarkerDetection] = Queue()
        self.camera = Camera(self.detection_queue, preview=False)

        self.publish = self.mav_connection.publish
        self.publish_function = self.mav_connection.publish_function
        self.unpublish = self.mav_connection.unpublish
        self.subscribe = self.mav_connection.subscribe
        self.unsubscribe = self.mav_connection.sub_manager.unsubscribe

        self.wait_for_condition = self.mav_connection.wait_for_condition_with_cancel(self.cancel_mission_event)

        self.mav = self.mav_connection.mav

        self.location = Location(self)
        self.status = Status(self)
        self.geofence = Geofence()
        self.sim = sim
        self.fence_alt_max_m = (constants.FENCE_ALT_MAX_SIM_M if sim else constants.FENCE_ALT_MAX_M) - constants.MAX_ALT_MARGIN_M
        # Last value from the OCS, to spot repeats.
        self.max_altitude_hae_m: float | None = None
        self.last_target_checked = None


        self.lastknown_state = constants.MIL_STATE_STANDBY
        self.lastknown_pos = self.location.global_frame_relative #should be in lat, lon, rel alt order

        self.magnet_state = 0

        self.intended_rtl_land = False

        self.MANUAL_MODES = {"LOITER", "ALT_HOLD"}
        self.LANDING_MODES = {"LAND", "RTL"}

        self.configure_messages()
        self.wait_for_condition(self.properties_populated)

        # Before the fence receiver starts, so only a fence from an earlier session can be cleared. Still CONNECTING
        # meanwhile: the OCS waits for the state to leave CONNECTING before it uploads (OCS_gaps.md gap 1).
        self.wait_for_condition(lambda: self.status.base_mode is not None, timeout_s=5.0)
        self.clear_stale_fence()

        self.mav_connection.system_status = constants.MIL_STATE_INITIAL_OVERRIDE

        self.subscribe(mavlink.MAVLink_heartbeat_message.msgname)(self.clear_uncontrolled_event)
        self.subscribe(mavlink.MAVLink_heartbeat_message.msgname)(self.check_intended_mode)
        self.subscribe(mavlink.MAVLink_command_long_message.msgname)(self.handle_system_command)
        self.subscribe(mavlink.MAVLink_mission_count_message.msgname)(self.on_fence_upload)
        self.subscribe(mavlink.MAVLink_mission_item_int_message.msgname)(self.on_fence_upload)
        self.subscribe(mavlink.MAVLink_mission_request_list_message.msgname)(self.on_fence_download)
        self.subscribe(mavlink.MAVLink_mission_request_int_message.msgname)(self.on_fence_download)
        self.subscribe(mavlink.MAVLink_mission_request_message.msgname)(self.on_fence_download)

        Thread(name="Fence receiver Thread", target=self.receive_fences, daemon=True).start()


    def handle_system_command(self, message: mavlink.MAVLink_command_long_message):
        """
        Handle system commands from ground control through MIL_SYSTEM_CMD.
        param1 = 0: Ping companion computer.
        param1 = 1: Set mode to GUIDED.
        param1 = 2: Set uncontrolled
        param1 = 3: Set cancel mission
        param1 = 4: Respond to emergency (AssistanceRequest)
        param1 = 5: Return to mission (ReadinessConfirm)
        param1 = 6: Toggle Magnet
        param1 = 7: Run started
        param1 = 8: Add keep-out circle (KeepOutZone)
        param1 = 9: Clear keep-out circles (AllClear)
        param1 = 11: Set the maximum altitude (param2, m HAE)
        """
        if message.command == constants.MIL_SYSTEM_CMD:
            # A NaN or inf would make int() raise and stop this dispatcher thread.
            if not all(math.isfinite(param) for param in (message.param1, message.param2, message.param3, message.param4)):
                print("Ignoring a system command with a non-finite parameter.")
                return

            code = int(message.param1)
            # The OCS sends ReadinessConfirm as param1 0 with confirmation 5.
            if code == constants.MIL_SYSTEM_PING and message.confirmation == constants.MIL_SYSTEM_EM_RETURN:
                code = constants.MIL_SYSTEM_EM_RETURN
            result = mavlink.MAV_RESULT_ACCEPTED

            match code:
                case constants.MIL_SYSTEM_PING:
                    self.mav_connection.mav.statustext_send(severity=mavlink.MAV_SEVERITY_WARNING, text="Pong".encode())
                    print("Ping received from ground control.")

                case constants.MIL_SYSTEM_GUIDED:
                    if self.uncontrolled_event.is_set():
                        # Run command in a separate thread to not block subscription manager.
                        print("Received system command: GUIDED. Setting mode to GUIDED.")
                        Thread(target=self.set_mode, args=("GUIDED",), daemon=True).start()

                case constants.MIL_SYSTEM_OVERRIDE:
                    print("Received system command: Set uncontrolled.")
                    if self.mav_connection.system_status == constants.MIL_STATE_MISSION:
                        self.cancel_mission_event.set()
                    self.uncontrolled_event.set()
                    self.mav_connection.system_status = constants.MIL_STATE_OVERRIDE
                    Thread(target=self.set_mode, args=("RTL",), daemon=True).start()

                case constants.MIL_SYSTEM_CANCEL:
                    print("Received system command: Cancel mission.")
                    if self.mav_connection.system_status == constants.MIL_STATE_MISSION:
                        self.cancel_mission_event.set()

                case constants.MIL_SYSTEM_EM_RESPOND:
                    print("Recieved system command: Emergency Respond.")
                    latitude, longitude = message.param2, message.param3
                    # The OCS sends degrees, the ground UI degrees * 1E7.
                    if abs(latitude) <= 90 and abs(longitude) <= 180:
                        latitude, longitude = latitude * 1E7, longitude * 1E7
                    # Capped here too, so arrival is judged against the height actually flown.
                    target = MavFrameGlobalRel(int(round(latitude)), int(round(longitude)), int(self.cap_altitude(message.param4)))
                    # Now rather than after a takeoff from the pad, when em_fly_to first sends it.
                    self.notify_target_outside_fence(target.lattitude_int / 1E7, target.longitude_int / 1E7)

                    if not self.status.armed or self.uncontrolled_event.is_set():
                        print("Emergency respond denied: the companion computer isn't flying the drone.")
                        result = mavlink.MAV_RESULT_DENIED
                    elif target == self.incident_goal:
                        print("Repeat of the current emergency.")
                    else:
                        if self.mav_connection.system_status == constants.MIL_STATE_MISSION:
                            self.mav_connection_system_status = constants.MIL_STATE_EMERGENCY
                            self.cancel_mission_event.set()
                        with self.incident_lock:
                            # A newer emergency keeps the first return point. From the pad, that's above it at the response height.
                            if self.incident_goal is None:
                                self.lastknown_pos = self.location.global_frame_relative
                                if self.status.flight_phase == constants.FLIGHT_PHASE_GROUNDED:
                                    self.lastknown_pos = self.lastknown_pos._replace(altitude_rel_m=target.altitude_rel_m)
                            self.incident_goal = target
                            if self.mav_connection.system_status == constants.MIL_STATE_AT_INCIDENT:
                                self.mav_connection.system_status = constants.MIL_STATE_STANDBY
                        self.mav_connection_system_status = constants.MIL_STATE_EMERGENCY
                        Thread(target=self.em_fly_to, args=(target, True,), daemon=True).start()
                        print("Ready for further instructions.")

                case constants.MIL_SYSTEM_EM_RETURN:
                    print("Recieved all clear, returning to normal operations.")
                    # A copy, so it's a new goal by identity even if the return point is unchanged.
                    goal = MavFrameGlobalRel(*self.lastknown_pos)
                    with self.incident_lock:
                        returning = self.incident_goal is not None
                        if returning:
                            self.incident_goal = goal
                            self.mav_connection.custom_mode = constants.RX_TASK_NONE
                            if self.mav_connection.system_status == constants.MIL_STATE_AT_INCIDENT:
                                self.mav_connection.system_status = constants.MIL_STATE_STANDBY
                    if returning:
                        Thread(target=self.em_fly_to, args=(goal, False,), daemon=True).start()
                    else:
                        # Before any emergency, lastknown_pos is the boot position at ground level.
                        print("No emergency to return from.")
                    #self.mav_connection_system_status = self.lastknown_state

                case constants.MIL_SYSTEM_MAGNET:
                    if self.magnet_state == 1:
                        Thread(target=self.trigger_magnet, args=(0,), daemon=True).start()
                        self.magnet_state = 0
                    elif self.magnet_state == 0:
                        Thread(target=self.trigger_magnet, args=(1,), daemon=True).start()
                        self.magnet_state = 1

                case constants.MIL_SYSTEM_RUN_START:
                    print("Received run start.")
                    self.run_started = True

                case constants.MIL_SYSTEM_KEEP_OUT:
                    if not message.param4 > 0:
                        print(f"Keep-out circle denied: radius {message.param4:g} m.")
                        result = mavlink.MAV_RESULT_DENIED
                    elif self.geofence.add_keep_out(LatLng(float(message.param2), float(message.param3)), float(message.param4)):
                        print(f"Keep-out circle: {message.param2:.7f}, {message.param3:.7f}, radius {message.param4:g} m.")
                        Thread(target=self.update_keep_outs, daemon=True).start()

                case constants.MIL_SYSTEM_ALL_CLEAR:
                    print("Received AllClear: lifting keep-out circles.")
                    if self.geofence.clear_keep_outs():
                        Thread(target=self.update_keep_outs, daemon=True).start()

                case constants.MIL_SYSTEM_MAX_ALTITUDE:
                    ceiling_amsl_m = self.amsl_ceiling_from_hae(message.param2)
                    if message.param2 == self.max_altitude_hae_m:
                        print("Repeat of the maximum altitude.")
                    elif ceiling_amsl_m is None or self.status.flight_phase != constants.FLIGHT_PHASE_GROUNDED:
                        if ceiling_amsl_m is None:
                            print("AMSL_M IS NONE")
                        else:
                            print(f"AMSL_M: {ceiling_amsl_m}")
                        print(f"self.status.flight_phase: {self.status.flight_phase}")
                        print(f"Maximum altitude {message.param2:g} m HAE denied.")
                        result = mavlink.MAV_RESULT_DENIED
                    else:
                        self.max_altitude_hae_m = message.param2
                        Thread(target=self.set_fence_ceiling, args=(ceiling_amsl_m,), daemon=True).start()


            if code in constants.MIL_SYSTEM_CMDS:
                print("Sending command ack")
                if code in (constants.MIL_SYSTEM_EM_RESPOND, constants.MIL_SYSTEM_KEEP_OUT, constants.MIL_SYSTEM_ALL_CLEAR):
                    print(f"Sending IncidentAck to OCS for {constants.MIL_SYSTEM_CMDS[code]}: {mavlink.enums['MAV_RESULT'][result].name}.")
                # result_param2 names the sub-code acked, so the OCS can tell its IncidentAcks apart.
                self.mav.command_ack_send(command=constants.MIL_SYSTEM_CMD, result=result, progress=0, result_param2=code,
                                          target_system=message.get_srcSystem(), target_component=message.get_srcComponent())

    def em_fly_to(self, goal: MavFrameGlobalRel, responding: bool):
        """
        Fly to goal and hold there until another EM_RESPOND or EM_RETURN replaces it. Runs in its own thread.
        Responding: take off if landed, then report on station (MIL_STATE_AT_INCIDENT) or a target it can't reach.
        Returning: the incident ends on arrival, or on timeout.
        """
        sent = on_station = warned = False
        deadline_s = settled_s = None

        while True:
            time.sleep(0.1)
            in_control = self.in_control()
            landed = self.status.flight_phase == constants.FLIGHT_PHASE_GROUNDED

            with self.incident_lock:
                if self.incident_goal is not goal:
                    return
                if not self.status.armed:
                    print("Disarmed during an emergency: standing down.")
                    self.incident_goal = None
                    self.mav_connection.custom_mode = constants.RX_TASK_NONE
                    if self.mav_connection.system_status == constants.MIL_STATE_AT_INCIDENT:
                        self.mav_connection.system_status = constants.MIL_STATE_STANDBY
                    return
                if responding and in_control:
                    # Set here rather than on receipt: main.py resets it when a cancelled mission returns.
                    self.mav_connection.custom_mode = constants.RX_TASK_DYNAMIC_INCIDENT
                holding = not in_control or self.keep_out_leaving.is_set() or (responding and landed)
                if not holding and not sent:
                    # Sent under the lock, so a superseded thread can't send after its replacement. Arrival is checked below.
                    self.move_global_gps_relative_alt(goal.lattitude_int, goal.longitude_int, goal.altitude_rel_m, timeout_s=0)
                    sent, settled_s = True, None
                    deadline_s = time.monotonic() + constants.INCIDENT_TRANSIT_BASE_S + constants.INCIDENT_TRANSIT_S_PER_M * self.distance_to(goal)[0]

            if holding:
                sent = False
                if responding and in_control and landed:
                    print(f"Taking off to {goal.altitude_rel_m:g} m for the emergency.")
                    if not self.takeoff(goal.altitude_rel_m):
                        time.sleep(1)
                continue
            if on_station:
                continue

            # Looser than target_global_reached's 0.1 m, which is too tight for GPS.
            horizontal_m, vertical_m = self.distance_to(goal)
            if horizontal_m < constants.INCIDENT_ARRIVAL_M and vertical_m < constants.INCIDENT_ARRIVAL_M:
                if settled_s is None:
                    settled_s = time.monotonic()
                if time.monotonic() - settled_s >= constants.INCIDENT_SETTLE_S:
                    with self.incident_lock:
                        if self.incident_goal is not goal:
                            return
                        if not responding:
                            print("Back from the emergency.")
                            self.incident_goal = None
                            return
                        print("On station at the emergency.")
                        on_station = True
                        if self.mav_connection.system_status == constants.MIL_STATE_STANDBY:
                            self.mav_connection.system_status = constants.MIL_STATE_AT_INCIDENT
                continue
            settled_s = None

            if time.monotonic() > deadline_s:
                with self.incident_lock:
                    if self.incident_goal is not goal:
                        return
                    if not responding:
                        print("Return timed out: holding here.")
                        self.incident_goal = None
                        return
                    if not warned:
                        # GUIDED silently ignores a target outside the fence; keep waiting in case it was only slow.
                        print("Emergency transit timed out: target outside the fence?")
                        self.mav.statustext_send(mavlink.MAV_SEVERITY_WARNING, constants.STATUS_INCIDENT_TARGET_OUTSIDE_FENCE.encode())
                        warned = True

    def in_control(self) -> bool:
        """
        True while the companion flies the drone: no pilot override, and no scripted mission running or being cancelled.
        """
        return not (self.uncontrolled_event.is_set() or self.cancel_mission_event.is_set()
                    or self.mav_connection.system_status == constants.MIL_STATE_MISSION)

    def incident_active(self) -> bool:
        with self.incident_lock:
            return self.incident_goal is not None

    def distance_to(self, target: MavFrameGlobalRel) -> tuple[float, float]:
        """
        Horizontal and vertical distance in meters from the drone to target (altitude relative to home).
        """
        current = self.location.global_frame_relative
        if current.lattitude_int is None or current.altitude_rel_m is None:
            return math.inf, math.inf
        dn = (target.lattitude_int - current.lattitude_int) / 1E7 * M_PER_DEG_LAT
        de = (target.longitude_int - current.longitude_int) / 1E7 * M_PER_DEG_LAT * math.cos(math.radians(current.lattitude_int / 1E7))
        return math.hypot(dn, de), abs(target.altitude_rel_m - current.altitude_rel_m)

    def offset_from(self, center: LatLng) -> tuple[float, float]:
        """
        The drone's position in meters north and east of center.
        """
        current = self.location.global_frame_relative
        north = (current.lattitude_int / 1E7 - center.latitude) * M_PER_DEG_LAT
        east = (current.longitude_int / 1E7 - center.longitude) * M_PER_DEG_LAT * math.cos(math.radians(center.latitude))
        return north, east

    def update_keep_outs(self):
        """
        Put the keep-out circles on the Cube's fence, after flying out of any the drone is in. Runs in its own thread.
        """
        if not self.geofence.fence_vertices:
            return  # No boundary yet: configure_geofence uploads the circles with it.

        with self.fence_lock:
            try:
                while self.status.armed and self.inside_keep_out():
                    with self.incident_lock:
                        self.keep_out_leaving.set()
                    # Copter disarms on a breach while landed, so a circle over the pad waits until the takeoff is done.
                    if self.in_control() and self.status.landed_state == mavlink.MAV_LANDED_STATE_IN_AIR:
                        if self.leave_keep_outs():
                            break
                        time.sleep(1)
                    time.sleep(0.1)

                if not self.update_fence():
                    print("Couldn't put the keep-out circles on the fence.")
                    return
                print(f"Fence updated: {len(self.geofence.exclusion_areas)} keep-out circle(s).")

                # Re-send the emergency goal: the drone may have just left it, or it was inside a circle AllClear lifted.
                with self.incident_lock:
                    goal = self.incident_goal
                    if goal is not None and self.in_control():
                        self.move_global_gps_relative_alt(goal.lattitude_int, goal.longitude_int, goal.altitude_rel_m, timeout_s=0)
            finally:
                self.keep_out_leaving.clear()

    def inside_keep_out(self) -> bool:
        for center, radius_m in self.geofence.exclusion_areas:
            if math.hypot(*self.offset_from(center)) < radius_m + constants.KEEP_OUT_CLEARANCE_M:
                return True
        return False

    def leave_keep_outs(self) -> bool:
        """
        Fly out of every keep-out circle the drone is in, away from its center, before it goes on the fence.
        """
        for center, radius_m in self.geofence.exclusion_areas:
            north, east = self.offset_from(center)
            if math.hypot(north, east) >= radius_m + constants.KEEP_OUT_CLEARANCE_M:
                continue

            # Straight out from the center, or the nearest other direction that stays inside the boundary.
            bearing = math.atan2(east, north)
            exit_m = radius_m + constants.KEEP_OUT_EXIT_M
            m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(center.latitude))
            for turn_deg in (0, 45, -45, 90, -90, 135, -135, 180):
                angle = bearing + math.radians(turn_deg)
                latitude = center.latitude + exit_m * math.cos(angle) / M_PER_DEG_LAT
                longitude = center.longitude + exit_m * math.sin(angle) / m_per_deg_lon
                if self.geofence.contains(latitude, longitude):
                    break
            else:
                print("No way out of the keep-out circle inside the boundary.")
                return False

            print("Leaving a new keep-out circle before it goes on the fence.")
            altitude_m = self.location.global_frame_relative.altitude_rel_m
            self.move_global_gps_relative_alt(int(round(latitude * 1E7)), int(round(longitude * 1E7)), altitude_m, timeout_s=1.0)
            if not self.wait_for_condition(lambda: math.hypot(*self.offset_from(center)) >= radius_m + constants.KEEP_OUT_CLEARANCE_M,
                                           timeout_s=constants.KEEP_OUT_LEAVE_TIMEOUT_S):
                return False
        return True


    def clear_uncontrolled_event(self, message: mavlink.MAVLink_heartbeat_message):
        """
        Clears uncontrolled event when mode is changed to GUIDED. Subscribed to heartbeat messages.
        """
        if message.get_srcSystem() != 1:
            return
        mode = self.status.mode_map_bynumber.get(message.custom_mode)
        if mode == "GUIDED" and self.uncontrolled_event.is_set():
            print("Uncontrolled event cleared due to mode change to GUIDED.")
            self.uncontrolled_event.clear()
            self.cancel_mission_event.clear()
            self.intended_rtl_land = False
            self.mav_connection.system_status = constants.MIL_STATE_STANDBY

    
    def check_intended_mode(self, message: mavlink.MAVLink_heartbeat_message):
        """
        Check if mode is changed by an external source. 
        If so, trigger an abort and cede control of drone until mode is changed back to guided.
        """
        if message.get_srcSystem() != 1:
            return
        
        mode = self.status.mode_map_bynumber.get(message.custom_mode)
        manual_control = mode in self.MANUAL_MODES
        unintended_landing = mode in self.LANDING_MODES and not self.intended_rtl_land

        if (manual_control or unintended_landing) and not self.mav_connection.system_status == constants.MIL_STATE_INITIAL_OVERRIDE and not self.mav_connection.system_status == constants.MIL_STATE_OVERRIDE and not self.mav_connection.system_status == constants.MIL_STATE_EMERGENCY:
            print(f"Manual mode: {manual_control}, Mode String: {mode}")
            print(f"Intended Land: {self.intended_rtl_land}")
            print(f"System Status: {self.mav_connection.system_status}")
            self.uncontrolled_event.set()
            self.cancel_mission_event.set()
            self.mav_connection.system_status = constants.MIL_STATE_OVERRIDE
    

    def configure_messages(self):
        """
        Send requests for messages from autopilot to ensure required data is being sent.
        param2 is interval requested in microseconds.
        """
        ten_hz_us = (1/10) * 1E6
        five_hz_us = (1/5) * 1E6
        one_hz_us = 1E6
        two_hz_us = 500_000

        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, param2=ten_hz_us)
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, param2=ten_hz_us)
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_SYS_STATUS, param2=ten_hz_us)
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_ATTITUDE, param2=ten_hz_us)
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, param2=two_hz_us)
        self.send_command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, param2=one_hz_us)
    
    def properties_populated(self):
        """
        Return True if vehicle properties have been populated. False otherwise.
        """
        if self.location.lat_int is not None and self.location.x_north_m is not None and self.status.onboard_control_sensors_health is not None and self.location.roll_rad is not None:
                return True
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
            # TODO: Log retries
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
            
            if self.wait_for_condition(lambda: ack_event.is_set(), timeout_s=retry_timeout_s):
                break
            
            elif self.cancel_mission_event.is_set():
                self.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
                return False
            
        self.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
        return ack_result == mavlink.MAV_RESULT_ACCEPTED

    def takeoff(self, alt_m, timeout_s=30, threshold_m=0.5):
        """
        Wait for vehicle to arm and take off to alt_m meters.
        """
        start_s = time.monotonic()

        if not self.status.armed:
            text = "Takeoff: waiting to arm"
            print(text)
            self.mav.statustext_send(mavlink.MAV_SEVERITY_NOTICE, text.encode())
            if not self.arm(timeout_s=timeout_s):
                return False

        alt_m = self.cap_altitude(alt_m)
        if not self.send_command(command=mavlink.MAV_CMD_NAV_TAKEOFF, param7=alt_m):
            reason = "cancelled" if self.cancel_mission_event.is_set() else "not accepted"
            text = f"Takeoff to {alt_m:.1f} m {reason}"
            print(text)
            self.mav.statustext_send(mavlink.MAV_SEVERITY_NOTICE, text.encode())
            return False

        remaining_s = timeout_s - (time.monotonic() - start_s)
        if not self.wait_for_condition(lambda: self.altitude_reached(alt_m), timeout_s=remaining_s):
            altitude_rel_m = self.location.global_frame_relative.altitude_rel_m
            reached = "?" if altitude_rel_m is None else f"{altitude_rel_m:.1f}"
            reason = "cancelled" if self.cancel_mission_event.is_set() else "timed out"
            # Under 50 characters, STATUSTEXT's limit.
            text = f"Takeoff {reason} at {reached} of {alt_m:.1f} m"
            print(text)
            self.mav.statustext_send(mavlink.MAV_SEVERITY_NOTICE, text.encode())
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

    def ceiling_rel_m(self):
        """
        Highest GUIDED target, in m above home: GUIDED_CEILING_BUFFER_M under the fence ceiling. None without a position.
        """
        altitude_amsl_m = self.location.global_frame.altitude_m
        altitude_rel_m = self.location.global_frame_relative.altitude_rel_m
        if altitude_amsl_m is None or altitude_rel_m is None:
            return None
        home_amsl_m = altitude_amsl_m - altitude_rel_m
        return self.fence_alt_max_m - home_amsl_m - constants.GUIDED_CEILING_BUFFER_M

    def notify_target_outside_fence(self, latitude: float, longitude: float):
        """
        Send a NOTICE if GUIDED will ignore a position target here, which it does silently.
        A repeat of the last target checked isn't reported again.
        """
        target = (round(latitude, 7), round(longitude, 7))
        if target == self.last_target_checked:
            return
        self.last_target_checked = target
        reason = self.geofence.excludes(latitude, longitude)
        if reason is None:
            return
        # Under 50 characters, STATUSTEXT's limit.
        text = f"Target {latitude:.6f},{longitude:.6f} {reason}"
        print(text)
        self.mav.statustext_send(mavlink.MAV_SEVERITY_NOTICE, text.encode())

    def cap_altitude(self, alt_m):
        """
        alt_m (m above home), lowered to ceiling_rel_m() if above it.
        """
        ceiling_m = self.ceiling_rel_m()
        if ceiling_m is None or alt_m <= ceiling_m:
            return alt_m
        print(f"Altitude {alt_m:g} m capped to {ceiling_m:.1f} m, {constants.GUIDED_CEILING_BUFFER_M:g} m under the ceiling.")
        return ceiling_m


    def land(self, timeout_s=30.0):
        """
        Land the vehicle.
        """
        start_s = time.monotonic()

        self.intended_rtl_land = True

        if not self.send_command(command=mavlink.MAV_CMD_NAV_LAND):
            # TODO: Log
            return False
        
        remaining = timeout_s - (time.monotonic() - start_s)
        if not self.wait_for_condition(lambda: not self.status.armed, timeout_s=remaining):
            if not self.altitude_reached(0, threshold_m=0.5):
                # TODO: Log failed landing
                return False
            
        return True
    
 
    def arm(self, timeout_s=30.0):
        """
        Arm vehicle.
        """
        start_s = time.monotonic()

        if not self.wait_for_condition(lambda: self.status.prearmed, timeout_s=timeout_s):
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


    def move_body_frd_position(self, forward_m, right_m, down_m=0.0, yaw=0, yaw_rate=0, maintain_heading=True, timeout_s=30.0):
        """
        Move relative to vehicle's FRD frame by Forward/Right. Optionally wait for target to be reached within timeout_s seconds.
        Will maintain current altitude by default.
        """
        start_s = time.monotonic()
        XYZ_POS = mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE & \
        mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE & mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE & \
        mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE

        XYZ_POS_YAW = mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE | \
        mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE | \
        mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE & ~mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE

        typemask = XYZ_POS
        if not maintain_heading:
            typemask = XYZ_POS_YAW

        altitude_rel_m = self.location.global_frame_relative.altitude_rel_m
        if altitude_rel_m is not None:
            down_m = altitude_rel_m - self.cap_altitude(altitude_rel_m - down_m)

        target_ned = self.convert_frd_target_to_local_ned_target(forward_m, right_m, down_m)
        if target_ned is None:
            # TODO: Log
            return False

        position, yaw_rad = self.location.global_frame, self.location.attitude.yaw_rad
        if position.lattitude_int is not None:
            latitude = position.lattitude_int / 1E7 + (forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)) / M_PER_DEG_LAT
            longitude = position.longitude_int / 1E7 + (forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)) / (M_PER_DEG_LAT * math.cos(math.radians(latitude)))
            self.notify_target_outside_fence(latitude, longitude)

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
            yaw = yaw,
            yaw_rate = yaw_rate
        )

        remaining_s = timeout_s - (time.monotonic() - start_s)
        if maintain_heading:
            cancel = lambda: self.target_ned_reached(target_ned)
        else:
            cancel = lambda: False
        if not self.wait_for_condition(cancel, timeout_s=remaining_s):
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

            return not maintain_heading

        return True


    def move_global_gps_relative_alt(self, lat_int: int, lon_int: int, alt_m: int, timeout_s=30.0):
        """
        Move to the given GPS WGS84 coordinates. Altitude is relative to home position. Maintain current heading.
        """
        #0b110111111000
        start_s = time.monotonic()
        # Position only: a target with velocity skips the path planner.
        typemask = mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE | \
        mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE

        alt_m = self.cap_altitude(alt_m)
        target_global_rel_pos = MavFrameGlobalRel(lat_int, lon_int, alt_m)
        self.notify_target_outside_fence(lat_int / 1E7, lon_int / 1E7)

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

        remaining_s = timeout_s - (time.monotonic() - start_s)

        if not self.wait_for_condition(lambda: self.target_global_reached(target_global_rel_pos), timeout_s=remaining_s):
            # Stop movement
            #self.mav.set_position_target_global_int_send(
            #    time_boot_ms=0,
            #    target_system=1,
            #    target_component=0,
            #    coordinate_frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            #    type_mask=typemask,
            #    x = 0,
            #    y = 0,
            #    z = 0,
            #    vx = 0,
            #    vy = 0,
            #    vz = 0,
            #    afx = 0,
            #    afy = 0,
            #    afz = 0,
            #    yaw = 0,
            #    yaw_rate = 0
            #)

            return False

        return True

    def set_mode(self, target_mode: str | int, timeout_s=5.0):
        """
        Change mode of the flight controller. See: https://ardupilot.org/copter/docs/parameters.html#fltmode1
        Common modes: 'GUIDED', 'LAND', 'CIRCLE'
        """
        start_s = time.monotonic()
        
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
        
        remaining_s = timeout_s - (time.monotonic() - start_s)
        if not self.wait_for_condition(lambda: self.check_mode(target_mode), timeout_s=remaining_s):
            # TODO: Log failure
            return False
        
        return True


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

        if current_ned is None:
            return False

        dn = current_ned.x_north_m - target_ned.x_north_m
        de = current_ned.y_east_m - target_ned.y_east_m
        dd = current_ned.z_down_m - target_ned.z_down_m

        distance_m = math.sqrt(dn*dn + de*de + dd*dd)

        return distance_m < threshold_m

    def target_global_reached(self, target_global: MavFrameGlobalRel, threshold_m=0.1) -> bool:
        """
        Check if target global position (altitude relative to home) has been reached within threshold_m meters.
        """
        M_PER_DEG_LAT = 111_320.0

        current = self.location.global_frame_relative

        if current.lattitude_int is None or current.longitude_int is None or current.altitude_rel_m is None:
            return False

        # lat/lon are degrees * 1E7. Equirectangular is close enough over a course.
        dn = (current.lattitude_int - target_global.lattitude_int) / 1E7 * M_PER_DEG_LAT
        de = (current.longitude_int - target_global.longitude_int) / 1E7 * M_PER_DEG_LAT * math.cos(math.radians(current.lattitude_int / 1E7))
        dd = current.altitude_rel_m - target_global.altitude_rel_m

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
        start_s = time.monotonic()

        while time.monotonic() - start_s < timeout_s:
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
            start_s = time.monotonic()
        
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
        start_s = time.monotonic()
        while self.detection_queue.empty():
            if time.monotonic() - start_s > 16:
                break
            time.sleep(0.5)

        self.set_mode("GUIDED")
        
        if not self.detection_queue.empty():
            return self.location.global_frame_relative
        
        print("Starting second Circle")

        self.circle(radius_m=0.5, center_gps=center_gps)
        start_s = time.monotonic()
        while self.detection_queue.empty():
            if time.monotonic() - start_s > 16:
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

    def set_parameter(self, parameter: str, value: float, timeout_s: float = 4.0, retries: int = 3) -> bool:
        """
        Write one flight controller parameter and wait for the PARAM_VALUE the autopilot echoes back.
        PARAM_SET is unacknowledged, so without the echo a dropped packet is indistinguishable from
        success. Return True once the autopilot reports the value we asked for.
        """
        confirmed = Event()

        def on_param_value(message: mavlink.MAVLink_param_value_message):
            if message.get_srcSystem() != 1:
                return
            # pymavlink strips the NUL padding from the 16 byte param_id field and hands back a str.
            if message.param_id == parameter and math.isclose(message.param_value, value, rel_tol=1E-6, abs_tol=1E-6):
                confirmed.set()

        self.subscribe(mavlink.MAVLink_param_value_message.msgname)(on_param_value)

        try:
            for _ in range(retries):
                confirmed.clear()

                self.mav.param_set_send(
                    target_system=1,
                    target_component=mavlink.MAV_COMP_ID_AUTOPILOT1,
                    param_id=parameter.encode("utf-8"),
                    param_value=float(value),
                    param_type=mavlink.MAV_PARAM_TYPE_REAL32
                )

                if self.wait_for_condition(lambda: confirmed.is_set(), timeout_s=timeout_s):
                    return True

            print(f"Parameter {parameter} was not confirmed as {value} after {retries} attempts.")
            return False

        finally:
            self.unsubscribe(mavlink.MAVLink_param_value_message.msgname, on_param_value)

    def get_parameter(self, parameter: str, timeout_s: float = 2.0, retries: int = 3) -> float | None:
        """
        Read one flight controller parameter. None if the autopilot never answered.
        """
        received = Event()
        value = None

        def on_param_value(message: mavlink.MAVLink_param_value_message):
            nonlocal value
            if message.get_srcSystem() != 1 or message.param_id != parameter:
                return
            value = message.param_value
            received.set()

        self.subscribe(mavlink.MAVLink_param_value_message.msgname)(on_param_value)

        try:
            for _ in range(retries):
                self.mav.param_request_read_send(
                    target_system=1,
                    target_component=mavlink.MAV_COMP_ID_AUTOPILOT1,
                    param_id=parameter.encode("utf-8"),
                    param_index=-1
                )

                if self.wait_for_condition(lambda: received.is_set(), timeout_s=timeout_s):
                    return value

            print(f"Parameter {parameter} could not be read after {retries} attempts.")
            return None

        finally:
            self.unsubscribe(mavlink.MAVLink_param_value_message.msgname, on_param_value)

    def clear_stale_fence(self, timeout_s: float = 2.0, retries: int = 3) -> bool:
        """
        Disable and delete the fence stored on the flight controller at startup. The Cube keeps its fence
        across reboots, so it's from an earlier session and may be another course's.
        Skipped while armed: the drone may be mid-run with this course's fence after a Pi restart.
        Never resets a fence the OCS already put live, even if called out of order.
        """
        if self.status.armed:
            print("Armed at startup: keeping the flight controller's fence.")
            return False

        # fence_lock is held while an OCS fence is applied, so this can't interleave with one.
        with self.fence_lock:
            if self.fence_live.is_set():
                print("A fence from the OCS is already live: keeping it.")
                return False

            if not self.set_parameter("FENCE_ENABLE", 0):
                print("Couldn't disable the stored fence.")
                return False

            acknowledged = Event()
            ack_result = None

            def on_ack(message: mavlink.MAVLink_mission_ack_message):
                if message.mission_type != mavlink.MAV_MISSION_TYPE_FENCE or message.get_srcComponent() != mavlink.MAV_COMP_ID_AUTOPILOT1:
                    return
                nonlocal ack_result
                ack_result = message.type
                acknowledged.set()

            self.subscribe(mavlink.MAVLink_mission_ack_message.msgname)(on_ack)
            try:
                for _ in range(retries):
                    acknowledged.clear()
                    self.mav.mission_clear_all_send(1, mavlink.MAV_COMP_ID_AUTOPILOT1, mission_type=mavlink.MAV_MISSION_TYPE_FENCE)
                    if acknowledged.wait(timeout_s):
                        break
            finally:
                self.unsubscribe(mavlink.MAVLink_mission_ack_message.msgname, on_ack)

        if ack_result != mavlink.MAV_MISSION_ACCEPTED:
            print("Stored fence disabled, but deleting it wasn't acknowledged.")
            return False
        print("Stored fence from an earlier session disabled and deleted.")
        return True

    def set_course_boundary(self, corners, exclusion_zones=()) -> bool:
        """
        Take the course boundary for this run and put it live on the flight controller as the UAV
        geofence. Single entry point for a received RxCourse boundary.

        exclusion_zones: polygons to keep out of, uploaded with the boundary.

        Returns False without enabling the fence if the upload fails, and forgets the geofence
        in that case. RunDeclaration reports the geofence we enforce, so a fence that failed to upload
        must not be left behind for it to declare.
        """
        if not self.geofence.set_course_boundary(corners) or not self.geofence.set_exclusion_zones(exclusion_zones) or not self.configure_geofence():
            self.geofence.clear()
            return False

        print(f"UAV geofence live: {len(self.geofence.fence_vertices)} vertices, {len(self.geofence.exclusion_vertices)} exclusion zone(s), "
              f"{constants.GEOFENCE_INSET_M} m inside the course boundary, {self.fence_alt_max_m} m AMSL ceiling.")
        return True

    def configure_geofence(self) -> bool:
        """
        Upload the geofence and enable it, before the run. Any failure leaves the fence disabled.
        OA_TYPE is checked, never written.
        """
        vertices = self.geofence.fence_vertices
        exclusions, circles = self.geofence.exclusion_vertices, self.geofence.exclusion_areas
        self.cube_fence = []

        if not vertices:
            print("No UAV geofence to upload. No course boundary has been received.")
            return False

        # Read first, so a failed read leaves the current fence untouched.
        oa_type = self.get_parameter("OA_TYPE")
        guided_options = self.get_parameter("GUID_OPTIONS")
        if oa_type is None or guided_options is None:
            return False

        if not self.set_parameter("FENCE_ENABLE", 0):
            return False

        if not self.mav_connection.upload_fence_polygon(vertices, self.cancel_mission_event, exclusions, circles):
            return False

        print("Uploaded geofence, checking for mismatch")
        time.sleep(1) # give time for update

        # The OCS is told what ArduPilot stored, not what was computed: read it back, and don't enable a mismatch.
        stored = self.mav_connection.read_cube_fence()
        expected = [(int(round(v.latitude * 1E7)), int(round(v.longitude * 1E7))) for v in vertices]
        inclusion = [(item.x, item.y) for item in stored or [] if item.command == mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION]
        if stored is None or inclusion != expected or len(stored) != len(vertices) + sum(map(len, exclusions)) + len(circles):
            print("The fence read back from the flight controller doesn't match what was uploaded.")
            return False
        self.cube_fence = [LatLng(x / 1E7, y / 1E7) for x, y in inclusion]

        ceiling_parameters = self.fence_ceiling_parameters(self.fence_alt_max_m)
        if ceiling_parameters is None:
            return False

        parameters = [
            ("FENCE_TYPE", constants.FENCE_TYPE_UAV),
            *ceiling_parameters,
            ("FENCE_ACTION", constants.FENCE_ACTION_RTL),
            ("FENCE_MARGIN", constants.FENCE_MARGIN_M),
            ("OA_MARGIN_MAX", constants.OA_MARGIN_MAX_M),
            # Keep the Cube's other bits.
            ("GUID_OPTIONS", int(guided_options) | constants.GUID_OPTIONS_WPNAV_POSITION_TARGETS),
        ]

        for parameter, value in parameters:
            if not self.set_parameter(parameter, value):
                return False

        if not self.read_fence_ceiling():
            return False

        if not self.set_parameter("FENCE_ENABLE", 1):
            return False

        # Setting OA_TYPE = 2 allows for drone path planning around exclusion zones.
        if oa_type != 2:
            print(f"OA_TYPE is {oa_type:g}, not 2: paths won't be planned around exclusion zones.")
        return True

    def fence_ceiling_parameters(self, ceiling_amsl_m: float) -> list[tuple[str, float]] | None:
        """
        Parameters that put an AMSL ceiling on the Cube, which measures it from home. None without a position.
        """
        altitude_amsl_m = self.location.global_frame.altitude_m
        altitude_rel_m = self.location.global_frame_relative.altitude_rel_m

        if altitude_amsl_m is None or altitude_rel_m is None:
            print("No GPS Fix. Cannot place the fence ceiling above home.")
            return None
        
        parameters = [("FENCE_ALT_MAX", ceiling_amsl_m - (altitude_amsl_m - altitude_rel_m))]
        if self.sim:
            # SITL's newer firmware has a ceiling frame, possibly saved as AMSL by earlier runs.
            parameters.append(("FENCE_ALT_MAX_TP", constants.FENCE_ALT_FRAME_HOME))
        return parameters

    def geoid_height_m(self) -> float | None:
        """
        HAE minus AMSL here. None without GPS altitudes.
        """
        altitude_hae_m = self.location.altitude_hae_m
        altitude_amsl_m = self.location.global_frame.altitude_m

        if altitude_hae_m is None or altitude_amsl_m is None:
            return None

        return altitude_hae_m - altitude_amsl_m

    def amsl_ceiling_from_hae(self, max_altitude_hae_m: float) -> float | None:
        """
        A max altitude in m HAE as an AMSL ceiling. None without GPS altitudes or if not above home.
        """
        geoid_m = self.geoid_height_m()
        altitude_rel_m = self.location.global_frame_relative.altitude_rel_m
        
        if geoid_m is None or altitude_rel_m is None:
            return None
        
        ceiling_amsl_m = max_altitude_hae_m - geoid_m - constants.MAX_ALT_MARGIN_M
        home_amsl_m = self.location.global_frame.altitude_m - altitude_rel_m
        
        if ceiling_amsl_m <= home_amsl_m:
            return None
        
        return ceiling_amsl_m

    def fence_ceiling_hae_m(self) -> float:
        """
        The Cube's ceiling in m HAE, for the OCS to declare. 0 without GPS altitudes.
        """
        geoid_m = self.geoid_height_m()
        altitude_rel_m = self.location.global_frame_relative.altitude_rel_m

        if geoid_m is None or altitude_rel_m is None or self.cube_ceiling_m is None:
            return 0.0

        home_amsl_m = self.location.global_frame.altitude_m - altitude_rel_m
        return self.cube_ceiling_m + home_amsl_m + geoid_m

    def read_fence_ceiling(self) -> bool:
        """
        Read FENCE_ALT_MAX back from the Cube, so the OCS is told what ArduPilot stored.
        """
        ceiling_m = self.get_parameter("FENCE_ALT_MAX")
        
        if ceiling_m is None:
            return False
        
        self.cube_ceiling_m = ceiling_m
        return True

    def set_fence_ceiling(self, ceiling_amsl_m: float):
        """
        Put a new AMSL ceiling on the Cube, and use it for later fences and the GUIDED cap.
        """
        with self.fence_lock:
            parameters = self.fence_ceiling_parameters(ceiling_amsl_m)

            if parameters is None or not all(self.set_parameter(*parameter) for parameter in parameters) or not self.read_fence_ceiling():
                print("Failed to put maximum altitude on the flight controller.")
                self.mav.statustext_send(mavlink.MAV_SEVERITY_WARNING, constants.STATUS_MAX_ALTITUDE_FAILED.encode())
                
                self.max_altitude_hae_m = None 
                return
            
            self.fence_alt_max_m = ceiling_amsl_m
            print(f"Fence ceiling is now {ceiling_amsl_m:.1f} m AMSL.")

    def update_fence(self) -> bool:
        """
        Re-upload the boundary, exclusion polygons and keep-out circles without touching FENCE_ENABLE, for use in flight.
        The drone must not be inside any of them, or the upload itself is a breach.
        """
        return self.mav_connection.upload_fence_polygon(self.geofence.fence_vertices, self.cancel_mission_event, self.geofence.exclusion_vertices, self.geofence.exclusion_areas)

    def on_fence_upload(self, message):
        """
        Queue the OCS's fence upload (MISSION_COUNT, MISSION_ITEM_INT addressed to this component) for receive_fences.
        """
        # The Cube's replies to read_cube_fence are addressed here too.
        if message.get_srcComponent() == mavlink.MAV_COMP_ID_AUTOPILOT1:
            return
        if message.mission_type == mavlink.MAV_MISSION_TYPE_FENCE and message.target_component == mavlink.MAV_COMP_ID_ONBOARD_COMPUTER:
            self.fence_inbox.put(message)

    def receive_fences(self):
        """
        Receive course fences from the OCS for as long as the companion runs. Runs in its own thread.
        """
        count = None
        while True:
            if count is None:
                count = self.fence_inbox.get()
                if count.get_type() != mavlink.MAVLink_mission_count_message.msgname:
                    count = None
                    continue
            count = self.receive_fence(count)

    def receive_fence(self, count: mavlink.MAVLink_mission_count_message):
        """
        The receiving end of the fence upload that upload_fence_polygon makes to the Cube: request each item, ack,
        then put the fence live while disarmed. Returns a MISSION_COUNT that restarted the upload, or None.
        """
        sender = (count.get_srcSystem(), count.get_srcComponent())
        fence = mavlink.MAV_MISSION_TYPE_FENCE

        items = []
        for seq in range(count.count):
            item = None
            for _ in range(3):
                self.mav.mission_request_int_send(*sender, seq, mission_type=fence)
                deadline_s = time.monotonic() + 1.0
                while item is None and (remaining_s := deadline_s - time.monotonic()) > 0:
                    try:
                        message = self.fence_inbox.get(timeout=remaining_s)
                    except Empty:
                        break
                    if message.get_type() == mavlink.MAVLink_mission_count_message.msgname:
                        return message
                    if message.seq == seq:
                        item = message
                if item is not None:
                    break
            if item is None:
                print("Fence upload from the OCS stopped arriving.")
                self.mav.mission_ack_send(*sender, mavlink.MAV_MISSION_ERROR, mission_type=fence)
                return None
            items.append(item)

        result, boundary, exclusions = self.parse_fence(items)
        apply = result == mavlink.MAV_MISSION_ACCEPTED
        if apply and self.fence_live.is_set() and boundary == self.geofence.course_boundary and exclusions == self.geofence.exclusion_vertices:
            print("Received the same course fence again.")
            apply = False
        elif apply and self.status.armed:
            print("Course fence denied: armed.")
            result, apply = mavlink.MAV_MISSION_DENIED, False

        if apply:
            print(f"Received course fence: {len(boundary) - 1} vertices, {len(exclusions)} exclusion zone(s).")
            self.fence_live.clear()
            with self.fence_lock:
                if self.set_course_boundary(boundary, exclusions):
                    self.fence_live.set()
                else:
                    print("Couldn't put the course fence on the flight controller.")
                    result = mavlink.MAV_MISSION_ERROR

        # Acked once the fence is live, so after ACCEPTED the OCS can download the inset fence (on_fence_download).
        self.mav.mission_ack_send(*sender, result, mission_type=fence)
        return None

    def on_fence_download(self, message):
        """
        Answer the OCS's fence download from this component with the inclusion polygon read back from the Cube after the
        upload (cube_fence): the inset boundary ArduPilot stored, not the one the OCS sent. Open; no fence live is a count of 0.
        """
        if message.mission_type != mavlink.MAV_MISSION_TYPE_FENCE or message.target_component != mavlink.MAV_COMP_ID_ONBOARD_COMPUTER:
            return
        # The Cube's own requests during upload_fence_polygon are addressed here too.
        if message.get_srcComponent() == mavlink.MAV_COMP_ID_AUTOPILOT1:
            return
        sender = (message.get_srcSystem(), message.get_srcComponent())

        if message.get_type() == mavlink.MAVLink_mission_request_list_message.msgname:
            vertices = self.cube_fence if self.fence_live.is_set() else []
            ceiling_hae_m = self.fence_ceiling_hae_m()
            self.fence_downloads[sender] = (vertices, ceiling_hae_m)
            self.mav.mission_count_send(*sender, len(vertices), mission_type=mavlink.MAV_MISSION_TYPE_FENCE)
            if vertices:
                self.mav.named_value_float_send(0, constants.NAMED_VALUE_MAX_ALT.encode(), ceiling_hae_m)
            return

        vertices, ceiling_hae_m = self.fence_downloads.get(sender, (None, 0.0))
        if vertices is None or not 0 <= message.seq < len(vertices):
            self.mav.mission_ack_send(*sender, mavlink.MAV_MISSION_INVALID_SEQUENCE, mission_type=mavlink.MAV_MISSION_TYPE_FENCE)
            return
        vertex = vertices[message.seq]
        self.mav.mission_item_int_send(
            *sender,
            seq=message.seq,
            frame=mavlink.MAV_FRAME_GLOBAL,
            command=mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            current=0,
            autocontinue=0,
            param1=len(vertices),
            param2=0,
            param3=0,
            param4=0,
            x=int(round(vertex.latitude * 1E7)),
            y=int(round(vertex.longitude * 1E7)),
            z=ceiling_hae_m,
            mission_type=mavlink.MAV_MISSION_TYPE_FENCE
        )

    @staticmethod
    def parse_fence(items) -> tuple[int, list[LatLng], list[list[LatLng]]]:
        """
        Split fence MISSION_ITEM_INTs into one closed boundary and open exclusion polygons. Returns a MAV_MISSION_RESULT too.
        """
        inclusion, exclusion = mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        boundary, exclusions = None, []

        start = 0
        while start < len(items):
            first = items[start]
            if first.command not in (inclusion, exclusion):
                return mavlink.MAV_MISSION_UNSUPPORTED, [], []
            vertex_count = int(first.param1)
            polygon = items[start:start + vertex_count]
            if vertex_count < 1 or len(polygon) != vertex_count or any(item.command != first.command or int(item.param1) != vertex_count for item in polygon):
                return mavlink.MAV_MISSION_INVALID, [], []

            vertices = [LatLng(item.x / 1E7, item.y / 1E7) for item in polygon]
            if len(vertices) > 1 and vertices[0] == vertices[-1]:
                vertices = vertices[:-1]
            if not MIN_POLYGON_VERTICES <= len(vertices) <= MAX_POLYGON_VERTICES:
                return mavlink.MAV_MISSION_INVALID, [], []

            if first.command == exclusion:
                exclusions.append(vertices)
            elif boundary is None:
                boundary = vertices + vertices[:1]
            else:
                return mavlink.MAV_MISSION_INVALID, [], []
            start += vertex_count

        if boundary is None:
            return mavlink.MAV_MISSION_INVALID, [], []
        # Refused up front rather than failing after the ack.
        if not inset_polygon(boundary):
            print(f"Course boundary can't be inset {constants.GEOFENCE_INSET_M} m: a repeated vertex, a hairpin, or too small.")
            return mavlink.MAV_MISSION_INVALID, [], []
        return mavlink.MAV_MISSION_ACCEPTED, boundary, exclusions

    def reset_state(self):
        """
        Reset default states after mission completion.
        """
        self.cancel_mission_event.clear()
        self.intended_rtl_land = False

        if not self.uncontrolled_event.is_set():
            self.mav_connection.system_status = constants.MIL_STATE_STANDBY

    def close(self):
        #self.mav_connection.close()
        # TODO: Check that threads in mavconnection are closed correctly
        self.camera.stop()

    open = 0

    def trigger_magnet(self, input):
        import lgpio

        if open == 0:   
            h = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(h,18)
            open = 1

        if input == 1:
            lgpio.tx_pwm(h,18,50,9)

        elif input == 0:
            lgpio.tx_pwm(h,18,50,5)

#        lgpio.gpiochip_close(h)
