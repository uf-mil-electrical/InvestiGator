from queue import Queue
from threading import Thread, Event, Lock
from typing import cast
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from .pubsub import PublicationManager, SubscriptionManager
from .constants import MIL_STATE_CONNECTING, RX_TASK_NONE
from .geofence import LatLng


class MAVWriter:
    """
    Acts as a file to redirect messages sent with mav_connection.send() to a queue. Taken from Dronekit-Python library. 
     """

    def __init__(self, queue):
        self.queue = queue

    def write(self, message):
        self.queue.put(message)

    @staticmethod
    def read():
        print("Should not read from this file!")


class MAVConnection:
    """
    Reads from and writes to a MAVLink device.
    """

    def __init__(self, address, mav_type, baud=112500, source_system=225, source_component=0, timeout_s=30):

        self.mav_connection = cast(mavutil.mavfile, mavutil.mavlink_connection(address, baud, source_system, source_component))

        self.send_queue = Queue()
        self.receive_queue = Queue()
        self.running = Event()
        self.running.set()

        self.mav_connection.mav.file = cast(mavlink.Any, MAVWriter(self.send_queue))

        self.pub_manager = PublicationManager(send_queue=self.send_queue)
        self.sub_manager = SubscriptionManager(receive_queue=self.receive_queue)

        self.publish = self.pub_manager.publish
        self.publish_function = self.pub_manager.register_publisher
        self.unpublish = self.pub_manager.remove_publisher
        self.subscribe = self.sub_manager.subscribe
        self.unsubscribe = self.sub_manager.unsubscribe

        self.heartbeat_lock = Lock()
        self.mav_type = mav_type
        self._system_status = MIL_STATE_CONNECTING
        # RxTask advertised in HEARTBEAT.custom_mode. Only the companion computer changes this.
        # Must default to RX_TASK_NONE: 0 is TASK_UNKNOWN, which must never be read as standing down.
        self._custom_mode = RX_TASK_NONE

        def mav_sender():
            while self.running.is_set():
                message: mavlink.MAVLink_message = self.send_queue.get(block=True)
                self.mav_connection.write(message)

        def mav_reader():
            while self.running.is_set():
                self.mav_connection.select(timeout=0.05)
                message = self.mav_connection.recv_match()
                if message is not None:
                    self.receive_queue.put(message)


        self.send_thread = Thread(name="mav_sender Thread", target=mav_sender, daemon=True)
        self.read_thread = Thread(name="mav_reader Thread", target=mav_reader, daemon=True)

        self.send_thread.start()
        self.read_thread.start()

        @self.publish('HEARTBEAT', 2)
        def publish_heartbeat():
            with self.heartbeat_lock:
                self.mav.heartbeat_send(
                    type=self.mav_type,
                    autopilot=mavlink.MAV_AUTOPILOT_INVALID,
                    base_mode=0,
                    custom_mode=self._custom_mode,
                    system_status=self._system_status
                )

        print("MAVConnection waiting for first heartbeat")
        self.wait_for_first_heartbeat(timeout_s=timeout_s)
        
    def wait_for_first_heartbeat(self, timeout_s):
        
        heartbeat_received = Event()

        @self.subscribe(mavlink.MAVLink_heartbeat_message.msgname)
        def on_heartbeat(message: mavlink.MAVLink_heartbeat_message):
            print("Heartbeat from system (system %u component %u)" % (self.mav_connection.target_system, self.mav_connection.target_component))
            heartbeat_received.set()

        try:
            start_s = time.monotonic()
            while time.monotonic() - start_s < timeout_s:
                if heartbeat_received.is_set():
                    break
                heartbeat_received.wait(timeout=0.1)

            if not heartbeat_received.is_set():
                raise TimeoutError(f"Connection Failed: No heartbeat received from system within {timeout_s} seconds.")
            
            self.sub_manager.unsubscribe(mavlink.MAVLink_heartbeat_message.msgname, on_heartbeat)
        
        except (KeyboardInterrupt, TimeoutError):
            self.close()
            raise

    def wait_for_condition_with_cancel(self, cancel_event):
        def wait_for_condition(condition_function, timeout_s=30.0, interval_s=0.1):
            """
            Wait for condition function to be True within timeout_s seconds, checking every interval_s seconds.
            If mission_cancel_event is set, returns False.
            Return True if condition met, False if timeout reached.
            """
            start_s = time.monotonic()
            while time.monotonic() - start_s < timeout_s:
                if cancel_event.wait(timeout=interval_s):
                    return False
                if condition_function():
                    return True
            return False

        return wait_for_condition

    def upload_fence_polygon(self, vertices: list[LatLng], cancel_event: Event, exclusions: list[list[LatLng]] = (), circles: list[tuple[LatLng, float]] = (), timeout_s: float = 5.0, retries: int = 3, target_component: int = mavlink.MAV_COMP_ID_AUTOPILOT1) -> bool:
        """
        Upload vertices to the flight controller as the inclusion fence, each polygon in
        exclusions as an exclusion fence, and each (center, radius_m) in circles as an exclusion circle.

        Uses the mission protocol with MAV_MISSION_TYPE_FENCE: MISSION_COUNT, then one MISSION_ITEM_INT
        per vertex as the autopilot requests them, then MISSION_ACK. The legacy FENCE_TOTAL/FENCE_POINT
        message pair is not an option: ArduCopter 4.8 has no handler for it, only the FENCE_STATUS
        downlink. This replaces whatever fence was stored; there is no partial update.

        No polygon may repeat its first point. Each goes up as a contiguous run of items, every vertex
        carrying its own polygon's vertex count in param1.
        """
        acknowledged = Event()
        ack_result = None

        # (command, param1, point) per mission item. param1 is the polygon's vertex count, or a circle's radius.
        items = [(mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, len(vertices), vertex) for vertex in vertices]
        for polygon in exclusions:
            items += [(mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, len(polygon), vertex) for vertex in polygon]
        items += [(mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, radius_m, center) for center, radius_m in circles]

        def send_vertex(seq: int):
            if not 0 <= seq < len(items):
                return

            command, vertex_count, vertex = items[seq]
            self.mav.mission_item_int_send(
                target_system=1,
                target_component=target_component,
                seq=seq,
                frame=mavlink.MAV_FRAME_GLOBAL,
                command=command,
                current=0,
                autocontinue=0,
                param1=vertex_count,
                param2=0,
                param3=0,
                param4=0,
                x=int(round(vertex.latitude * 1E7)),
                y=int(round(vertex.longitude * 1E7)),
                z=0,
                mission_type=mavlink.MAV_MISSION_TYPE_FENCE
            )

        # Only the Cube's messages: the OCS also talks fence protocol to this component (receive_fence, on_fence_download).
        def from_target(message) -> bool:
            return message.mission_type == mavlink.MAV_MISSION_TYPE_FENCE and message.get_srcComponent() == target_component

        def on_request_int(message: mavlink.MAVLink_mission_request_int_message):
            if from_target(message):
                send_vertex(message.seq)

        def on_request(message: mavlink.MAVLink_mission_request_message):
            # ArduPilot asks with MISSION_REQUEST_INT. Older stacks ask with the float MISSION_REQUEST;
            # either way the vertex goes back as MISSION_ITEM_INT so no precision is lost.
            if from_target(message):
                send_vertex(message.seq)

        def on_ack(message: mavlink.MAVLink_mission_ack_message):
            if not from_target(message):
                return
            nonlocal ack_result
            ack_result = message.type
            acknowledged.set()

        self.subscribe(mavlink.MAVLink_mission_request_int_message.msgname)(on_request_int)
        self.subscribe(mavlink.MAVLink_mission_request_message.msgname)(on_request)
        self.subscribe(mavlink.MAVLink_mission_ack_message.msgname)(on_ack)

        try:
            for _ in range(retries):
                acknowledged.clear()
                ack_result = None

                self.mav.mission_count_send(
                    target_system=1,
                    target_component=target_component,
                    count=len(items),
                    mission_type=mavlink.MAV_MISSION_TYPE_FENCE
                )

                # One timeout covers the whole request/item exchange, not a single item.
                if self.wait_for_condition_with_cancel(cancel_event)(lambda: acknowledged.is_set(), timeout_s=timeout_s):
                    if ack_result == mavlink.MAV_MISSION_ACCEPTED:
                        return True

                    print(f"Fence upload rejected: {mavlink.enums['MAV_MISSION_RESULT'][ack_result].name}.")
                    return False

            print(f"Fence upload was not acknowledged after {retries} attempts.")
            return False

        finally:
            self.unsubscribe(mavlink.MAVLink_mission_request_int_message.msgname, on_request_int)
            self.unsubscribe(mavlink.MAVLink_mission_request_message.msgname, on_request)
            self.unsubscribe(mavlink.MAVLink_mission_ack_message.msgname, on_ack)

    @property
    def mav(self) -> mavlink.MAVLink:
        """
        Cast self.mav_connection.mav as ardupilotmega MAVLink dialect to allow for type hinting and autocompletion.
        """
        return cast(mavlink.MAVLink, self.mav_connection.mav)
    
    @property
    def system_status(self):
        with self.heartbeat_lock:
            return self._system_status
        
    @system_status.setter
    def system_status(self, system_status):
        with self.heartbeat_lock:
            self._system_status = system_status

    @property
    def custom_mode(self):
        with self.heartbeat_lock:
            return self._custom_mode

    @custom_mode.setter
    def custom_mode(self, custom_mode):
        with self.heartbeat_lock:
            self._custom_mode = custom_mode

    def stop_threads(self):
        if self.send_thread.is_alive():
            self.send_thread.join(timeout=1)
            if self.send_thread.is_alive():
                print("Sender thread join timed out.")

        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1)
            if self.read_thread.is_alive():
                print("Reader thread join timed out.")

    def close(self):
        self.running.clear()
        self.pub_manager.close()
        self.sub_manager.close()
        self.stop_threads()
        self.mav_connection.close()
