from queue import Queue, ShutDown
from threading import Thread, Event, Lock
from typing import cast
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from .pubsub import PublicationManager, SubscriptionManager
from .constants import MIL_STATE_CONNECTING, RX_TASK_NONE


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

        self.heartbeat_lock = Lock()
        self.mav_type = mav_type
        self._system_status = MIL_STATE_CONNECTING
        # RxTask advertised in HEARTBEAT.custom_mode. Only the companion computer changes this.
        # Must default to RX_TASK_NONE: 0 is TASK_UNKNOWN, which must never be read as standing down.
        self._custom_mode = RX_TASK_NONE

        def mav_sender():
            while self.running.is_set():
                try:
                    message: mavlink.MAVLink_message = self.send_queue.get(block=True)
                    self.mav_connection.write(message)
                except ShutDown:
                    break

        def mav_reader():
            while self.running.is_set():
                self.mav_connection.select(timeout=0.05)
                message = self.mav_connection.recv_match()
                if message is not None:
                    try:
                        self.receive_queue.put(message)
                    except ShutDown:
                        break

        self.send_thread = Thread(name="mav_sender Thread", target=mav_sender, daemon=True)
        self.read_thread = Thread(name="mav_reader Thread", target=mav_reader, daemon=True)

        self.send_thread.start()
        self.read_thread.start()

        @self.publish('HEARTBEAT', 1)
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
        self.send_queue.shutdown(immediate=True)
        self.receive_queue.shutdown(immediate=True)
        self.running.clear()
        self.pub_manager.close()
        self.sub_manager.close()
        self.stop_threads()
        self.mav_connection.close()
