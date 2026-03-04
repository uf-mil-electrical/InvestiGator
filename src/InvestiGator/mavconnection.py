from queue import Queue, ShutDown
from threading import Thread, Event
from typing import cast

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from pubsub import PublicationManager, SubscriptionManager


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

    def __init__(self, address, baud=112500, source_system=225, source_component=0):

        print("MAVConnection waiting for heartbeat")
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
            self.mav.heartbeat_send(
                type=mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=0
            )

        self.mav_connection.wait_heartbeat(blocking=True)
        print("Heartbeat from system (system %u component %u)" % (self.mav_connection.target_system, self.mav_connection.target_component))

    @property
    def mav(self) -> mavlink.MAVLink:
        """
        Cast self.mav_connection.mav as ardupilotmega MAVLink dialect to allow for type hinting and autocompletion.
        """
        return cast(mavlink.MAVLink, self.mav_connection.mav)

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
