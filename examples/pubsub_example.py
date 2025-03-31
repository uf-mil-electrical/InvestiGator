"""
Example of the pub/sub or observer pattern and how to use decorators to subscribe functions.
For more information, see: Observer Pattern, Publish/Subscribe Pattern articles on wikipedia.

This example copies SubscriptionManager from pubsub.py as Stream.
"""
import queue
import threading
import time

from pymavlink.dialects.v20 import ardupilotmega as mavlink


class Stream:
    """
    A publisher of messages that are categorized by topic. Implements methods to register and deregister subscribers.
    """

    def __init__(self, source_queue: queue.Queue):
        self.source_queue = source_queue
        self.subscribers = {}
        self.running = threading.Event()
        self.running.set()

        def source():
            """
            The stream gets messages from IO in its own thread.
            """
            while self.running.is_set():
                try:
                    message: mavlink.MAVLink_message = self.source_queue.get(block=True)
                    if message.get_type() in self.subscribers:
                        self.update_subscribers(message)
                except queue.ShutDown:
                    break

        self.source_thread = threading.Thread(target=source)
        self.source_thread.start()

    def subscribe_method(self, function, topic):
        """
        Adds the function to the dictionary of subscribers.
        """
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        self.subscribers[topic].append(function)

    def subscribe_decorator(self, topic):
        """
        This is the decorator function. Adding @subscribe(topic, message) above a function will pass it in as
        an argument to this method. Equivalent to the method above.
        """

        def wrap(function):
            if topic not in self.subscribers:
                self.subscribers[topic] = []
            self.subscribers[topic].append(function)

        return wrap

    def update_subscribers(self, message: mavlink.MAVLink_message):
        """
        As topics come in, call the registered functions.
        """
        for function in self.subscribers[message.get_type()]:
            function(message)

    def close(self):
        self.source_queue.shutdown(immediate=True)
        self.running.clear()
        self.source_thread.join()


class OwnsSubscribers:
    """
    This object owns objects and the stream they listen to.
    """

    def __init__(self, source_queue: queue.Queue):
        self.stream = Stream(source_queue)
        self.subscribe_decorator = self.stream.subscribe_decorator  # Aliases
        self.subscribe_method = self.stream.subscribe_method
        self.gps_data = GPSData(self)

    def close(self):
        self.stream.close()


class Subscriber:
    """
    This class wants to observe/subscribe to a subject/stream that publishes topics. It implements an update
    method that is called by the stream.
    """

    def __init__(self, main_object):
        self.main_object = main_object

    @classmethod
    def update(cls, message):
        return


class GPSData(Subscriber):
    """
    Keeps GPS data updated.
    """

    def __init__(self, main_object: OwnsSubscribers):
        super().__init__(main_object)

        self._latitude_deg: float = 0.0
        self._longitude_deg: float = 0.0
        self._altitude_m: float = 0.0
        self._rel_altitude_m: float = 0.0
        self._raw_data = None
        self.lock = threading.Lock()

        # To use the subscribe method, call the main_object's subscribe method on the function to be added.
        self.main_object.subscribe_method(self.update, "MAVLINK_MSG_ID_GPS_RAW_INT")

        # To use the decorator, the update function must be defined in the __init__ method of the subscriber.
        @self.main_object.subscribe_decorator("GLOBAL_POSITION_INT")
        def update_2(message: mavlink.MAVLink_global_position_int_message):
            self._latitude_deg = message.lat / 1E7
            self._longitude_deg = message.lon / 1E7
            self._altitude_m = message.alt / 1E3
            self._rel_altitude_m = message.relative_alt / 1E3

    def update(self, message: mavlink.MAVLink_gps_raw_int_message):
        self._raw_data = message.get_payload()

    @property
    def location_global(self):
        return self._latitude_deg, self._longitude_deg, self._altitude_m

    @property
    def raw_gps(self):
        return self._raw_data


if __name__ == '__main__':
    message_queue = queue.Queue()
    vehicle = OwnsSubscribers(message_queue)

    print(f"Initial GPS: {vehicle.gps_data.location_global}")
    message_queue.put(mavlink.MAVLink_global_position_int_message(8, 4, 3, 5, 2, 23, 52, 13, 3))
    time.sleep(0.01)
    print(f"After message: {vehicle.gps_data.location_global}")

    print(f"Initial Raw GPS: {vehicle.gps_data.raw_gps}")
    message_queue.put(
        mavlink.MAVLink_gps_raw_int_message(0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, ))
    time.sleep(0.01)
    print(f"After message: {vehicle.gps_data.raw_gps}")

    time.sleep(0.01)
    vehicle.close()
