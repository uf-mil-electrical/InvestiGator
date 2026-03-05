from dataclasses import dataclass
from queue import Queue, ShutDown
from threading import Thread, Event, Lock
from time import monotonic, sleep
from typing import Callable

from pymavlink.dialects.v20 import ardupilotmega as mavlink


class SubscriptionManager:
    """
    Manages and notifies subscribers when a message is published.
    """

    def __init__(self, receive_queue: Queue):
        self.message_subscribers = {}
        self.receive_queue = receive_queue
        self.running = Event()
        self.running.set()
        self.lock = Lock()

        def reader():
            while self.running.is_set():
                try:
                    message: mavlink.MAVLink_message = receive_queue.get(block=True)
                    if message.get_type() in self.message_subscribers:
                        self.update_subscribers(message)
                except ShutDown:
                    break

        self.reader_thread = Thread(target=reader, name="SubManager Thread")
        self.reader_thread.start()

    def subscribe(self, message_type):
        """
        Decorator: Add a function to the mailing list for message_type or attribute. 
        Usage: Decorate with @__class__.__name__.subscribe('message_type') to register a function.
        """

        def wrap(function):
            with self.lock:
                if message_type not in self.message_subscribers:
                    self.message_subscribers[message_type] = []
                self.message_subscribers[message_type].append(function)
            return function

        return wrap

    def update_subscribers(self, message: mavlink.MAVLink_message):
        """
        Call functions that have a subscription to message_type.
        """
        with self.lock:
            subscribers = self.message_subscribers.get(message.get_type(), []).copy()
        for function in subscribers:
            function(message)

    def close(self):
        self.receive_queue.shutdown(immediate=True)
        self.running.clear()
        self.reader_thread.join()


@dataclass
class PublishingInfo:
    function: Callable
    period: float
    last_published: float


class PublicationManager:
    """
    Manages functions that publish at a given interval.
    """

    def __init__(self, send_queue: Queue):
        self.publishing = {}
        self.send_queue = send_queue
        self.running = Event()
        self.running.set()

        def publication_thread():
            while self.running.is_set():
                for publisher in self.publishing.values():
                    if (publisher["last_published"] is None) or (monotonic() - publisher["last_published"]) >= (
                            1 / publisher["frequency"]):
                        publisher["function"]()
                        publisher["last_published"] = monotonic()
                sleep(0.01)

        self.publish_thread = Thread(target=publication_thread, name="Publication Thread", daemon=True)
        self.publish_thread.start()

    def publish(self, message_name: str, frequency: float):
        """
        Decorator: Registers a function to be published at given frequency.
        """
        if frequency > 50:
            print("Frequency too large. Please choose a frequency less than or equal to 50Hz.")

        def wrap(function):
            if message_name in self.publishing:
                print("This function is already registered.")
            elif frequency <= 50:
                self.publishing[message_name] = {"function": function, "frequency": frequency, "last_published": None}
            return function

        return wrap

    def register_publisher(self, message_name, frequency, function):
        """
        Registers a function to be published at given frequency.
        """
        if frequency > 50:
            print("Frequency too large. Please choose a frequency less than or equal to 50.")
            return

        if message_name in self.publishing:
            print("This function is already registered.")
        else:
            self.publishing[message_name] = {"function": function, "frequency": frequency, "last_published": None}

    def remove_publisher(self, message_name):
        """
        Removes a function from publishing list.
        """
        if message_name in self.publishing:
            del self.publishing[message_name]
        else:
            print("Function not a publisher.")

    def close(self):
        self.running.clear()
        self.publish_thread.join()
