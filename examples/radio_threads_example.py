""" Same concept as test_radio.py, except implements the serial device through a class. """

import serial
import threading
import queue
import sys

port = '/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0'

class SerialComm:

    def __init__(self, address, baud=57600, timeout=1):
        self.connection = serial.Serial(address, baud, timeout=timeout)
        self.send_q = queue.Queue()
        self.stop_event = threading.Event()
        self.sender = threading.Thread(name="Sender", target=self.serial_sender, args=(self.send_q,))
        self.reader = threading.Thread(name="Reader", target=self.serial_listener)

    def stop_threads(self):
        if self.sender is not None:
            self.sender.join(timeout=1)
            if self.sender.is_alive():
                print("Sender thread join timed out.")
            self.sender = None
        if self.reader is not None:
            self.reader.join(timeout=1)
            if self.reader.is_alive():
                print("Reader thread join timed out.")
            self.reader = None

    def serial_sender(self, q):
        while not self.stop_event.is_set():
            try:
                message = q.get(timeout=1)
                if message is not None:
                    if message == KeyboardInterrupt:
                        break
                    message += "\n"
                    self.connection.write(message.encode('utf-8'))
            except queue.Empty:
                continue

    def serial_listener(self):
        while not self.stop_event.is_set():
            try:
                message = self.connection.read_until()
                if len(message) > 0:
                    sys.stdout.write("\r\033[K")
                    sys.stdout.write("Message received: {}\n".format(message.decode('utf-8')))
                    sys.stdout.write("Enter string to send: ")
                    sys.stdout.flush() 
            except serial.SerialException:
                continue

    def write(self, message):
        self.send_q.put(message)

    def close(self):
        self.stop_event.set()
        self.stop_threads()
        self.connection.close()

connection = SerialComm(port)

while True:
    try:
        sys.stdout.write("Enter string to send: ")
        message_to_send = input()
        connection.write(message_to_send)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt. Exiting program.")
        connection.close()
        break

print("Program exited.")