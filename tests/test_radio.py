''' This file shows a simple way to set up a serial device with pyserial using threads for listening and reading. '''

import serial
import threading
import queue
import sys

def serial_sender(q):
    while not stop_event.is_set():
        try:
            message = q.get(timeout=1)
            if (message is not None):
                if (message == KeyboardInterrupt):
                    break
                message += "\n"
                ser.write(message.encode('utf-8'))
        except queue.Empty:
            continue


def serial_listener():
    while not stop_event.is_set():
        try:
            message = ser.read_until()
            if (len(message) > 0):
                sys.stdout.write("\r\033[K")
                sys.stdout.write("Message received: {}\n".format(message.decode('utf-8')))
                sys.stdout.write("Enter string to send: ")
                sys.stdout.flush() 
        except serial.SerialException:
            continue


# Create a serial object that connects to the radio on the Pi
port = '/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0'
baudrate = 57600
ser = serial.Serial(port, baudrate, timeout=1)

send_q = queue.Queue()
stop_event = threading.Event()

sender = threading.Thread(target=serial_sender, args=(send_q,))
listener = threading.Thread(target=serial_listener)

sender.start()
listener.start()

while True:
    try:
        sys.stdout.write("Enter string to send: ")
        sys.stdout.flush()
        message = input()
        send_q.put(message)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt. Exiting program.")
        stop_event.set()

        sender.join(timeout=1)
        listener.join(timeout=1)
        ser.close()

        if sender.is_alive():
            print("Sender thread did not finish in time, terminating.")
        if listener.is_alive():
            print("Listener thread did not finish in time, terminating.")

        break
    
print("Program exited.")