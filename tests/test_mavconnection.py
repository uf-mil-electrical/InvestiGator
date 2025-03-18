''' File for testing sending and receiving mavlink messages to flight controller with pymavlink '''

from pymavlink import mavutil
import threading
import queue

port = '/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0'
simulation = 'udp:127.0.0.1:14550'

class MAVConnection:

    def __init__(port):

        connection = mavutil.mavlink_connection(port)
        connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))