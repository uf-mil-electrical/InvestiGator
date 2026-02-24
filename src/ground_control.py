from mavconnection import MAVConnection
from enum import Enum
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import platform
import sys

System = Enum('System', [('INVESTIGATOR', 37), ('ROVER', 44), ('NAVIGATOR', 47), ('SUBJUGATOR', 59), ('GROUND_CONTROL', 255)])

SIMULATION_RADIO = 'udpin:0.0.0.0:14550' 
GROUND_CONTROL_RADIO_LINUX = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0"

def initialize() -> MAVConnection:
    """
    Prompt user for system connection mode. The connection is made to a simulated radio or hardware radio connected via USB. 
    If on Windows, user is prompted for the relevant COM port.
    """
    print("--- Select Radio Mode ---")
    print("1) Hardware Mode (USB)")
    print("2) Simulation Mode (UDP)")

    mode = input("Select mode [1-2]: ")

    if mode == '1':
        if platform.system == 'Linux':
            connection = MAVConnection(GROUND_CONTROL_RADIO_LINUX)

        elif platform.system == 'Windows':
            com_port = input("Enter COM Port for Radio (COMx): ")
            connection = MAVConnection(com_port)
        
        else:
            print("Platform not supported. Please use Windows or Linux.")
            sys.exit(1)

    elif mode == '2':
        connection = MAVConnection(SIMULATION_RADIO)

    return connection

def mission_select():
    """
    Prompt user for mission selection from list of known missions.
    """
    print("--- Select Mission ---")


if __name__ == "__main__":

    connection = initialize()
    print("Connection made!")

    