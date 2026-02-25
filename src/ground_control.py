from mavconnection import MAVConnection
from enum import Enum
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import platform
import sys

System = Enum('System', [('INVESTIGATOR', 37), ('ROVER', 44), ('NAVIGATOR', 47), ('SUBJUGATOR', 59), ('GROUND_CONTROL', 255)])

SIMULATION_RADIO = 'udpin:10.0.0.207:14550' 
GROUND_CONTROL_RADIO_LINUX = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTDCKG37-if00-port0"

INVGATOR = "InvGator".encode('utf-8')

def initialize() -> MAVConnection:
    """
    Prompt user for system connection mode. The connection is made to a simulated radio or hardware radio connected via USB. 
    If on Windows, user is prompted for the relevant COM port.
    """
    print("--- Select Radio Mode ---")
    print("1) Hardware Mode (USB)")
    print("2) Simulation Mode (UDP)")

    mode = input("Select mode [1-2]: ")

    address = None

    if mode == '1':
        if platform.system == 'Linux':
            address = GROUND_CONTROL_RADIO_LINUX

        elif platform.system == 'Windows':
            address = input("Enter COM Port for Radio (COMx): ")
        
        else:
            print("Platform not supported. Please use Windows or Linux.")
            sys.exit(1)

    elif mode == '2':
        address = SIMULATION_RADIO

    connection = MAVConnection(address, source_system=System.GROUND_CONTROL.value)

    return connection

def mission_select() -> str:
    """
    Prompt user for mission selection from list of known missions.
    """
    print("--- Select Mission ---")
    print("1) Mission 1: UAV Recovery")
    print("2) Mission 2: Close Drone Connection")

    return input("Select mission [1-2]: ")

if __name__ == "__main__":

    connection = initialize()
    print("Connection made!")

    mission_selection = mission_select()

    while True:
        if mission_selection == None:
            mission_selection = mission_select()
            
        if mission_selection == '1':
            print("Starting Mission 1")
            connection.mav_connection.mav.named_value_int_send(
                time_boot_ms = 0, 
                name = INVGATOR, 
                value = 1)
            mission_selection = None

        elif mission_selection == '2':
            print("Starting Mission 2. Closing connection.")
            connection.mav_connection.mav.named_value_int_send(
                time_boot_ms = 0, 
                name = INVGATOR, 
                value = 2)
            break

    connection.close()

    