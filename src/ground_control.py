from re import I

from InvestiGator import MAVConnection
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import platform
import sys

SIMULATION_RADIO = 'udpin:127.0.0.1:14552' 
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

    address = None

    if mode == '1':
        if platform.system() == 'Linux':
            address = GROUND_CONTROL_RADIO_LINUX

        elif platform.system() == 'Windows':
            address = input("Enter COM Port for Radio (COMx): ")
        
        else:
            print("Platform not supported. Please use Windows or Linux.")
            sys.exit(1)

    elif mode == '2':
        address = "udpin:127.0.0.1:14551"

    connection = MAVConnection(address, source_system=254)

    return connection

def mission_select() -> str:
    """
    Prompt user for mission selection from list of known missions.
    """
    print("--- Select Mission ---")
    print("1) Mission 1: UAV Recovery")
    print("2) Mission 2: Close Drone Connection")

    return input("Select mission [1-2]: ")

def main():

    connection = initialize()
    print("Connection made!")

    try:
        mission_selection = mission_select()

        while True:
            if mission_selection == None:
                mission_selection = mission_select()

            if mission_selection == '1':
                print("Starting Mission 1")
                connection.mav_connection.mav.command_long_send(
                    target_system = 1,
                    target_component = 0,
                    command = mavlink.MAV_CMD_USER_1,
                    confirmation = 0,
                    param1 = 1,
                    param2 = 0,
                    param3 = 0,
                    param4 = 0,
                    param5 = 0,
                    param6 = 0,
                    param7 = 0)
                mission_selection = None

            elif mission_selection == '2':
                print("Starting Mission 2. Closing connection.")
                connection.mav_connection.mav.command_long_send(
                    target_system = 1,
                    target_component = 0,
                    command = mavlink.MAV_CMD_USER_1,
                    confirmation = 0,
                    param1 = 2,
                    param2 = 0,
                    param3 = 0,
                    param4 = 0,
                    param5 = 0,
                    param6 = 0,
                    param7 = 0)
                break
        
    finally:
        connection.close()    

if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting.")
    except TimeoutError as e:
        print(e)

    