from InvestiGator import MAVConnection
from InvestiGator import VehicleManager
from InvestiGator.constants import MIL_MISSION_ABORT, MIL_MISSION_CANCEL, MIL_MISSION_CMD
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import argparse
from config import load_config
from missions import MISSIONS, MISSION_MENU, accept_mission, send_mission_complete, valid_mission
from threading import Event

interactive = False

def initialize() -> MAVConnection:

    config = load_config()

    parser = argparse.ArgumentParser(description="Companion computer script for InvestiGator UAV. Default connection is to OrangeCube+ flight controller via USB. Use -s/--sim flag to connect to SITL. Connection strings are defined in config.toml.")
    parser.add_argument("-s", "--sim", action="store_true", help="Use simulation connection string from config.toml")
    parser.add_argument("-i", "--interactive", action="store_true", help="Use interactive mode to select missions from command line.")
    args = parser.parse_args()

    if args.interactive:
        global interactive
        interactive = True

    if args.sim:
        address = config["simulation"].get("companion_computer")
    else:
        address = config["hardware"].get("flight_controller")

    baud = config["hardware"].get("flight_controller_baud")

    print(f"Connecting with address: {address}")
    connection = MAVConnection(address, source_system=1, source_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER, baud=baud)

    return connection

def main():

    # Make connection
    connection = initialize()
    print("Connection made!")

    vehicle = VehicleManager(mav_connection=connection)

    command_event = Event()
    mission_number = -1

    @vehicle.subscribe(mavlink.MAVLink_command_long_message.msgname)
    def handle_command_int(message):
        if message.command == MIL_MISSION_CMD and not command_event.is_set():
            nonlocal mission_number
            mission_number = int(message.param1)
            command_event.set() 

    try:
        if not interactive:
            while True:
                if not command_event.wait(timeout=0.5):
                    continue
                
                if vehicle.uncontrolled_event.is_set():
                    print("Vehicle is in uncontrolled state. Rejecting mission. Change mode to GUIDED to clear.")
                    send_mission_complete(connection, mission_number, success=False, result = mavlink.MAV_RESULT_DENIED)
                    command_event.clear()
                    continue

                if not accept_mission(connection=connection, mission_number=mission_number):
                    print(f"Mission {mission_number} rejected.")
                    command_event.clear()
                    continue
                
                success = MISSIONS[mission_number].function(vehicle)
                send_mission_complete(connection, mission_number, success=success)

                vehicle.reset_state()
                command_event.clear()
        
        else:
            while True:
                print(MISSION_MENU)
                mission_number = input("Enter mission number: ")

                if not valid_mission(mission_number):
                    continue

                mission_number = int(mission_number)
                success = MISSIONS[mission_number].function(vehicle)
                print(f"Mission {mission_number} {'succeeded' if success else 'failed'}.\n")

    finally:
        print("Closing connections.")
        vehicle.close()
        connection.close()

if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting.")
    except TimeoutError as e:
        print(e)
    except FileNotFoundError as e:
        print(e)
    except KeyError as e:
        print(e)
    
