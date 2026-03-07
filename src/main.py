from InvestiGator import MAVConnection
from InvestiGator import VehicleManager
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import argparse
from config import load_config
from missions import MISSIONS, MIL_MISSION_CMD, accept_mission, send_mission_complete
from threading import Event


def initialize() -> MAVConnection:

    config = load_config()

    parser = argparse.ArgumentParser(description="Companion computer script for InvestiGator UAV. Default connection is to OrangeCube+ flight controller via USB. Use -s/--sim flag to connect to SITL. Connection strings are defined in config.toml.")
    parser.add_argument("-s", "--sim", action="store_true", help="Use simulation connection string from config.toml")
    args = parser.parse_args()

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
        while True:
            if not command_event.wait(timeout=0.5):
                continue

            if not accept_mission(connection=connection, mission_number=mission_number):
                print(f"Mission {mission_number} rejected.")
                command_event.clear()
                continue
            
            success = MISSIONS[mission_number].function(vehicle)
            send_mission_complete(connection, mission_number, success=success)
            command_event.clear()

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
    
