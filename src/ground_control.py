from InvestiGator import MAVConnection
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from config import load_config
import argparse
from missions import MISSIONS, MISSION_MENU, send_mission_message_wait_ack, wait_for_mission_complete
import time


def initialize() -> MAVConnection:
    """
    Get MAVLink connection. By default, connection is made to hardware RFD900x radio modem. Simulation can be selected with -s/--sim flag. 
    Strings for connection are stored in InvestiGator/config.toml.
    """

    config: dict = load_config()

    parser = argparse.ArgumentParser(description="Ground Control script for InvestiGator UAV. Default connection is to RFD900x radio modem. Use -s/--sim to connect to SITL. Connection strings are defined in config.toml.")
    parser.add_argument("-s", "--sim", action="store_true", help="Use simulation connection string from config.toml")
    args = parser.parse_args()

    if args.sim:
        address = config["simulation"].get("ground_control")
    else:
        address = config["hardware"].get("ground_control")
    
    baud = config["hardware"].get("ground_control_baud")

    print(f"Connecting with address: {address}")
    connection = MAVConnection(address, source_system=254, baud=baud)

    return connection

def valid_mission(mission_number: str) -> bool:
    """
    Check if mission number is valid.
    """
    if not mission_number.isdigit():
        print(f"Invalid mission number: {mission_number}\n")
        return False

    mission_index = int(mission_number)
    if mission_index < 0 or mission_index >= len(MISSIONS):
        print(f"Invalid mission number: {mission_number}\n")
        return False
    
    return True

def main():

    connection = initialize()
    print("Connection made!\n")

    try:
        while True:
            print(MISSION_MENU)
            mission_number = input("Enter mission number: ")
            
            if not valid_mission(mission_number):
                continue
            
            mission_number = int(mission_number)
            start_s = time.monotonic()

            if not send_mission_message_wait_ack(connection, mission_number):
                print("Mission failed to be acknowledged.")
                continue

            if not wait_for_mission_complete(connection, mission_number):
                print("Mission failed to complete.")
                continue

            print(f"Mission {mission_number}: {MISSIONS[mission_number].name} completed successfully in {time.monotonic()- start_s:.2f} seconds.\n")

    finally:
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
        print (e)

    