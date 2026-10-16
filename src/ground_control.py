from InvestiGator import MAVConnection
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from config import load_config
import argparse
from InvestiGator.constants import MIL_SYSTEM_CMD
import time
from gc_helpers import valid_mission, wait_for_mission_complete, send_mission_message_wait_ack, MISSIONS_BY_NUMBER, MISSION_MENU, send_system_command
from textual_ui import MissionControl


def parse_args():
    """
    Get arguments from argparse.
    Simulation can be selected with -s/--sim flag.
    No_GUI mode can be selected with --no_gui flag.
    """
    parser = argparse.ArgumentParser(description="Ground Control script for InvestiGator UAV. Default connection is to RFD900x radio modem. Use -s/--sim to connect to SITL. Connection strings are defined in config.toml.")
    
    parser.add_argument("-s", "--sim", action="store_true", help="Use simulation connection string from config.toml")
    parser.add_argument("--no_gui", action="store_true", help="Run in CLI mode.")
    
    return parser.parse_args()


def create_connection(args) -> MAVConnection:
    """
    Get MAVLink connection. By default, connection is made to hardware RFD900x radio modem.  
    Strings for connection are stored in InvestiGator/config.toml.
    """
    config: dict = load_config()

    if args.sim:
        address = config["simulation"].get("ground_control")
    else:
        address = config["hardware"].get("ground_control")
    
    baud = config["hardware"].get("ground_control_baud")

    print(f"Connecting with address: {address}")
    connection = MAVConnection(address, mav_type=mavlink.MAV_TYPE_GCS, source_system=254, baud=baud)

    return connection


def run_gui(connection: MAVConnection):
    """
    Run Textual GUI.
    """
    app = MissionControl(connection)
    app.run()


def run_cli(connection: MAVConnection):
    """
    Run CLI prompt for running missions and communicating with InvestiGator UAV.
    """
    while True:
        print(MISSION_MENU)
        mission_number = input("Enter selection: ")

        if mission_number in ("p", "g", "u", "c"):
            send_system_command(connection, mission_number)
            continue

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

        print(f"Mission {mission_number}: {MISSIONS_BY_NUMBER[mission_number].name} completed successfully in {time.monotonic()- start_s:.2f} seconds.\n")
    

def main():

    args = parse_args()
    connection = create_connection(args)

    try:
        if args.no_gui:
            run_cli(connection)
        else:
            run_gui(connection)

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

    