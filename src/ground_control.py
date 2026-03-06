from InvestiGator import MAVConnection
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from config import load_config
import argparse

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

    print(f"Connecting with address: {address}")
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
    except FileNotFoundError as e:
        print(e)
    except Exception as e:
        print (e)

    