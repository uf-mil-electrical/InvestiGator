from InvestiGator import MAVConnection
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from config import load_config

def initialize() -> MAVConnection:
    """
    Prompt user for system connection mode. The connection is made to a simulated radio or hardware radio connected via USB. 
    Strings for connection are stored in InvestiGator/config.toml.
    """

    config: dict = load_config()

    print("--- Select Radio Mode ---")
    print("1) Hardware Mode (USB)")
    print("2) Simulation Mode (UDP)")

    mode = input("Select mode [1-2]: ")

    address = None

    while True:
        if mode == '1':
            address = config["hardware"].get("ground_control")
            break

        elif mode == '2':
            address = config["simulation"].get("ground_control")
            break
        
        else:
            print("Invalid mode selected. Please select 1 or 2.")
            mode = input("Select mode [1-2]: ")


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

    