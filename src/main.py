from InvestiGator import MAVConnection
from InvestiGator import VehicleManager
from InvestiGator import constants
from InvestiGator.constants import MIL_MISSION_CMD, MIL_STATE_MISSION
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import argparse
from config import load_config
from missions import MISSIONS_BY_NAME, accept_mission, send_mission_complete
from gc_helpers import valid_mission, MISSION_MENU
from threading import Event, Thread
import time

interactive = False
sim = False

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
        global sim
        sim = True
        address = config["simulation"].get("companion_computer")
    else:
        address = config["hardware"].get("flight_controller")

    baud = config["hardware"].get("flight_controller_baud")

    print(f"Connecting with address: {address}")
    connection = MAVConnection(address, mav_type=mavlink.MAV_TYPE_ONBOARD_CONTROLLER, source_system=1, source_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER, baud=baud)

    return connection

def self_arm(vehicle: VehicleManager):
    """
    Once the OCS's fence is live, arm in GUIDED on the pad so the drone reports STATE_AUTO for RunStart.
    Runs once, in its own thread. Never writes OA_TYPE: without the planner it reports not ready and stays disarmed.
    """
    vehicle.fence_live.wait()

    # OA_TYPE = 2 is the path planner, needed to route around keep-out circles.
    while (oa_type := vehicle.get_parameter("OA_TYPE")) != 2:
        if oa_type is not None:
            print(f"OA_TYPE is {oa_type:g}. Not arming: set parameter OA_TYPE = 2 and reboot the drone; "
                  "this is NECESSARY for pathfinding on Task 4 - Advanced tier.")
            vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_WARNING, constants.STATUS_NOT_READY_OA_TYPE.encode())
            vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_WARNING, constants.STATUS_NOT_READY_OA_TYPE_WHY.encode())
        time.sleep(constants.NOT_READY_REPORT_S)

    # Auto-disarm applies in GUIDED, and the drone waits armed on the pad for RunStart.
    while not vehicle.set_parameter("DISARM_DELAY", 0):
        time.sleep(1)

    # arm() can return False even when armed, so check the heartbeat. Pre-arm fails during GPS warm-up.
    while not vehicle.status.armed:
        if vehicle.set_mode("GUIDED"):
            vehicle.arm()
        time.sleep(1)
    print("Armed in GUIDED on the pad.")

def main():

    # Make connection
    connection = initialize()
    print("Connection made!")

    vehicle = VehicleManager(mav_connection=connection, sim=sim)
    Thread(name="Self-arm Thread", target=self_arm, args=(vehicle,), daemon=True).start()

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

                if vehicle.incident_active():
                    print("Responding to an incident. Rejecting mission.")
                    send_mission_complete(connection, mission_number, success=False, result = mavlink.MAV_RESULT_DENIED)
                    command_event.clear()
                    continue

                if not accept_mission(connection=connection, mission_number=mission_number):
                    print(f"Mission {mission_number} rejected.")
                    command_event.clear()
                    continue
                
                mission_id = constants.MISSION_NUMBERS_REVERSE[mission_number]
                try:
                    connection.custom_mode = MISSIONS_BY_NAME[mission_id].rx_task
                    connection.system_status = MIL_STATE_MISSION
                    success = MISSIONS_BY_NAME[mission_id].function(vehicle)
                    send_mission_complete(connection, mission_number, success=success)
                finally:
                    connection.custom_mode = constants.RX_TASK_NONE

                vehicle.reset_state()
                command_event.clear()

        else:
            while True:
                print(MISSION_MENU)
                mission_number = input("Enter mission number: ")

                if not valid_mission(mission_number):
                    continue

                mission_number = int(mission_number)
                mission_id = constants.MISSION_NUMBERS_REVERSE[mission_number]
                try:
                    connection.custom_mode = MISSIONS_BY_NAME[mission_id].rx_task
                    success = MISSIONS_BY_NAME[mission_id].function(vehicle)
                finally:
                    connection.custom_mode = constants.RX_TASK_NONE
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
    
