from InvestiGator import MAVConnection
from InvestiGator import VehicleManager
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from dataclasses import dataclass
import time
from pymavlink import mavutil

def test(vehicle: VehicleManager):
    
    #vehicle.camera.switch_mode("UAV Recovery")
    vehicle.set_mode(target_mode = "GUIDED")
    vehicle.arm()
    vehicle.takeoff(alt_m=10)

    vehicle.move_body_frd_position(forward_m=5, right_m=4, down_m=-10, timeout_s=20)
    print("Positioned for search.")
    #detection_gps = vehicle.search_for_detection("UAV Recovery")
    #print("Search Complete")
    #if detection_gps is not None:
        #vehicle.center_on_marker(timeout_s=100, target_distance_m=0.25)

    vehicle.set_mode(target_mode = "LAND")
    print("DONE")

    #vehicle.close()

@dataclass
class MissionState:
    mission_selection: int = 0

def main():

    # Make connection
    connection = MAVConnection("udpin:127.0.0.1:14552", source_system=1, source_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER )
    vehicle = VehicleManager(mav_connection=connection)

    try:
        # Instantiate mission state to detect when a mission message is received
        mission_state = MissionState()

        @vehicle.subscribe(mavlink.MAVLink_command_long_message.msgname)
        def handle_command_int(message):
            if message.command == mavlink.MAV_CMD_USER_1:
                print("Mission Selection Message Received.")
                if message.param1 == 1:
                    mission_state.mission_selection = 1
                elif message.param1 == 2:
                    mission_state.mission_selection = 2

        while True:
            if mission_state.mission_selection == 0:
                print("Listening for mission selection.")
                mission_state.mission_selection = -1

            if mission_state.mission_selection == 1:
                print("Starting Mission 1")
                test(vehicle)
                mission_state.mission_selection = 0
                print("Mission 1 done. Listening for new mission selection.")

            elif mission_state.mission_selection == 2:
                print("Starting Mission 2. Closing connection.")
                vehicle.close()
                connection.close()
                mission_state.mission_selection = 0
                break
            
            time.sleep(0.1)

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
    
