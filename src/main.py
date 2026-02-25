from mavconnection import MAVConnection
from vehicle import VehicleManager
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from constants import Radio, Robot
from dataclasses import dataclass
import time

def test(vehicle: VehicleManager):
    
    vehicle.camera.switch_mode("UAV Recovery")
    vehicle.set_mode(target_mode="GUIDED")
    vehicle.wait_for_armed()
    vehicle.takeoff(alt_m=10)

    vehicle.move_body_frd_position(forward_m=5, right_m=4, down_m=-10, timeout_s=20)
    print("Positioned for search.")
    detection_gps = vehicle.search_for_detection("UAV Recovery")
    print("Search Complete")
    if detection_gps is not None:
        vehicle.center_on_marker(timeout_s=100, target_distance_m=0.25)

    vehicle.set_mode("LAND")
    print("DONE")

    vehicle.close()

@dataclass
class MissionState:
    mission_selection: int = 0

if __name__ == "__main__":

    # Make connection
    connection = MAVConnection(Radio.SIMULATION, source_system=Robot.INVESTIGATOR)
    vehicle = VehicleManager(mav_connection=connection)

    # Instantiate mission state to detect when a mission message is received
    mission_state = MissionState()

    @vehicle.subscribe(mavlink.MAVLINK_MSG_ID_NAMED_VALUE_INT)
    def handle_named_value_int(message):
        if message.name.decode('utf-8') == "INVGATOR":
            print(f"STATUSTEXT: {message.text}")
            if message.value == 1:
                mission_state.mission_selection = 1
            elif message.value == 2:
                mission_state.mission_selection = 2

    print("Listening for mission selection.")
    while True:
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
