from InvestiGator import VehicleManager
from InvestiGator import MAVConnection
from InvestiGator.constants import MIL_MISSION_CMD
from dataclasses import dataclass
from typing import Callable
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import time


@dataclass
class Mission():
    name: str
    function: Callable


MISSIONS: list[Mission] = []


def mission(name: str):
    """
    Register a mission as a function in MISSIONS.
    Decorator usage: @mission("name") above a mission function definition.
    """
    def wrap(function):
        MISSIONS.append(Mission(name, function))
        return function
    return wrap


def accept_mission(mission_number: int, connection: MAVConnection):
    """
    Validate mission number and send mavlink.COMMAND_ACK with MAV_RESULT_IN_PROGRESS to indicate acceptance.
    """
    if mission_number not in range(len(MISSIONS)):
        print(f"Invalid mission number: {mission_number} is not in mission list\n")
        return False
    
    connection.mav.command_ack_send(
        command = MIL_MISSION_CMD,
        result = mavlink.MAV_RESULT_IN_PROGRESS)
    
    print(f"Mission {mission_number}: {MISSIONS[mission_number].name} accepted.")
    return True


def send_mission_complete(connection: MAVConnection, mission_number: int, success: bool, result = None):
    """
    Send a mavlink.COMMAND_ACK message to the vehicle to indicate completion of a mission.
    """
    if result is None:
        result = mavlink.MAV_RESULT_ACCEPTED if success else mavlink.MAV_RESULT_FAILED
    
    if success:
        print(f"Mission {mission_number}: {MISSIONS[mission_number].name} completed successfully.")
        print("Waiting for new mission.")

    connection.mav.command_ack_send(
        command = MIL_MISSION_CMD,
        result = result,
        result_param2 = mission_number)


def answer_ping(vehicle: VehicleManager):
    """
    Answer ping from ground control. Ping successful if this function is called by companion computer. 
    """
    vehicle.mav_connection.mav.statustext_send(
        severity = mavlink.MAV_SEVERITY_NOTICE,
        text = "Pong".encode())
    
    return True


@mission("Aruco Landing")
def test(vehicle: VehicleManager):
    
    vehicle.camera.switch_mode("UAV Recovery")
    vehicle.set_mode(target_mode = "GUIDED")
    vehicle.arm()
    print("Taking off!")
    vehicle.takeoff(alt_m=10)

    print("Positioning for search.")
    vehicle.move_body_frd_position(forward_m=5, right_m=0, down_m=0, timeout_s=20)
    print("Positioned for search.")

    detection_gps = vehicle.search_for_detection("UAV Recovery")
    print("Search Complete")

    if detection_gps is not None:
        vehicle.center_on_marker(timeout_s=100, target_distance_m=0.30)

    print("Landing")
    return vehicle.land()

@mission("Arm")
def arm(vehicle: VehicleManager):
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    return vehicle.arm()

@mission("Square_Test")
def square_test(vehicle:VehicleManager):
    """ 
    This mission will launch the drone go in a 10x10 m square (counter clockwise) then return to launch and land: Pilot safety check - Element 1 - by Ethan Mitchell
    """
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    
    print("Guided mode set")

    if not vehicle.arm():
        return False
    time.sleep(2)
    print("Vehicle armed")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off to 3m".encode())
    
    if not vehicle.takeoff(alt_m = 3):
        vehicle.land()
        return False

    print("Vehicle at altitude = 3m")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Vehicle at altitude = 3m".encode())

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving right 5m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=5, timeout_s=10):
        print("Failed to move right, landing")
        vehicle.land()
        return False

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving forward 10m".encode())
    if not vehicle.move_body_frd_position(forward_m=10, right_m=0, timeout_s=10):
        print("Failed to move forward, landing")
        vehicle.land()
        return False
    
    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving left 10m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=-10, timeout_s=10):
        print("Failed to move left, landing")
        vehicle.land()
        return False
    
    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving back 10m".encode())
    if not vehicle.move_body_frd_position(forward_m=-10, right_m=0, timeout_s=10):
        print("Failed to move back, landing")
        vehicle.land()
        return False

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving right 5m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=5, timeout_s=10):
        print("Failed to move right, landing")
        vehicle.land()
        return False
    
    print("Movement success! Landing.")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Square success! Landing".encode())

    return vehicle.land()

@mission("Hour_Glass")
def hour_glass(vehicle:VehicleManager):
    """ 
    This mission will launch the drone go in an hour glass shape(right, diagonal forward and left, right, diagonal back and left) 10x10m then return to launch and land: Pilot safety check - Element 2 - by Ethan Mitchell
    """
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    
    print("Guided mode set")

    if not vehicle.arm():
        return False
    time.sleep(2)
    print("Vehicle armed")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off to 3m".encode())
    
    if not vehicle.takeoff(alt_m = 3):
        vehicle.land()
        return False

    print("Vehicle at altitude = 3m")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Vehicle at altitude = 3m".encode())

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving right 5m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=5, timeout_s=10):
        print("Failed to move right, landing")
        vehicle.land()
        return False

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving diagonal forward and left 10m".encode())
    if not vehicle.move_body_frd_position(forward_m=10, right_m=-10, timeout_s=10):
        print("Failed to move diagonal forward and left, landing")
        vehicle.land()
        return False
    
    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving right 10m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=10, timeout_s=10):
        print("Failed to move right, landing")
        vehicle.land()
        return False
    
    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving diagonal back and left 10m".encode())
    if not vehicle.move_body_frd_position(forward_m=-10, right_m=-10, timeout_s=10):
        print("Failed to move diagonal back and left, landing")
        vehicle.land()
        return False

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving right 5m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=5, timeout_s=10):
        print("Failed to move right, landing")
        vehicle.land()
        return False
    
    print("Movement success! Landing.")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Hour Glass success! Landing".encode())

    return vehicle.land()

@mission("Pirouette")
def pirouette(vehicle:VehicleManager):
    """ 
    This mission will launch the drone, go about 30m out, go left 3 times by 10m and perform a pirouette between each one, then return to launch- Element 3 - by Ethan Mitchell
    """
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    
    print("Guided mode set")

    if not vehicle.arm():
        return False
    time.sleep(2)
    print("Vehicle armed")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off to 3m".encode())
    
    if not vehicle.takeoff(alt_m = 3):
        vehicle.land()
        return False

    print("Vehicle at altitude = 3m")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Vehicle at altitude = 3m".encode())

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving up to point 2".encode())
    if not vehicle.move_body_frd_position(forward_m=15, right_m=15, down_m=-20, timeout_s=20):
        print("Failed to move to point 2, landing")
        vehicle.land()
        return False

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving left 15m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=-15, timeout_s=15):
        print("Failed to move left, landing")
        vehicle.land()
        return False
    
    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 1.1".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False


    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 1.2".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False


    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 2.1".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False


    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 2.2".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False


    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 3.1".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False


    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 3.2".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False

    time.sleep(2)

    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving left 15m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=-15, timeout_s=20):
        print("Failed to move left, landing")
        vehicle.land()
        return False
    
    time.sleep(2)

    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving up home".encode())
    if not vehicle.move_body_frd_position(forward_m=-15, right_m=15, down_m=20, timeout_s=20):
        print("Failed to move home, landing")
        vehicle.land()
        return False

    time.sleep(2)

    print("Movement success! Landing.")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette success! Landing".encode())

    return vehicle.land()

@mission("Test_down_yaw")
def test_down_yaw(vehicle:VehicleManager):
    
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    
    print("Guided mode set")

    if not vehicle.arm():
        return False
    time.sleep(2)
    print("Vehicle armed")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off to 3m".encode())
    
    if not vehicle.takeoff(alt_m = 3):
        vehicle.land()
        return False

    print("Vehicle at altitude = 3m")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Vehicle at altitude = 3m".encode())

    time.sleep(2)
    """
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Test down".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, down_m=-2, timeout_s=10):
        print("Failed to test down, landing")
        vehicle.land()
        return False

    time.sleep(2)
    """

    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Test yaw".encode())
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 1.1".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False


    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 1.2".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False


    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 2.1".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False


    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Pirouette 2.2".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=0, yaw_rate=3.14, maintain_heading=False, timeout_s=3.14):
        print("Failed to pirouette, landing")
        vehicle.land()
        return False

    
    time.sleep(2)
    
    
    print("Movement success! Landing.")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Test down yaw! Landing".encode())

    return vehicle.land()

@mission("Wait for cancel/abort")
def wait_for_cancel(vehicle: VehicleManager):
    print("Waiting for cancel or abort command...")
    while True:
        if vehicle.uncontrolled_event.is_set():
            print("Abort command received.")
            break
        elif vehicle.cancel_mission_event.is_set():
            print("Cancel command received.")
            return True
        time.sleep(0.1)

    while True:
        if not vehicle.uncontrolled_event.is_set():
            print("Abort cleared")
            return True
        time.sleep(0.1) 

@mission("Ups and Downs")
def ups_and_downs(vehicle:VehicleManager):
    """ 
    This mission will launch the drone and land - by Ethan Mitchell
    """
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    
    print("Guided mode set")

    if not vehicle.arm():
        return False
    time.sleep(2)
    print("Vehicle armed")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off to 10m".encode())
    
    if not vehicle.takeoff(alt_m = 10):
        vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "I died :(".encode())
        vehicle.land()
        return False
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off good :)\nmoving to sleep honkshoo".encode())
    time.sleep(2)

    return vehicle.land()

@mission("rtl_batt_test")
def rtl_batt_test(vehicle:VehicleManager):
    """
    This mission will launch the drone and hold in the air until 21.6V and return to launch
    """

    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False

    print("Guided mode set")

    if not vehicle.arm():
        return False

    time.sleep(2)
    print("Vehicle armed")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off to 5m".encode())

    time_start = time.monotonic()
    start_voltage = vehicle.status.voltage_battery

    if not vehicle.takeoff(alt_m = 5):
        vehicle.mav.statustext_send(mavlink.MAV_SEVERRITY_INFO, "I died :(".encode())
        vehicle.land()
        return False
    vehicle.intended_rtl_land = True
    while True:
        if vehicle.check_mode == "RTL":
            duration = time_start - time.monotonic()
            end_voltage = vehicle.status.volatge_battery
            print(f"RTL detected. Start voltage: {start_voltage} End voltage: {end_voltage}")
            print(f"Time taken: {duration}")
            vehicle.statustext_send("RTL detected. Start voltage: {start_voltage} End voltage: {end_voltage}\nTime taken: {duration}")
            return True

    return False
