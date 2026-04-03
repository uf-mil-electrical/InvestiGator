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
    This mission will launch the drone go in a 2x2 m square (clockwise) then return to launch and land - by Ethan Mitchell
    """
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    
    print("Guided mode set")

    if not vehicle.arm():
        return False

    print("Vehicle armed")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off to 10m".encode())
    
    if not vehicle.takeoff(alt_m = 10):
        vehicle.land()
        return False

    print("Vehicle at altitude = 10m")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Vehicle at altitude = 10m".encode())

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving forward 2m".encode())
    if not vehicle.move_body_frd_position(forward_m=2, right_m=0, timeout_s=10):
        print("Failed to move forward, landing")
        vehicle.land()
        return False

    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving right 2m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=2, timeout_s=10):
        print("Failed to move right, landing")
        vehicle.land()
        return False
    
    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving back 2m".encode())
    if not vehicle.move_body_frd_position(forward_m=-2, right_m=0, timeout_s=10):
        print("Failed to move backward, landing")
        vehicle.land()
        return False
    
    time.sleep(2)
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Moving left 2m".encode())
    if not vehicle.move_body_frd_position(forward_m=0, right_m=-2, timeout_s=10):
        print("Failed to move left, landing")
        vehicle.land()
        return False
    
    print("Movement success! Landing.")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Square success! Landing".encode())

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
def square_test(vehicle:VehicleManager):
    """ 
    This mission will launch the drone and land - by Ethan Mitchell
    """
    if not vehicle.set_mode(target_mode = "GUIDED"):
        return False
    
    print("Guided mode set")

    if not vehicle.arm():
        return False

    print("Vehicle armed")
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off to 10m".encode())
    
    if not vehicle.takeoff(alt_m = 10):
        vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "I died :(".encode())
        vehicle.land()
        return False
    
    vehicle.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, "Taking off good :)\nmoving to sleep honkshoo".encode())
    time.sleep(2)

    return vehicle.land()

