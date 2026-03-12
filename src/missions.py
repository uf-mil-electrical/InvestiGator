from InvestiGator import VehicleManager
from InvestiGator import MAVConnection
from InvestiGator.constants import MIL_MISSION_CMD
from dataclasses import dataclass
from typing import Callable
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from threading import Event
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


def send_mission_complete(connection: MAVConnection, mission_number: int, success: bool = True, result = None):
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


def send_mission_message_wait_ack(connection: MAVConnection, mission_number: int):
    """
    Send a mavlink.COMMAND_LONG message to the vehicle to request a mission.
    """
    ack_event = Event()
    ack_result = None

    @connection.subscribe(mavlink.MAVLink_command_ack_message.msgname)
    def on_ack(message: mavlink.MAVLink_command_ack_message):
        nonlocal ack_result
        if message.command == MIL_MISSION_CMD:
            ack_result = message.result
            ack_event.set()

    for attempt in range(3):
        ack_event.clear()
        ack_result = None

        print(f"Sending mission command, attempt {attempt + 1}")

        connection.mav.command_long_send(
            target_system = 1,
            target_component = mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
            command = MIL_MISSION_CMD,
            confirmation = attempt,
            param1 = mission_number,
            param2 = 0,
            param3 = 0,
            param4 = 0,
            param5 = 0,
            param6 = 0,
            param7 = 0)
        
        if ack_event.wait(timeout=3):
            connection.sub_manager.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
            return ack_result == mavlink.MAV_RESULT_IN_PROGRESS
    
    connection.sub_manager.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
    return False


def wait_for_mission_complete(connection: MAVConnection, mission_number: int):
    """
    Wait for mavlink.COMMAND_ACK message indicating completion of the mission.
    Interrupted by loss of connection. 
    """
    ack_event = Event()
    ack_result = None
    ack_mission_int = None

    @connection.subscribe(mavlink.MAVLink_command_ack_message.msgname)
    def on_ack(message: mavlink.MAVLink_command_ack_message):
        nonlocal ack_result, ack_mission_int
        if message.command == MIL_MISSION_CMD:
            ack_result = message.result
            ack_mission_int = message.result_param2
            ack_event.set()

    # TODO: Heartbeat or user abort timeout
    while True:
        if ack_event.wait(timeout=0.5):
            if ack_mission_int == mission_number:
                connection.sub_manager.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
                return ack_result == mavlink.MAV_RESULT_ACCEPTED
            
            ack_event.clear()


def build_mission_menu() -> str:
    """
    Return a string menu of available missions.
    """
    menu = "--- Select Mission ---\n"
    for i, mission in enumerate(MISSIONS):
        menu += f"{i}) {mission.name}\n"
    menu += "--- System Commands ---\n"
    menu += "p) Ping Companion Computer\n"
    menu += "g) Set mode to GUIDED\n"
    menu += "u) Set uncontrolled (for testing abort)\n"
    menu += "c) Cancel mission (for testing cancel)\n"
    return menu


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
    
    if not vehicle.takeoff(alt_m = 10):
        vehicle.land()
        return False

    print("Vehicle at altitude = 10m")

    time.sleep(2)
    
    if not vehicle.move_body_frd_position(forward_m=2, right_m=0, timeout_s=10):
        print("Failed to move forward, landing")
        vehicle.land()
        return False
    
    if not vehicle.move_body_frd_position(forward_m=0, right_m=2, timeout_s=10):
        print("Failed to move right, landing")
        vehicle.land()
        return False
    
    if not vehicle.move_body_frd_position(forward_m=-2, right_m=0, timeout_s=10):
        print("Failed to move backward, landing")
        vehicle.land()
        return False
    
    if not vehicle.move_body_frd_position(forward_m=0, right_m=-2, timeout_s=10):
        print("Failed to move left, landing")
        vehicle.land()
        return False
    
    print("Movement success! Landing.")

    return vehicle.land()

# This must be called at the end of this file after MISSIONS list is populated by mission decorators.
MISSION_MENU = build_mission_menu()