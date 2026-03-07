from InvestiGator import VehicleManager
from InvestiGator import MAVConnection
from dataclasses import dataclass
from typing import Callable
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from threading import Event
import time

MIL_MISSION_CMD = mavlink.MAV_CMD_USER_1

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
    Call a mission by its number. Log acknowledgement of receipt and return success of mission.
    """
    if mission_number < 0 or mission_number >= len(MISSIONS):
        print(f"Invalid mission number: {mission_number}")
        return False
    
    connection.mav.command_ack_send(
        command = MIL_MISSION_CMD,
        result = mavlink.MAV_RESULT_IN_PROGRESS)
    
    print("Mission %d: %s accepted." % (mission_number, MISSIONS[mission_number].name))
    return True


def send_mission_complete(connection: MAVConnection, mission_number: int):
    """
    Send a mavlink.COMMAND_ACK message to the vehicle to indicate completion of a mission.
    """
    connection.mav.command_ack_send(
        command = MIL_MISSION_CMD,
        result = mavlink.MAV_RESULT_ACCEPTED,
        result_param2 = mission_number)


def send_mission_message(connection: MAVConnection, mission_number: int):
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

        print("Sending mission command, attempt %d" % (attempt + 1))

        connection.mav.command_long_send(
            target_system = 1,
            target_component = 0,
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


@mission("Ping Companion Computer")
def ping_companion(vehicle: VehicleManager):
    """
    Ping the companion computer. Ping successful if this function is called by companion computer. 
    """
    vehicle.mav_connection.mav.statustext_send(
        severity = mavlink.MAV_SEVERITY_NOTICE,
        text = "Pong".encode())
    
    return True