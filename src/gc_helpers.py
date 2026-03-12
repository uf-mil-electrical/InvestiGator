from missions import MISSIONS
from InvestiGator import MAVConnection
from InvestiGator.constants import MIL_SYSTEM_CMD, MIL_MISSION_CMD
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from threading import Event


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


MISSION_MENU = build_mission_menu()