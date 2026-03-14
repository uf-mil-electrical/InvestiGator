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
    return send_command(connection=connection, command=MIL_MISSION_CMD, param1=mission_number, target_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)


def send_command(connection: MAVConnection, command: int, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0, target_system=1, target_component=0, retries:int = 3, retry_timeout_s: float=1.0):
    """
    Send a mavlink.COMMAND_LONG message and wait for ack from vehicle. Retries up to retries times, each for retry_timeout_s seconds.
    """
    ack_event = Event()
    ack_result = None

    @connection.subscribe(mavlink.MAVLink_command_ack_message.msgname)
    def on_ack(message: mavlink.MAVLink_command_ack_message):
        nonlocal ack_result
        if message.command == command:
            ack_result = message.result
            ack_event.set()

    for attempt in range(retries):
        ack_event.clear()
        ack_result = None

        print(f"Sending mission command, attempt {attempt + 1}")

        connection.mav.command_long_send(
            target_system = target_system,
            target_component = target_component,
            command = command,
            confirmation = attempt,
            param1 = param1,
            param2 = param2,
            param3 = param3,
            param4 = param4,
            param5 = param5,
            param6 = param6,
            param7 = param7)
        
        if ack_event.wait(timeout=retry_timeout_s):
            connection.sub_manager.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
            return ack_result == mavlink.MAV_RESULT_IN_PROGRESS
    
    connection.sub_manager.unsubscribe(mavlink.MAVLink_command_ack_message.msgname, on_ack)
    return False

def configure_messages(connection: MAVConnection):
    """
    Send requests for messages from autopilot to ensure required data is being sent.
    param2 is interval requested in microseconds.
    """
    ten_hz_us = (1/10) * 1E6
    five_hz_us = (1/5) * 1E6

    send_command(connection=connection, command=mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, param2=five_hz_us)
    send_command(connection=connection, command=mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, param2=five_hz_us)
    send_command(connection=connection, command=mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_SYS_STATUS, param2=five_hz_us)
    send_command(connection=connection, command=mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavlink.MAVLINK_MSG_ID_ATTITUDE, param2=five_hz_us)


MISSION_MENU = build_mission_menu()