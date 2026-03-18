import math
import time
from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Button, Select, RichLog, Static, DataTable
from textual.containers import Horizontal, Vertical, Center
from textual.reactive import reactive
from textual import work

from InvestiGator import MAVConnection, constants
import gc_helpers
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil

MISSIONS = [(mission[1].name, mission[0]) for mission in enumerate(gc_helpers.MISSIONS)]

STATUS_TABLE_ROWS = [
    "Prearm Status",
    "Arm Status",
    "Flight Mode",
    "State",
    "NED Location",
    "NED Target",
    "NED Error",
    "NED Velocity",
    "Relative Altitude",
    "Attitude Degrees",
    "Heading",
    "Battery Voltage",
    "Battery Current"
]

MAV_SEVERITY_TO_COLOR = {
    mavlink.MAV_SEVERITY_EMERGENCY: "bold red",
    mavlink.MAV_SEVERITY_ALERT: "red",
    mavlink.MAV_SEVERITY_CRITICAL: "",
    mavlink.MAV_SEVERITY_ERROR: "",
    mavlink.MAV_SEVERITY_WARNING: "yellow",
    mavlink.MAV_SEVERITY_NOTICE: "",
    mavlink.MAV_SEVERITY_INFO: "green",
    mavlink.MAV_SEVERITY_DEBUG: "",
}

MIL_STATE_TO_TEXT = {
    constants.MIL_STATE_CONNECTING: "[yellow]Connecting to Drone[/yellow]",
    constants.MIL_STATE_MISSION: "[green]Running Mission[/green]",
    constants.MIL_STATE_OVERRIDE: "[bold red]OVERRIDE[/bold red]",
    constants.MIL_STATE_STANDBY: "[green]Standby[/green]",
    constants.MIL_STATE_INITIAL_OVERRIDE: "[bold yellow]Waiting for initial GUIDED mode[/bold yellow]"
}

class MissionControl(App):
    """
    Mission Control textual app
    """

    CSS_PATH = "gc_ui/gc.tcss"

    companion_state = reactive("Unknown")

    def __init__(self, connection: MAVConnection):
        self.connection = connection
        self.mode_map = mavutil.mode_mapping_bynumber(mavlink.MAV_TYPE_QUADROTOR)

        self.last_companion_heartbeat: float = 0.0
        self.last_drone_heartbeat: float = 0.0

        super().__init__()

    
    def on_mavlink_heartbeat(self, message: mavlink.MAVLink_heartbeat_message):
        if message.get_srcSystem() != 1:
            return
        self.call_from_thread(self.heartbeat_callback, message)

    def heartbeat_callback(self, message: mavlink.MAVLink_heartbeat_message):
        if message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
            self.last_drone_heartbeat = time.monotonic()

            armed = bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            armed = "Armed" if armed else "Disarmed"

            mode = None
            if self.mode_map is not None:
                mode = self.mode_map.get(message.custom_mode)

            table = self.query_one(DataTable)
            table.update_cell(row_key="Arm Status", column_key="value", value=armed)
            table.update_cell(row_key="Flight Mode", column_key="value", value=mode)

        elif message.get_srcSystem() == 1 and message.get_srcComponent() == mavlink.MAV_COMP_ID_ONBOARD_COMPUTER:
            self.last_companion_heartbeat = time.monotonic()
            self.companion_state = message.system_status

    
    def on_mavlink_sys_status(self, message: mavlink.MAVLink_sys_status_message):
        if message.get_srcSystem() != 1:
            return
        self.call_from_thread(self.sys_status_callback, message)

    def sys_status_callback(self, message: mavlink.MAVLink_sys_status_message):
        prearmed = bool(message.onboard_control_sensors_health & mavlink.MAV_SYS_STATUS_PREARM_CHECK)
        prearmed = "Prearmed" if prearmed else "Prearm Failing"

        voltage = message.voltage_battery / 1E3 # Given in mV
        current = message.current_battery / 1E2 # Given in cA

        voltage_string = f"{voltage} V"
        current_string = f"{current} A"

        table = self.query_one(DataTable)
        table.update_cell(row_key="Prearm Status", column_key="value", value=prearmed)
        table.update_cell(row_key="Battery Voltage", column_key="value", value=voltage_string)
        table.update_cell(row_key="Battery Current", column_key="value", value=current_string)


    def on_mavlink_local_position_ned(self, message: mavlink.MAVLink_local_position_ned_message):
        if message.get_srcSystem() != 1:
            return
        self.call_from_thread(self.local_position_ned_callback, message)

    def local_position_ned_callback(self, message: mavlink.MAVLink_local_position_ned_message):
        position = f"{message.x:.2f} | {message.y:.2f} | {message.z:.2f}"
        velocity = f"{message.vx:.2f} | {message.vy:.2f} | {message.vz:.2f}"

        table = self.query_one(DataTable)
        table.update_cell(row_key="NED Location",column_key="value", value=position)
        table.update_cell(row_key="NED Velocity", column_key="value", value=velocity)


    def on_mavlink_attitude(self, message: mavlink.MAVLink_attitude_message):
        if message.get_srcSystem() != 1:
            return
        self.call_from_thread(self.attitude_callback, message)
    
    def attitude_callback(self, message: mavlink.MAVLink_attitude_message):
        attitude = f"{math.degrees(message.roll):.2f} | {math.degrees(message.pitch):.2f} | {math.degrees(message.yaw):.2f}"

        table = self.query_one(DataTable)
        table.update_cell(row_key="Attitude Degrees", column_key="value", value=attitude)

    
    def on_mavlink_global_position_int(self, message: mavlink.MAVLink_global_position_int_message):
        if message.get_srcSystem() != 1:
            return
        self.call_from_thread(self.global_position_int_callback, message)

    def global_position_int_callback(self, message: mavlink.MAVLink_global_position_int_message):
        alt_rel_m = f"{message.relative_alt / 1000:.2f}" # Given in mm
        heading_deg = f"{message.hdg / 100:.2f}" # Given in cdeg

        table = self.query_one(DataTable)
        table.update_cell(row_key="Relative Altitude", column_key="value", value=alt_rel_m)
        table.update_cell(row_key="Heading", column_key="value", value=heading_deg)


    def on_mavlink_position_target_local_ned(self, message: mavlink.MAVLink_position_target_local_ned_message):
        if message.get_srcSystem() != 1:
            return
        self.call_from_thread(self.position_target_local_ned_callback, message)
    
    def position_target_local_ned_callback(self, message: mavlink.MAVLink_position_target_local_ned_message):
        target_ned = f"{message.x:.2f} | {message.y:.2f} | {message.z:.2f}"
        
        table = self.query_one(DataTable)
        current_ned_str = table.get_cell(row_key="NED Location", column_key="value")
        current_ned_str = current_ned_str.split('|')
        ned_error = f"{message.x - float(current_ned_str[0]):.2f} | {message.y - float(current_ned_str[1]):.2f} | {message.z - float(current_ned_str[2]):.2f}"

        table.update_cell(row_key="NED Target", column_key="value", value=target_ned)
        table.update_cell(row_key="NED Error", column_key="value", value=ned_error)
    

    def on_mavlink_statustext(self, message: mavlink.MAVLink_statustext_message):
        self.call_from_thread(self.statustext_callback, message)

    def statustext_callback(self, message: mavlink.MAVLink_statustext_message):
        color = MAV_SEVERITY_TO_COLOR.get(message.severity, "")

        prefix = "Unkown"
        if message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
            prefix = "FC: "
        if message.get_srcSystem() == 1 and message.get_srcComponent() == mavlink.MAV_COMP_ID_ONBOARD_COMPUTER:
            prefix = "PI: "

        formatted_message = prefix + message.text
        self.log_(formatted_message, color=color, gc_src=False)
        print(formatted_message)

    
    @work(thread=True)
    def send_mission_command(self, mission_number):
        result = gc_helpers.send_mission_message_wait_ack(self.connection, mission_number)
        self.call_from_thread(self.mission_command_callback, result, mission_number)

    def mission_command_callback(self, result, mission_number):
        message = f"Mission {mission_number}: {gc_helpers.MISSIONS[mission_number].name} was {"acknowledged" if result == mavlink.MAV_RESULT_ACCEPTED else "not acknowledged"}."
        color = "" if result == mavlink.MAV_RESULT_ACCEPTED else "red"
        self.log_(message, color)


    @work(thread=True)
    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0, target_system=1, target_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER):
        result = gc_helpers.send_command(connection=connection, command=command, param1=param1, param2=param2, param3=param3, param4=param4, param5=param5, param6=param6, param7=param7, target_system=target_system, target_component=target_component)
        self.call_from_thread(self.command_callback, result, command, param1)
    
    def command_callback(self, result, command, param1):
        system_command = None
        if command == constants.MIL_SYSTEM_CMD:
            match param1:
                case constants.MIL_SYSTEM_PING:
                    system_command = "Pong"
                case constants.MIL_SYSTEM_GUIDED:
                    system_command = "GUIDED"
                case constants.MIL_SYSTEM_OVERRIDE:
                    system_command = "Uncontrolled"
                case constants.MIL_SYSTEM_CANCEL:
                    system_command = "Cancel Mission"

            if system_command is None:
                message = f"Unknown command {"acknowledged" if result else "failed"}: (Command: {command}, param1: {param1}"
                self.log_(message)

            else:
                message = f"System command {system_command} {"acknowledged" if result else "failed"}"
                self.log_(message)


    @work(thread=True)
    def configure_messages(self):
        gc_helpers.configure_messages(self.connection)


    def log_(self, message, color="", gc_src=True):
        # TODO: Add timestamp and python logger
        if gc_src:
            message = "GC: " + message
        rich_message = message
        if color:
            print(color)
            rich_message = f"[{color}]{message}[/{color}]"

        self.query_one(RichLog).write(rich_message)

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()

        with Horizontal():
            with Vertical(id="left"):
                with Vertical(id="status_panel", classes="panel") as status_panel:
                    status_panel.border_title = "Drone Status"
                    with DataTable(show_header=False, id="status_table", show_cursor=False) as table:
                        table.add_column("Key", key="key")
                        table.add_column("Value", key="value", width=40)
                        for label in STATUS_TABLE_ROWS:
                            table.add_row(label, "--", key=label)

                with Horizontal(id="bottom_left"):
                    with Vertical(id="mission_select_panel", classes="panel") as mission_select_panel:
                        mission_select_panel.border_title = "Mission Select"
                        with Center():
                            yield Select(MISSIONS, id="mission_selector", prompt="Select Mission")
                        with Center():
                            yield Button("Start Mission", variant="primary", id="start_mission_button", disabled=True)
                        with Center():
                            yield Button("Cancel Mission", variant="error", id="cancel_mission_button")
                        with Center():
                            yield Static(id="uncontrolled_state_message", content="Drone in Uncontrolled State. \nSet mode to GUIDED to clear.")

                    with Vertical(id="system_panel", classes="panel") as system_panel:
                        system_panel.border_title = "System Commands"
                        # with Center():
                        #     yield Button(id="recording_button", label="Toggle Recording")
                        with Center():
                            yield Button(id="abort_mission_button", label="Abort", variant="error")
                        with Center():
                            yield Button(id="set_guided_button", label="Enable Guided", variant="success")
                        with Center():
                            yield Button(id="send_ping_button", label="Send Pi Ping", variant="primary")

            
            with Vertical(id="log_panel", classes="panel") as log_panel:
                log_panel.border_title = "Mission Log"
                yield RichLog(id="rich_log", wrap=True, markup=True)
                
        yield Footer()


    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "start_mission_button":
            selection = self.query_one(Select).value
            if selection == Select.NULL:
                return
            self.send_mission_command(selection)

        elif event.button.id == "cancel_mission_button":
            self.send_command(command=constants.MIL_SYSTEM_CMD, param1=3)

        elif event.button.id == "abort_mission_button":
            self.send_command(command=constants.MIL_SYSTEM_CMD, param1=2)

        elif event.button.id == "set_guided_button":
            self.send_command(command=constants.MIL_SYSTEM_CMD, param1=1)

        elif event.button.id == "send_ping_button":
            self.send_command(command=constants.MIL_SYSTEM_CMD, param1=0)
            
    def on_select_changed(self, event: Select.Changed):
        select = self.query_one(Select)
        start_mission_button = self.query_one("#start_mission_button")
        if select.is_blank():
            start_mission_button.disabled = True
        else:
            start_mission_button.disabled = False

    def watch_companion_state(self, old_state, new_state):
        if old_state == new_state:
            return
        old_state_text = MIL_STATE_TO_TEXT.get(old_state, "Unknown")
        new_state_text = MIL_STATE_TO_TEXT.get(new_state, "Unknown")
        message = f"STATE: {old_state_text} -> {new_state_text}"
        self.log_(message)
        self.query_one(DataTable).update_cell(row_key="State", column_key="value", value=new_state_text)

        for css_class in ["mission_abort", "mission_started"]:
            self.remove_class(css_class)

        if new_state in [constants.MIL_STATE_INITIAL_OVERRIDE, constants.MIL_STATE_OVERRIDE, constants.MIL_STATE_CONNECTING]:
            self.add_class("mission_abort")
            abort_button = self.query_one("#abort_mission_button", Button)
            abort_button.disabled = True
            guided_button = self.query_one("#set_guided_button", Button)
            guided_button.disabled = False
            guided_button.variant = "success"

        if new_state in [constants.MIL_STATE_MISSION, constants.MIL_STATE_STANDBY]:
            guided_button = self.query_one("#set_guided_button", Button)
            guided_button.disabled = True
            guided_button.variant = "primary"
            
        if new_state == constants.MIL_STATE_MISSION:
            self.add_class("mission_started")
            self.query_one(Select).disabled = True

        elif new_state == constants.MIL_STATE_STANDBY:
            selector = self.query_one(Select)
            selector.disabled = False
            selector.clear()
            abort_button = self.query_one("#abort_mission_button", Button)
            abort_button.disabled = False


    def on_mount(self):
        self.theme = "nord"
        self.configure_messages()
        self.connection.subscribe(mavlink.MAVLink_heartbeat_message.msgname)(self.on_mavlink_heartbeat)
        self.connection.subscribe(mavlink.MAVLink_sys_status_message.msgname)(self.on_mavlink_sys_status)
        self.connection.subscribe(mavlink.MAVLink_local_position_ned_message.msgname)(self.on_mavlink_local_position_ned)
        self.connection.subscribe(mavlink.MAVLink_attitude_message.msgname)(self.on_mavlink_attitude)
        self.connection.subscribe(mavlink.MAVLink_global_position_int_message.msgname)(self.on_mavlink_global_position_int)
        self.connection.subscribe(mavlink.MAVLink_position_target_local_ned_message.msgname)(self.on_mavlink_position_target_local_ned)
        self.connection.subscribe(mavlink.MAVLink_statustext_message.msgname)(self.on_mavlink_statustext)

if __name__ == "__main__":
    connection = MAVConnection("tcp:127.0.0.1:5762", mav_type=mavlink.MAV_TYPE_GCS)
    app = MissionControl(connection)
    app.run()
    connection.close()