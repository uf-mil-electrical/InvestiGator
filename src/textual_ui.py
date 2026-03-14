import math
from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Button, Select, Label, RichLog, Static, DataTable
from textual.containers import Horizontal, Vertical, Center

from InvestiGator import MAVConnection
import gc_helpers
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil

MISSIONS = [("Mission 1", 0), ("Mission 2", 1), ("Mission 3", 2)]

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
    "Heading"
]

class MissionControl(App):
    """
    Mission Control textual app
    """

    CSS_PATH = "gc_ui/gc.tcss"

    def __init__(self, connection: MAVConnection):
        self.connection = connection
        self.mode_map = mavutil.mode_mapping_bynumber(mavlink.MAV_TYPE_QUADROTOR)

        super().__init__()

    
    def on_mavlink_heartbeat(self, message: mavlink.MAVLink_heartbeat_message):
        if message.get_srcSystem() != 1:
            return
        self.call_from_thread(self.heartbeat_callback, message)

    def heartbeat_callback(self, message: mavlink.MAVLink_heartbeat_message):
        armed = bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        armed = "Armed" if armed else "Disarmed"

        mode = None
        if self.mode_map is not None:
            mode = self.mode_map.get(message.custom_mode)

        table = self.query_one(DataTable)
        table.update_cell(row_key="Arm Status", column_key="value", value=armed)
        table.update_cell(row_key="Flight Mode", column_key="value", value=mode)

    
    def on_mavlink_sys_status(self, message: mavlink.MAVLink_sys_status_message):
        if message.get_srcSystem() != 1:
            return
        self.call_from_thread(self.sys_status_callback, message)

    def sys_status_callback(self, message: mavlink.MAVLink_sys_status_message):
        prearmed = bool(message.onboard_control_sensors_health & mavlink.MAV_SYS_STATUS_PREARM_CHECK)
        prearmed = "Prearmed" if prearmed else "Prearm Failing"

        table = self.query_one(DataTable)
        table.update_cell(row_key="Prearm Status", column_key="value", value=prearmed)


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
                            table.add_row(label, key=label)

                with Horizontal(id="bottom_left"):
                    with Vertical(id="mission_select_panel", classes="panel") as mission_select_panel:
                        mission_select_panel.border_title = "Mission Select"
                        with Center():
                            yield Select(MISSIONS, id="mission_selector", prompt="Select Mission")
                        with Center():
                            yield Button("Start Mission", variant="primary", id="start_mission_button")
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

            
            with Vertical(id="log_panel", classes="panel") as log_panel:
                log_panel.border_title = "Mission Log"
                
        yield Footer()


    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "start_mission_button":
            selector = self.query_one(Select)
            if selector.value == Select.NULL:
                return
            self.add_class("mission_started")
            selector.disabled = True
            guided_button = self.query_one("#set_guided_button", Button)
            guided_button.disabled = True
            guided_button.variant = "primary"

        elif event.button.id == "cancel_mission_button":
            self.remove_class("mission_started")
            self.query_one(Select).disabled = False
            guided_button = self.query_one("#set_guided_button", Button)
            guided_button.disabled = False
            guided_button.variant = "success"

        elif event.button.id == "abort_mission_button":
            if not self.has_class("mission_abort"):
                self.add_class("mission_abort")
                self.remove_class("mission_started")
                self.query_one("#abort_mission_button", Button).disabled = True
                guided_button = self.query_one("#set_guided_button", Button)
                guided_button.disabled = False
                guided_button.variant = "success"

        elif event.button.id == "set_guided_button":
            if not self.has_class("guided_mode"):
                self.add_class("guided_mode")
            if self.has_class("mission_abort"):
                self.remove_class("mission_abort")
                selector = self.query_one(Select)
                selector.disabled = False
                selector.clear()
                self.query_one("#abort_mission_button", Button).disabled = False

        elif event.button.id == "recording_button":
            if not self.has_class("recording_enabled"):
                self.add_class("recording_enabled")


    def on_mount(self):
        self.theme = "nord"
        gc_helpers.configure_messages(connection)
        self.connection.subscribe(mavlink.MAVLink_heartbeat_message.msgname)(self.on_mavlink_heartbeat)
        self.connection.subscribe(mavlink.MAVLink_sys_status_message.msgname)(self.on_mavlink_sys_status)
        self.connection.subscribe(mavlink.MAVLink_local_position_ned_message.msgname)(self.on_mavlink_local_position_ned)
        self.connection.subscribe(mavlink.MAVLink_attitude_message.msgname)(self.on_mavlink_attitude)
        self.connection.subscribe(mavlink.MAVLink_global_position_int_message.msgname)(self.on_mavlink_global_position_int)

if __name__ == "__main__":
    connection = MAVConnection("tcp:127.0.0.1:5762")
    app = MissionControl(connection)
    app.run()
    connection.close()