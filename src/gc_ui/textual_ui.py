from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Button, Select, Label, RichLog, Static, DataTable
from textual.containers import Horizontal, Vertical, Center

MISSIONS = [("Mission 1", 0), ("Mission 2", 1), ("Mission 3", 2)]

STATUS_TABLE_ROWS = [
    "Arm Status",
    "Flight Mode",
    "State",
    "GPS Location",
    "NED Location",
]

class MissionControl(App):
    """
    Mission Control textual app
    """

    CSS_PATH = "gc.tcss"

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()

        with Horizontal():
            with Vertical(id="left"):
                with Vertical(id="status_panel", classes="panel") as status_panel:
                    status_panel.border_title = "Drone Status"
                    with DataTable(show_header=False, id="status_table") as table:
                        table.add_column("Key", key="key")
                        table.add_column("Value", key="value")
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
        self.query_one(DataTable).update_cell(row_key="Arm Status", column_key="value", value="Test")

if __name__ == "__main__":
    app = MissionControl()
    app.run()