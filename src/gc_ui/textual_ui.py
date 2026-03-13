from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Button, Select, Label, RichLog, Static
from textual.containers import Horizontal, Vertical, Center

MISSIONS = [("Mission 1", 0), ("Mission 2", 1), ("Mission 3", 2)]

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
                        with Center():
                            yield Button(id="recording_button", label="Toggle Recording")
                        with Center():
                            yield Button(id="abort_mission_button", label="Abort")
                        with Center():
                            yield Button(id="set_guided_button", label="Enable Guided")

            
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

        elif event.button.id == "cancel_mission_button":
            self.remove_class("mission_started")
            self.query_one(Select).disabled = False

        elif event.button.id == "abort_mission_button":
            if not self.has_class("mission_abort"):
                self.add_class("mission_abort")
                self.remove_class("mission_started")

        elif event.button.id == "set_guided_button":
            if not self.has_class("guided_mode"):
                self.add_class("guided_mode")
            if self.has_class("mission_abort"):
                self.remove_class("mission_abort")
                self.query_one(Select).disabled = False

        elif event.button.id == "recording_button":
            if not self.has_class("recording_enabled"):
                self.add_class("recording_enabled")


    def on_mount(self):
        self.theme = "nord"

if __name__ == "__main__":
    app = MissionControl()
    app.run()