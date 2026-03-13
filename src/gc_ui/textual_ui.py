from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Button, Select, Label, RichLog
from textual.containers import Horizontal, Vertical

MISSIONS = [("Mission 1", 0), ("Mission 2", 1), ("Mission 3", 2)]

class MissionControl(App):
    """
    Practice Textual app to test UI.
    """

    CSS = """
    .with-border {
        border: heavy green;
    }
    """

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()

        with Horizontal():
            with Vertical(id="left"):
                with Vertical(id="status", classes="with-border"):
                    yield Label("--- Drone Status ---")
                    yield Label("Mode: --", id="mode")
                    yield Label("Armed: --", id="armed")
                    yield Label("Relative Altitude: --", id="altitude")
                    yield Label("GPS: --", id="gps")
                    yield Label("NED Position: --", id="ned")
                    yield Label("State: --", id="state")

                with Vertical(id="mission_select", classes="with-border"):
                    yield Label("--- Mission Select ---")
                    yield Select(options=MISSIONS, id="mission_select")
                    yield Button("Start Mission", id="start_mission", variant="success")

            with Vertical(id="mission_log_display", classes="with-border"):
                yield Label("--- Mission Log ---")
                yield RichLog(id="mission_log")
                
        yield Footer()

    def on_mount(self):
        self.query_one("#mission_log", RichLog).write("Testing")

if __name__ == "__main__":
    app = MissionControl()
    app.run()