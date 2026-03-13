from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Button, Select, Label, RichLog
from textual.containers import Horizontal, Vertical

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
                        yield Select(MISSIONS, id="mission_selector")
                    
                    with Vertical(id="system_panel", classes="panel") as system_panel:
                        system_panel.border_title = "System Commands"
            
            with Vertical(id="log_panel", classes="panel") as log_panel:
                log_panel.border_title = "Mission Log"
                
        yield Footer()

    def on_mount(self):
        self.theme = "nord"

if __name__ == "__main__":
    app = MissionControl()
    app.run()