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

        
                
        yield Footer()

    def on_mount(self):
        self.theme = "nord"

if __name__ == "__main__":
    app = MissionControl()
    app.run()