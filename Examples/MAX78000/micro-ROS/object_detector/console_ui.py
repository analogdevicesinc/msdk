import os
from rich.console import Console
from rich.layout import Layout
from rich.panel import Panel
from rich.live import Live
from rich.status import Status
from rich import box
import time

class ConsolePanel(Console):
    def __init__(self, *args, **kwargs):
        console_file = open(os.devnull, "w")
        super().__init__(record=True, file=console_file, *args, **kwargs)

    def __rich_console__(self, console, options):
        lines = self.export_text(clear = False).splitlines()
        for line in lines[-options.height:]:
            yield line

class ConsoleUI():
    omx_console: ConsolePanel
    omx_status: Status
    microros_console: ConsolePanel
    microros_status: Status
    main_console: ConsolePanel
    main_console_status: Status
    layout: Layout
    input: str = None

    def __init__(self):
        self.layout = self._build_layout()

    def _build_layout(self) -> Layout:
        self.omx_console = ConsolePanel()
        self.omx_status = Status("OMX")        

        omx_console_layout = Layout()
        omx_console_layout.split_column(
            Layout(self.omx_status, size=1),
            Layout(self.omx_console)
        )

        self.microros_console = ConsolePanel()
        self.microros_status = Status("microROS")

        microros_console_layout = Layout()
        microros_console_layout.split_column(
            Layout(self.microros_status, size=1),
            Layout(self.microros_console)
        )

        self.main_console = ConsolePanel()
        self.main_console_status = Status("main console", spinner='point')

        main_console_layout = Layout()
        main_console_layout.split_column(
            Layout(Panel(self.main_console_status, box=box.HEAVY), size=3),
            Layout(self.main_console)
        )

        console_layout = Layout()
        console_layout.split_column(
            Layout(Panel(omx_console_layout), name="left"),
            Layout(Panel(microros_console_layout), name="right")
        )

        master_layout = Layout()
        master_layout.split_column(
            console_layout,
            main_console_layout
        )

        return master_layout
