"""Compatibility wrapper using the same terminal renderer as the dashboard."""

from . import cli


class Spinner:
    def __init__(self, message="Loading", delay=0.1, style="default"):
        self.message = message
        self.running = False
        self._status = None

    def start(self):
        if self.running:
            return
        self.running = True
        if cli.console.is_terminal and not cli.console.is_dumb_terminal:
            self._status = cli.console.status(cli._text(self.message), spinner="dots")
            self._status.start()
        else:
            cli.print_info(self.message)

    def stop(self):
        if self._status:
            self._status.stop()
            self._status = None
        self.running = False
