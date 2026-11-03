import logging
import time

from .utils import cli
from .utils.backend_manager import BackendManager
from .utils.requirements_checker import RequirementsChecker
from .utils.unity_monitor import UnityConnectionMonitor
from .utils.rosout_monitor import get_rosout_monitor
from .utils.network import resolve_advertise_ip

logger = logging.getLogger(__name__)


class Client:
    def __init__(self, backend="ros2", auto_launch=True):
        self.backend_type = backend
        self.auto_launch = auto_launch
        self.backend_manager = BackendManager(backend)
        self.requirements_checker = RequirementsChecker()
        self.unity_monitor = UnityConnectionMonitor(self.backend_manager.get_unity_port())
        self.rosout_monitor = None
        self._shutdown_called = False
        self._initialize_with_backend()

    def _initialize_with_backend(self):
        self._setup_backend()
        self._connect_to_backend()

    def _setup_backend(self):
        cli.print_step(f"Initializing {self.backend_type.upper()} backend")
        self._check_requirements_with_loading()
        if self.auto_launch:
            self.backend_manager.launch_backend()

    def _check_requirements_with_loading(self):
        requirements = {}
        for name, check in self.requirements_checker.get_backend_checks(self.backend_type):
            try:
                with cli.status(f"Checking {name.lower()}"):
                    available, message = check(self.backend_type)
            except Exception as exc:
                available, message = False, f"Check failed: {exc}"
            (cli.print_success if available else cli.print_error)(f"{name}: {message}")
            requirements[name] = {"available": available, "message": message}
        missing = [name for name, result in requirements.items() if not result["available"]]
        if missing:
            raise RuntimeError(f"Missing requirements: {', '.join(missing)}")
        return requirements

    def _connect_to_backend(self):
        with cli.status("Checking backend listener"):
            listening = self.backend_manager.test_connection()
        if not listening:
            raise RuntimeError("Backend connection failed")
        cli.print_success(f"Backend TCP listener reachable on port {self.backend_manager.get_port()}")
        with cli.status("Checking Unity bridge listener"):
            listening = self.backend_manager.test_unity_endpoint()
        if listening:
            cli.print_success(f"Unity TCP listener reachable on port {self.backend_manager.get_unity_port()}")
        else:
            cli.print_warning("Unity TCP listener not reachable yet")
        self._display_unity_connection_info()
        try:
            self.rosout_monitor = get_rosout_monitor()
            self.rosout_monitor.start()
        except Exception:
            logger.debug("Failed to start rosout monitor", exc_info=True)
        cli.print_info("SDK initialized. App/workspace readiness is shown during robot registration.")

    def _display_unity_connection_info(self):
        local_ip = resolve_advertise_ip()
        unity_port = self.backend_manager.get_unity_port()
        cli.print_info(f"HORUS MR endpoint: {local_ip}:{unity_port}")

        def on_connection(ip_address, connected):
            if not self.unity_monitor.observation_available:
                cli.print_warning("TCP peer observation unavailable; app state unknown")
            elif connected:
                cli.print_info(f"TCP peer connected: {ip_address} (app readiness not verified)")
            else:
                cli.print_info(f"TCP peer disconnected: {ip_address}")

        self.unity_monitor.set_connection_callback(on_connection)
        self.unity_monitor.start_monitoring()

    def _check_unity_connection(self, port):
        """Legacy transport-only check, not workspace readiness."""
        return self.unity_monitor.is_unity_connected()

    def shutdown(self):
        if self._shutdown_called:
            return
        self._shutdown_called = True
        cli.print_info("Stopping HORUS SDK monitoring")
        from .utils.topic_status import get_topic_status_board
        from .utils.topic_monitor import get_topic_monitor
        # One failed observer teardown must not prevent owned backend cleanup.
        callbacks = [self.unity_monitor.stop_monitoring]
        if self.rosout_monitor:
            callbacks.append(self.rosout_monitor.stop)
        callbacks.extend([get_topic_monitor().stop, get_topic_status_board().stop, self.backend_manager.stop_backend])
        failures = []
        for callback in callbacks:
            try:
                callback()
            except Exception as exc:
                failures.append(str(exc))
                logger.debug("SDK shutdown step failed", exc_info=True)
        if failures:
            cli.print_warning("SDK shutdown finished with errors: " + "; ".join(failures))
        else:
            cli.print_success("HORUS SDK shutdown complete")


def main(argv=None):
    """Console entrypoint for the HORUS SDK connection client."""
    import argparse

    parser = argparse.ArgumentParser(description="Start the HORUS SDK client.")
    parser.add_argument(
        "--backend",
        default="ros2",
        choices=("ros2",),
        help="Backend transport to initialize. Default: ros2.",
    )
    parser.add_argument(
        "--no-auto-launch",
        action="store_true",
        help="Do not auto-launch the configured backend process.",
    )
    parser.add_argument(
        "--no-keep-alive",
        action="store_true",
        help="Exit after initialization instead of keeping the client alive.",
    )
    args = parser.parse_args(argv)

    client = Client(backend=args.backend, auto_launch=not args.no_auto_launch)
    if args.no_keep_alive:
        client.shutdown()
        return 0

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        client.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
