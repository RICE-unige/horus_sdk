import time

from .utils.backend_manager import BackendManager
from .utils.branding import show_ascii_art
from .utils.requirements_checker import RequirementsChecker
from .utils.unity_monitor import UnityConnectionMonitor
from .utils.rosout_monitor import get_rosout_monitor


class Client:
    def __init__(self, backend="ros2", auto_launch=True):
        self.backend_type = backend
        self.auto_launch = auto_launch
        self.backend_manager = BackendManager(backend)
        self.requirements_checker = RequirementsChecker()
        self.unity_monitor = UnityConnectionMonitor()
        self.rosout_monitor = None  # Will be initialized after backend is ready
        self._shutdown_called = False  # Prevent duplicate shutdowns

        # Initialize with backend management
        self._initialize_with_backend()

    def _initialize_with_backend(self):
        """Complete initialization flow with backend management"""
        # 1. Show ASCII art
        show_ascii_art()

        # 2. Check and launch backend
        self._setup_backend()

        # 3. Establish connection
        self._connect_to_backend()

    def _setup_backend(self):
        """Check requirements and launch backend"""
        print(f"\n\033[96mInitializing {self.backend_type.upper()} backend\033[0m")

        # Check requirements with loading animation
        self._check_requirements_with_loading()

        # Launch backend if needed
        if self.auto_launch:
            self.backend_manager.launch_backend()

    def _check_requirements_with_loading(self):
        """Check requirements with interactive loading"""
        from .utils.spinner import Spinner

        requirements = {}
        checks = self.requirements_checker.get_backend_checks(self.backend_type)

        for req_name, check_func in checks:
            spinner = Spinner(f"Checking {req_name.lower()}")
            spinner.start()

            try:
                available, message = check_func(self.backend_type)
                spinner.stop()

                # Color coding: green for success, red for failure
                if available:
                    status_icon = "\033[92m✓\033[0m"
                    status_text = f"\033[90m{message}\033[0m"
                else:
                    status_icon = "\033[91m✗\033[0m"
                    status_text = f"\033[91m{message}\033[0m"

                print(f"  {status_icon} {req_name}: {status_text}")

                requirements[req_name] = {"available": available, "message": message}
            except Exception as e:
                spinner.stop()
                print(
                    f"  \033[91m✗\033[0m {req_name}: "
                    f"\033[91mCheck failed: {str(e)}\033[0m"
                )
                requirements[req_name] = {
                    "available": False,
                    "message": f"Check failed: {str(e)}",
                }

        # Overall status
        all_good = all(req["available"] for req in requirements.values())
        if not all_good:
            missing = [
                name for name, status in requirements.items() if not status["available"]
            ]
            print("\n\033[91mError: Missing required components\033[0m")
            print("\033[93mPlease install missing requirements and try again\033[0m")
            raise RuntimeError(f"Missing requirements: {', '.join(missing)}")

        return requirements

    def _connect_to_backend(self):
        """Establish connection to backend"""
        from .utils.spinner import Spinner

        spinner = Spinner("Establishing backend connection")
        spinner.start()

        # Test backend connection
        if self.backend_manager.test_connection():
            spinner.stop()
            port = self.backend_manager.get_port()
            print(
                f"  \033[92m✓\033[0m Backend connection: "
                f"\033[90mConnected on port {port}\033[0m"
            )

            # Test Unity endpoint connection
            unity_spinner = Spinner("Verifying Unity TCP endpoint")
            unity_spinner.start()
            time.sleep(1)  # Give it a moment to fully start

            if self.backend_manager.test_unity_endpoint():
                unity_spinner.stop()
                unity_port = self.backend_manager.get_unity_port()
                print(
                    f"  \033[92m✓\033[0m Unity TCP endpoint: "
                    f"\033[90mRunning on port {unity_port}\033[0m"
                )
            else:
                unity_spinner.stop()
                print(
                    "  \033[93m⚠\033[0m Unity TCP endpoint: "
                    "\033[93mNot yet ready (may still be starting)\033[0m"
                )

            # Show Unity MR connection information
            self._display_unity_connection_info()
            
            # Start rosout monitor for topic subscription tracking
            try:
                self.rosout_monitor = get_rosout_monitor()
                self.rosout_monitor.start()
            except Exception:
                pass

            print("\n\033[92mSDK initialized successfully\033[0m")
        else:
            spinner.stop()
            print(
                "  \033[91m✗\033[0m Backend connection: "
                "\033[91mFailed to connect\033[0m"
            )
            raise RuntimeError("Backend connection failed")

    def _display_unity_connection_info(self):
        """Display Unity MR application connection information"""
        import socket
        import time

        from .utils.spinner import Spinner

        print("\n\033[96m🎮 Unity Mixed Reality Connection\033[0m")
        print("\033[96m" + "═" * 45 + "\033[0m")

        # Get local IP address
        try:
            # Create a socket to get the local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            local_ip = "127.0.0.1"

        unity_port = 10000

        print("  \033[94m📡 Connection Details:\033[0m")
        print(f"     IP Address: \033[93m{local_ip}\033[0m")
        print(f"     Port:       \033[93m{unity_port}\033[0m")
        print("     Protocol:   \033[93mTCP\033[0m")

        print("\n  \033[95m🔗 HORUS MR App Configuration:\033[0m")
        print("     Enter these details in your Quest 3 HORUS app")
        print(f"     Host: \033[92m{local_ip}:{unity_port}\033[0m")

        # Start Unity connection monitoring
        print("\n  \033[96m⏳ Monitoring for Unity MR connections...\033[0m")

        # Set up connection callback
        def on_unity_connection(ip_address, is_connected):
            if is_connected:
                print(
                    f"  \033[92m✓\033[0m Unity MR connection: "
                    f"\033[92mConnected from {ip_address}\033[0m"
                )
                print("  \033[90m  → Mixed Reality interface active\033[0m")
            else:
                print(
                    f"  \033[93m⧖\033[0m Unity MR connection: "
                    f"\033[93mDisconnected from {ip_address}\033[0m"
                )
                print("  \033[90m  → Monitoring for new connections...\033[0m")

        self.unity_monitor.set_connection_callback(on_unity_connection)
        self.unity_monitor.start_monitoring()

        # Brief initial check
        spinner = Spinner("Checking for existing connections", style="dots")
        spinner.start()
        time.sleep(2)
        spinner.stop()

        if self.unity_monitor.is_unity_connected():
            connected_clients = self.unity_monitor.get_connected_clients()
            for ip in connected_clients:
                print(
                    f"  \033[92m✓\033[0m Unity MR connection: "
                    f"\033[92mAlready connected from {ip}\033[0m"
                )
            print("  \033[90m  → Mixed Reality interface active\033[0m")
        else:
            print(
                "  \033[93m⧖\033[0m Unity MR connection: "
                "\033[93mStandby mode (monitoring for connections)\033[0m"
            )
            print("  \033[90m  → Launch HORUS app on Quest 3 to connect\033[0m")

    def _check_unity_connection(self, port):
        """Check if Unity MR application is connected"""
        return self.unity_monitor.is_unity_connected()

    def shutdown(self):
        """Clean shutdown of client and monitoring"""
        # Prevent duplicate shutdown calls
        if self._shutdown_called:
            return
        self._shutdown_called = True

        print("\n\033[96mShutting down HORUS SDK...\033[0m")

        # Stop Unity connection monitoring
        if hasattr(self, "unity_monitor"):
            print("\033[90m  Stopping Unity connection monitoring...\033[0m")
            self.unity_monitor.stop_monitoring()
            print("\033[90m  ✓ Unity monitoring stopped\033[0m")

        # Stop rosout monitor
        if hasattr(self, "rosout_monitor") and self.rosout_monitor:
            try:
                self.rosout_monitor.stop()
            except Exception:
                pass

        # Stop live topic status board (if running)
        try:
            from .utils.topic_status import get_topic_status_board

            get_topic_status_board().stop()
        except Exception:
            pass

        # Stop backend and all ROS2 processes
        if hasattr(self, "backend_manager"):
            self.backend_manager.stop_backend()

        print("\033[92m  ✓ HORUS SDK shutdown complete\033[0m")
