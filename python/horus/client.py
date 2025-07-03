import subprocess
import time
import socket
from .utils.backend_manager import BackendManager
from .utils.requirements_checker import RequirementsChecker
from .utils.branding import show_ascii_art

class Client:
    def __init__(self, backend='ros2', auto_launch=True):
        self.backend_type = backend
        self.auto_launch = auto_launch
        self.backend_manager = BackendManager(backend)
        self.requirements_checker = RequirementsChecker()
        
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
        from .utils.spinner import Spinner
        
        print(f"\n\033[96mInitializing {self.backend_type.upper()} backend\033[0m")
        
        # Check requirements with loading animation
        requirements = self._check_requirements_with_loading()
        
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
                
                requirements[req_name] = {
                    'available': available,
                    'message': message
                }
            except Exception as e:
                spinner.stop()
                print(f"  \033[91m✗\033[0m {req_name}: \033[91mCheck failed: {str(e)}\033[0m")
                requirements[req_name] = {
                    'available': False,
                    'message': f'Check failed: {str(e)}'
                }
        
        # Overall status
        all_good = all(req['available'] for req in requirements.values())
        if not all_good:
            missing = [name for name, status in requirements.items() if not status['available']]
            print(f"\n\033[91mError: Missing required components\033[0m")
            print(f"\033[93mPlease install missing requirements and try again\033[0m")
            raise RuntimeError(f"Missing requirements: {', '.join(missing)}")
        
        return requirements
    
    def _connect_to_backend(self):
        """Establish connection to backend"""
        from .utils.spinner import Spinner
        
        spinner = Spinner("Establishing backend connection")
        spinner.start()
        
        # Test connection
        if self.backend_manager.test_connection():
            spinner.stop()
            port = self.backend_manager.get_port()
            print(f"  \033[92m✓\033[0m Backend connection: \033[90mConnected on port {port}\033[0m")
            
            # Show Unity MR connection information
            self._display_unity_connection_info()
            
            print(f"\n\033[92mSDK initialized successfully\033[0m")
        else:
            spinner.stop()
            print(f"  \033[91m✗\033[0m Backend connection: \033[91mFailed to connect\033[0m")
            raise RuntimeError("Backend connection failed")
    
    def _display_unity_connection_info(self):
        """Display Unity MR application connection information"""
        import socket
        import time
        from .utils.spinner import Spinner
        
        print(f"\n\033[96m🎮 Unity Mixed Reality Connection\033[0m")
        print(f"\033[96m{'═' * 45}\033[0m")
        
        # Get local IP address
        try:
            # Create a socket to get the local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except:
            local_ip = "127.0.0.1"
        
        unity_port = 10000
        
        print(f"  \033[94m📡 Connection Details:\033[0m")
        print(f"     IP Address: \033[93m{local_ip}\033[0m")
        print(f"     Port:       \033[93m{unity_port}\033[0m")
        print(f"     Protocol:   \033[93mTCP\033[0m")
        
        print(f"\n  \033[95m🔗 HORUS MR App Configuration:\033[0m")
        print(f"     Enter these details in your Quest 3 HORUS app")
        print(f"     Host: \033[92m{local_ip}:{unity_port}\033[0m")
        
        # Check for Unity connection with animation
        print(f"\n  \033[96m⏳ Waiting for Unity MR connection...\033[0m")
        
        spinner = Spinner("Monitoring Unity connection", style="dots")
        spinner.start()
        
        # Monitor for Unity connection (with timeout)
        connection_timeout = 10  # seconds
        start_time = time.time()
        unity_connected = False
        
        while time.time() - start_time < connection_timeout:
            if self._check_unity_connection(unity_port):
                unity_connected = True
                break
            time.sleep(0.5)
        
        spinner.stop()
        
        if unity_connected:
            print(f"  \033[92m✓\033[0m Unity MR connection: \033[92mConnected successfully\033[0m")
            print(f"  \033[90m  → Mixed Reality interface active\033[0m")
        else:
            print(f"  \033[93m⧖\033[0m Unity MR connection: \033[93mStandby mode (no MR app connected)\033[0m")
            print(f"  \033[90m  → Launch HORUS app on Quest 3 to connect\033[0m")
    
    def _check_unity_connection(self, port):
        """Check if Unity MR application is connected"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.1)
            # Try to connect to see if something is listening
            result = sock.connect_ex(('127.0.0.1', port))
            sock.close()
            return result == 0
        except:
            return False
