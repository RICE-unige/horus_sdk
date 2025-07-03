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
            print(f"\n\033[92mSDK initialized successfully\033[0m")
        else:
            spinner.stop()
            print(f"  \033[91m✗\033[0m Backend connection: \033[91mFailed to connect\033[0m")
            raise RuntimeError("Backend connection failed")
