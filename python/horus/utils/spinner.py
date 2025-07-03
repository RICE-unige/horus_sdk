import sys
import time
import threading

class Spinner:
    def __init__(self, message="Loading", delay=0.1, style="default"):
        self.message = message
        self.delay = delay
        self.running = False
        self.thread = None
        self.style = style
        
        # Different spinner styles
        self.spinner_styles = {
            'default': ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏'],
            'dots': ['⠈', '⠐', '⠠', '⢀', '⡀', '⠄', '⠂', '⠁'],
            'circle': ['◐', '◓', '◑', '◒'],
            'arrows': ['←', '↖', '↑', '↗', '→', '↘', '↓', '↙'],
            'pulse': ['●', '●', '●', '○', '○', '○']
        }
        
        self.spinner_chars = self.spinner_styles.get(style, self.spinner_styles['default'])
        self.current_char = 0
    
    def _spin(self):
        """Internal spinning animation"""
        while self.running:
            char = self.spinner_chars[self.current_char]
            sys.stdout.write(f"\r  \033[96m{char}\033[0m {self.message}...")
            sys.stdout.flush()
            self.current_char = (self.current_char + 1) % len(self.spinner_chars)
            time.sleep(self.delay)
    
    def start(self):
        """Start the spinner"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._spin)
            self.thread.daemon = True
            self.thread.start()
    
    def stop(self):
        """Stop the spinner and clear the line"""
        if self.running:
            self.running = False
            if self.thread:
                self.thread.join()
            # Clear the line
            sys.stdout.write(f"\r{' ' * (len(self.message) + 10)}\r")
            sys.stdout.flush()