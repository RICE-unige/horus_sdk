#!/usr/bin/env python3
"""
Unity Connection Simulator

This script simulates a Unity MR application connecting to the HORUS backend
on port 10000. Run this in a separate terminal while testing the SDK initialization
to see the connection detection in action.
"""

import socket
import time
import sys
import threading
import signal

class UnityConnectionSimulator:
    def __init__(self, host='localhost', port=10000):
        self.host = host
        self.port = port
        self.running = False
        self.server_socket = None
        self.client_connections = []
        
    def start_server(self):
        """Start the Unity connection simulator server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            
            print(f"🎮 Unity MR Simulator Started")
            print(f"={'=' * 40}")
            print(f"📡 Listening on: {self.host}:{self.port}")
            print(f"🔗 Simulating Quest 3 HORUS app connection")
            print(f"⏹️  Press Ctrl+C to stop")
            print(f"{'=' * 40}")
            
            # Accept connections in a loop
            while self.running:
                try:
                    self.server_socket.settimeout(1.0)  # Non-blocking accept
                    client_socket, address = self.server_socket.accept()
                    
                    print(f"✅ Unity connection established from {address}")
                    print(f"🎯 Mixed Reality interface now active")
                    
                    self.client_connections.append(client_socket)
                    
                    # Handle client in a separate thread
                    client_thread = threading.Thread(
                        target=self.handle_client, 
                        args=(client_socket, address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except socket.timeout:
                    continue
                except OSError:
                    break
                    
        except Exception as e:
            print(f"❌ Error starting server: {e}")
            return False
            
        return True
    
    def handle_client(self, client_socket, address):
        """Handle individual client connections"""
        try:
            while self.running:
                try:
                    # Send heartbeat every 5 seconds
                    heartbeat = b'{"type": "heartbeat", "timestamp": ' + str(int(time.time())).encode() + b'}'
                    client_socket.send(heartbeat + b'\n')
                    time.sleep(5)
                except:
                    break
        except:
            pass
        finally:
            print(f"🔌 Unity connection from {address} disconnected")
            try:
                client_socket.close()
            except:
                pass
            if client_socket in self.client_connections:
                self.client_connections.remove(client_socket)
    
    def stop_server(self):
        """Stop the Unity connection simulator"""
        print(f"\n🛑 Stopping Unity MR Simulator...")
        self.running = False
        
        # Close all client connections
        for client in self.client_connections[:]:
            try:
                client.close()
            except:
                pass
        self.client_connections.clear()
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        print(f"✅ Unity MR Simulator stopped")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global simulator
    if simulator:
        simulator.stop_server()
    sys.exit(0)

def main():
    global simulator
    
    print("🚀 HORUS Unity MR Connection Simulator")
    print("=" * 50)
    print("This tool simulates a Unity Mixed Reality application")
    print("connecting to the HORUS backend on port 10000.")
    print("")
    print("Usage:")
    print("1. Start this simulator")
    print("2. In another terminal, run the HORUS SDK initialization")
    print("3. Watch the SDK detect the 'Unity' connection")
    print("")
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and start simulator
    simulator = UnityConnectionSimulator()
    
    try:
        if simulator.start_server():
            # Keep running until interrupted
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.stop_server()

if __name__ == "__main__":
    simulator = None
    main()