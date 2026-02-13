"""
Robot Communications Library using ZeroMQ
Replaces socket-based communication with bidirectional ZMQ
"""

import zmq
import json
import time
import threading
from PyQt6.QtCore import QObject, pyqtSignal

# Configuration
ROBOT_ADDRESSES = [
    "10.42.0.85",
    "127.0.0.1",
]
COMMAND_PORT = 5555
TELEMETRY_PORT = 5556
PING_INTERVAL_S = 1
HEARTBEAT_TIMEOUT_S = 2.0

class WorkerSignals(QObject):
    """Signals for communication with Qt GUI thread"""
    connection_status = pyqtSignal(bool, str)
    ping_response = pyqtSignal(float)  # Sends ping time in ms
    telemetry_update = pyqtSignal(dict)


class RobotClient:
    """
    ZMQ-based client for communicating with the robot
    Manages both command (REQ/REP) and telemetry (SUB) channels
    """
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.context = zmq.Context()
        self.signals = WorkerSignals()
        
        # REQ socket for commands
        self.command_socket = self.context.socket(zmq.REQ)
        self.command_socket.connect(f"tcp://{robot_ip}:{COMMAND_PORT}")
        self.command_socket.setsockopt(zmq.RCVTIMEO, 2000)  # 2 second timeout
        self.command_socket.setsockopt(zmq.LINGER, 0)
        
        # SUB socket for telemetry
        self.telemetry_socket = self.context.socket(zmq.SUB)
        self.telemetry_socket.connect(f"tcp://{robot_ip}:{TELEMETRY_PORT}")
        self.telemetry_socket.subscribe("")
        self.telemetry_socket.setsockopt(zmq.RCVTIMEO, 100)  # Non-blocking with short timeout
        
        self.connected = False
        self.running = True
        self.last_ping_time = 0
        self.ping_sent_time = None
        
        print(f"[RobotClient] Initialized connection to {robot_ip}")
    
    def send_command(self, command_type, **kwargs):
        """
        Send a command to the robot and wait for response
        Returns: response dict or None on error
        """
        try:
            command = {'type': command_type, 'timestamp': time.time(), **kwargs}
            self.command_socket.send_json(command)
            response = self.command_socket.recv_json()
            
            if not self.connected:
                self.connected = True
                self.signals.connection_status.emit(True, self.robot_ip)
            
            return response
            
        except zmq.Again:
            if self.connected:
                self.connected = False
                self.signals.connection_status.emit(False, "")
            return None
        except Exception as e:
            print(f"[RobotClient] Command error: {e}")
            if self.connected:
                self.connected = False
                self.signals.connection_status.emit(False, "")
            return None
    
    def send_joystick(self, lx, ly, rx, ry):
        """Send joystick values to robot"""
        return self.send_command('joystick', lx=lx, ly=ly, rx=rx, ry=ry)
    
    def send_button(self, button_id, action):
        """Send button press/release"""
        return self.send_command('button', button_id=button_id, action=action)
    
    def set_mode(self, mode):
        """Set robot mode (AUTO, TELEOP, STOPPED)"""
        return self.send_command('mode', mode=mode)
    
    def reset_robot(self):
        """Reset robot to stopped state"""
        return self.send_command('reset')
    
    def send_ping(self):
        """Send ping command"""
        self.ping_sent_time = time.time()
        return self.send_command('ping')
    
    def receive_telemetry(self):
        """
        Try to receive telemetry (non-blocking)
        Returns: telemetry dict or None
        """
        try:
            data = self.telemetry_socket.recv_json(flags=zmq.NOBLOCK)
            
            if not self.connected:
                self.connected = True
                self.signals.connection_status.emit(True, self.robot_ip)
            
            self.signals.telemetry_update.emit(data)
            return data
            
        except zmq.Again:
            return None
        except Exception as e:
            print(f"[RobotClient] Telemetry error: {e}")
            return None
    
    def cleanup(self):
        """Clean up sockets"""
        self.running = False
        self.command_socket.close()
        self.telemetry_socket.close()
        self.context.term()


class ConnectionManager(threading.Thread):
    """
    Manages connection attempts to robot across multiple addresses
    """
    def __init__(self):
        super().__init__()
        self.signals = WorkerSignals()
        self.client = None
        self.lock = threading.Lock()
        self.running = True
        self.current_address_idx = 0
        self.daemon = True
        
    def run(self):
        print("[ConnectionManager] Starting...")
        
        while self.running:
            with self.lock:
                if self.client is None or not self.client.connected:
                    # Try to connect to next address
                    address = ROBOT_ADDRESSES[self.current_address_idx]
                    print(f"[ConnectionManager] Attempting {address}...")
                    
                    try:
                        # Clean up old client
                        if self.client:
                            self.client.cleanup()
                        
                        # Create new client
                        self.client = RobotClient(address)
                        self.client.signals = self.signals
                        
                        # Test connection with ping
                        response = self.client.send_ping()
                        if response and response.get('status') == 'success':
                            print(f"[ConnectionManager] ✅ Connected to {address}")
                            self.signals.connection_status.emit(True, f"{address}:{COMMAND_PORT}")
                        else:
                            # Connection failed, try next address
                            self.current_address_idx = (self.current_address_idx + 1) % len(ROBOT_ADDRESSES)
                            self.client = None
                            
                    except Exception as e:
                        print(f"[ConnectionManager] Connection failed: {e}")
                        self.current_address_idx = (self.current_address_idx + 1) % len(ROBOT_ADDRESSES)
                        self.client = None
                        self.signals.connection_status.emit(False, "")
            
            time.sleep(1.0 if self.client is None else 0.5)
    
    def get_client(self):
        """Get the current connected client"""
        with self.lock:
            return self.client if self.client and self.client.connected else None
    
    def stop(self):
        self.running = False
        with self.lock:
            if self.client:
                self.client.cleanup()


class TelemetryReceiver(threading.Thread):
    """
    Continuously receives telemetry and handles pings
    """
    def __init__(self, conn_manager):
        super().__init__()
        self.conn_manager = conn_manager
        self.running = True
        self.last_ping_time = 0
        self.daemon = True
        
    def run(self):
        print("[TelemetryReceiver] Starting...")
        
        while self.running:
            client = self.conn_manager.get_client()
            
            if client:
                # Receive telemetry
                telemetry = client.receive_telemetry()
                
                # Send periodic pings
                if time.time() - self.last_ping_time > PING_INTERVAL_S:
                    ping_start = time.time()
                    response = client.send_ping()
                    
                    if response and response.get('status') == 'success':
                        ping_ms = (time.time() - ping_start) * 1000
                        client.signals.ping_response.emit(ping_ms)
                    
                    self.last_ping_time = time.time()
            else:
                time.sleep(0.1)
            
            time.sleep(0.01)  # Small delay to prevent busy-waiting
    
    def stop(self):
        self.running = False
