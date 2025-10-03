import socket
import threading
import time
from PyQt6.QtCore import QObject, pyqtSignal

ROBOT_ADDRESSES = [
    #"wildrobo.local:5000",
    "10.42.0.85:5000",
    "127.0.0.1:5000",
]
GAMEPAD_POLL_RATE_MS = 20
PING_INTERVAL_S = 1

class WorkerSignals(QObject):
    connection_status = pyqtSignal(bool, str)
    ping_response = pyqtSignal()

class TelemetryReceiver(threading.Thread):
    def __init__(self, conn_manager):
        super().__init__()
        self.signals = WorkerSignals()
        self.conn_manager = conn_manager
        self.running = True

    def run(self):
        buffer = ""
        while self.running:
            sock = self.conn_manager.get_socket()
            if sock:
                try:
                    data = sock.recv(1024)
                    if not data:
                        self.conn_manager.disconnect()
                        continue
                    
                    buffer += data.decode('utf-8')
                    while '\n' in buffer:
                        message, buffer = buffer.split('\n', 1)
                        if message.strip() == "ACK":
                            self.signals.ping_response.emit()
                except (socket.error, OSError):
                    self.conn_manager.disconnect()
            else:
                time.sleep(1)

    def stop(self):
        self.running = False

class ConnectionManager(threading.Thread):
    def __init__(self):
        super().__init__()
        self.signals = WorkerSignals()
        self.sock = None
        self.lock = threading.Lock()
        self.running = True

    def run(self):
        print("Starting connection manager...")
        address_idx = 0
        while self.running:
            with self.lock:
                if self.sock is None:
                    address_str = ROBOT_ADDRESSES[address_idx]
                    host, port_str = address_str.split(':')
                    port = int(port_str)
                    
                    print(f"Trying to connect to: {host}:{port}")
                    try:
                        new_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        new_sock.settimeout(1.0)
                        new_sock.connect((host, port))
                        new_sock.settimeout(None)
                        self.sock = new_sock
                        print(f"✅ Connected to {address_str}")
                        self.signals.connection_status.emit(True, address_str)
                    except (socket.error, OSError) as e:
                        print(f"🔌 Connection to {address_str} failed: {e}")
                        self.signals.connection_status.emit(False, "")
                        address_idx = (address_idx + 1) % len(ROBOT_ADDRESSES)
            time.sleep(0.2)

    def get_socket(self):
        with self.lock:
            return self.sock

    def disconnect(self):
        with self.lock:
            if self.sock:
                self.sock.close()
                self.sock = None
                self.signals.connection_status.emit(False, "")
    
    def stop(self):
        self.running = False
        self.disconnect()