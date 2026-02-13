import os
import zmq
import time
import threading
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Constants
COMMAND_PORT = 5555
TELEMETRY_PORT = 5556
TELEMETRY_RATE_HZ = 10
HEARTBEAT_TIMEOUT_S = 0.75
WATCHDOG_CHECK_INTERVAL_S = 0.1

# Global state
last_heartbeat = time.time()
heartbeat_lock = threading.Lock()
connection_lost = False
robot_mode = "STOPPED"  # STOPPED, AUTO, TELEOP


class JoystickData:
    """Container for joystick input data."""
    def __init__(self, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        self.lx = max(-1.0, min(1.0, lx))
        self.ly = max(-1.0, min(1.0, ly))
        self.rx = max(-1.0, min(1.0, rx))
        self.ry = max(-1.0, min(1.0, ry))


def all_stop():
    """Emergency stop - called when connection is lost."""
    global connection_lost
    if not connection_lost:
        logger.warning("!!!! CONNECTION LOST - EMERGENCY STOP !!!!")
        connection_lost = True
        set_motor_speeds([0.0, 0.0, 0.0, 0.0])


def watchdog_thread():
    """Monitor heartbeat and trigger emergency stop if connection lost."""
    global connection_lost
    logger.info("Watchdog thread started")
    
    while True:
        with heartbeat_lock:
            time_since_heartbeat = time.time() - last_heartbeat
            
            if time_since_heartbeat > HEARTBEAT_TIMEOUT_S:
                if not connection_lost:
                    all_stop()
            elif connection_lost:
                logger.info("Connection restored")
                connection_lost = False
                
        time.sleep(WATCHDOG_CHECK_INTERVAL_S)


def calculate_motor_speeds(data: JoystickData) -> list:
    """
    Calculate mecanum drive motor speeds from joystick input.
    Returns: List of 4 motor speeds [FL, FR, RL, RR]
    """
    motor1_speed = data.ly + data.lx + data.rx  # Front Left
    motor2_speed = data.ly - data.lx - data.rx  # Front Right
    motor3_speed = data.ly - data.lx + data.rx  # Rear Left
    motor4_speed = data.ly + data.lx - data.rx  # Rear Right

    speeds = [motor1_speed, motor2_speed, motor3_speed, motor4_speed]
    
    # Normalize speeds
    max_speed = max(abs(s) for s in speeds)
    if max_speed > 1.0:
        speeds = [s / max_speed for s in speeds]

    return speeds


def set_motor_speeds(speeds: list):
    """Set the motor speeds (placeholder for actual motor control)."""
    # TODO: Implement actual motor control
    pass


def update_heartbeat():
    """Update the last heartbeat timestamp."""
    global last_heartbeat, connection_lost
    with heartbeat_lock:
        last_heartbeat = time.time()
        if connection_lost:
            logger.info("Connection restored via command")
            connection_lost = False


class RobotServer:
    """ZMQ-based robot server"""
    def __init__(self):
        self.context = zmq.Context()
        
        # REP socket for commands
        self.command_socket = self.context.socket(zmq.REP)
        self.command_socket.bind(f"tcp://*:{COMMAND_PORT}")
        
        # PUB socket for telemetry
        self.telemetry_socket = self.context.socket(zmq.PUB)
        self.telemetry_socket.bind(f"tcp://*:{TELEMETRY_PORT}")
        
        self.running = True
        self.telemetry_data = {
            'battery': 12.5,
            'mode': robot_mode,
            'motor_speeds': [0.0, 0.0, 0.0, 0.0],
            'sensors': {
                'ultrasonic': 0,
                'ir': 0,
                'gyro': 0.0
            }
        }
        
        logger.info(f"Robot server initialized on ports {COMMAND_PORT}/{TELEMETRY_PORT}")
    
    def handle_command(self, command):
        """Process incoming command"""
        global robot_mode
        
        cmd_type = command.get('type')
        #update_heartbeat()
        
        try:
            if cmd_type == 'ping':
                return {'status': 'success', 'timestamp': time.time()}
            
            elif cmd_type == 'joystick':
                lx = command.get('lx', 0.0)
                ly = command.get('ly', 0.0)
                rx = command.get('rx', 0.0)
                ry = command.get('ry', 0.0)
                
                joystick_data = JoystickData(lx, ly, rx, ry)
                motor_speeds = calculate_motor_speeds(joystick_data)
                
                if robot_mode == "TELEOP":
                    set_motor_speeds(motor_speeds)
                    self.telemetry_data['motor_speeds'] = motor_speeds
                    logger.debug(f"Motors: {motor_speeds}")
                
                return {'status': 'success'}
            
            elif cmd_type == 'button':
                button_id = command.get('button_id')
                action = command.get('action')
                logger.info(f"Button {button_id} {action}")
                
                # TODO: Handle button actions
                
                return {'status': 'success'}
            
            elif cmd_type == 'mode':
                new_mode = command.get('mode', 'STOPPED').upper()
                
                if new_mode in ["AUTO", "TELEOP", "STOPPED"]:
                    robot_mode = new_mode
                    self.telemetry_data['mode'] = robot_mode
                    logger.info(f"Mode changed to: {robot_mode}")
                    
                    if robot_mode == "STOPPED":
                        set_motor_speeds([0.0, 0.0, 0.0, 0.0])
                    
                    return {'status': 'success', 'mode': robot_mode}
                else:
                    return {'status': 'error', 'message': f'Invalid mode: {new_mode}'}
            
            elif cmd_type == 'reset':
                robot_mode = "STOPPED"
                set_motor_speeds([0.0, 0.0, 0.0, 0.0])
                self.telemetry_data['mode'] = robot_mode
                self.telemetry_data['motor_speeds'] = [0.0, 0.0, 0.0, 0.0]
                logger.info("Robot reset")
                return {'status': 'success'}
            
            else:
                logger.warning(f"Unknown command: {cmd_type}")
                return {'status': 'error', 'message': f'Unknown command: {cmd_type}'}
                
        except Exception as e:
            logger.error(f"Error handling command: {e}")
            return {'status': 'error', 'message': str(e)}
    
    def command_loop(self):
        """Handle incoming commands"""
        logger.info("Command handler ready")
        
        while self.running:
            try:
                command = self.command_socket.recv_json()
                response = self.handle_command(command)
                self.command_socket.send_json(response)
            except Exception as e:
                logger.error(f"Command loop error: {e}")
                try:
                    self.command_socket.send_json({
                        'status': 'error',
                        'message': str(e)
                    })
                except:
                    pass
    
    def telemetry_loop(self):
        """Broadcast telemetry"""
        logger.info("Telemetry broadcaster ready")
        
        while self.running:
            try:
                # TODO: Update with real sensor data
                # self.telemetry_data['battery'] = read_battery()
                # self.telemetry_data['sensors']['ultrasonic'] = read_ultrasonic()
                
                self.telemetry_data['timestamp'] = time.time()
                self.telemetry_data['mode'] = robot_mode
                
                self.telemetry_socket.send_json(self.telemetry_data)
                time.sleep(1.0 / TELEMETRY_RATE_HZ)
                
            except Exception as e:
                logger.error(f"Telemetry error: {e}")
    
    def start(self):
        """Start server threads"""
        # Start watchdog
        watchdog = threading.Thread(target=watchdog_thread, daemon=True)
        watchdog.start()
        
        # Start telemetry
        telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        telemetry_thread.start()
        
        # Run command handler in main thread
        try:
            self.command_loop()
        except KeyboardInterrupt:
            logger.info("Server shutdown requested")
            self.running = False
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        self.command_socket.close()
        self.telemetry_socket.close()
        self.context.term()


def main():
    os.system('cls' if os.name == 'nt' else 'clear')
    """Start the robot server"""
    logger.info("🤖 Starting robot server...")
    
    server = RobotServer()
    
    try:
        server.start()
    finally:
        server.cleanup()
        logger.info("🤖 Robot server stopped")


if __name__ == "__main__":
    main()