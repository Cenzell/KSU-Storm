import socket
import threading
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Constants
HEARTBEAT_TIMEOUT_S = 2.0  # Must be > PING_INTERVAL_S from driver (1s) with margin
WATCHDOG_CHECK_INTERVAL_S = 0.1

# Global state
last_heartbeat = time.time()
heartbeat_lock = threading.Lock()
connection_lost = False
current_client = None
client_lock = threading.Lock()
robot_mode = "STOPPED"  # STOPPED, AUTO, TELEOP


class JoystickData:
    """Container for joystick input data."""
    def __init__(self, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        self.lx = max(-1.0, min(1.0, lx))  # Clamp to valid range
        self.ly = max(-1.0, min(1.0, ly))
        self.rx = max(-1.0, min(1.0, rx))
        self.ry = max(-1.0, min(1.0, ry))


def all_stop():
    """Emergency stop - called when connection is lost."""
    global connection_lost
    if not connection_lost:
        logger.warning("!!!! CONNECTION LOST - EMERGENCY STOP !!!!")
        connection_lost = True
        # TODO: Implement actual motor stop commands
        # Example: set_motor_speeds([0.0, 0.0, 0.0, 0.0])


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
                # Connection restored
                logger.info("Connection restored")
                connection_lost = False
                
        time.sleep(WATCHDOG_CHECK_INTERVAL_S)


def calculate_motor_speeds(data: JoystickData) -> list[float]:
    """
    Calculate mecanum drive motor speeds from joystick input.
    
    Args:
        data: JoystickData object with lx, ly, rx, ry values
        
    Returns:
        List of 4 motor speeds [FL, FR, RL, RR] normalized to [-1.0, 1.0]
    """
    # Mecanum drive kinematics
    motor1_speed = data.ly + data.lx + data.rx  # Front Left
    motor2_speed = data.ly - data.lx - data.rx  # Front Right
    motor3_speed = data.ly - data.lx + data.rx  # Rear Left
    motor4_speed = data.ly + data.lx - data.rx  # Rear Right

    speeds = [motor1_speed, motor2_speed, motor3_speed, motor4_speed]
    
    # Normalize speeds to be within [-1.0, 1.0]
    max_speed = max(abs(s) for s in speeds)
    if max_speed > 1.0:
        speeds = [s / max_speed for s in speeds]

    return speeds


def set_motor_speeds(speeds: list[float]):
    """
    Set the motor speeds (placeholder for actual motor control).
    
    Args:
        speeds: List of 4 motor speeds [FL, FR, RL, RR]
    """
    # TODO: Implement actual motor control logic - And figure out what we will get
    pass


def update_heartbeat():
    """Update the last heartbeat timestamp."""
    global last_heartbeat, connection_lost
    with heartbeat_lock:
        last_heartbeat = time.time()
        if connection_lost:
            logger.info("Connection restored via command")
            connection_lost = False


def handle_command(cmd: str):
    """
    Parse and handle a single command string from the client.
    
    Args:
        cmd: Command string (without newline)
    """
    global robot_mode
    
    parts = cmd.strip().split()
    if not parts:
        return

    command_name = parts[0]
    
    # Update heartbeat on any command
    update_heartbeat()
    
    if command_name == "PING":
        # Silent heartbeat
        pass
        
    elif command_name == "JOYSTICKS":
        try:
            if len(parts) < 2:
                logger.error("JOYSTICKS command missing values")
                return
                
            values_str = parts[1]
            speeds = [float(s) for s in values_str.split(',')]
            
            if len(speeds) != 4:
                logger.error(f"Invalid number of joystick values: {len(speeds)}, expected 4")
                return
                
            joystick_data = JoystickData(lx=speeds[0], ly=speeds[1], 
                                        rx=speeds[2], ry=speeds[3])
            motor_speeds = calculate_motor_speeds(joystick_data)
            
            # Only apply motor speeds in TELEOP mode
            if robot_mode == "TELEOP":
                set_motor_speeds(motor_speeds)
                logger.debug(
                    f"Motor speeds: FL={motor_speeds[0]:.2f}, FR={motor_speeds[1]:.2f}, "
                    f"RL={motor_speeds[2]:.2f}, RR={motor_speeds[3]:.2f}"
                )
            
        except (IndexError, ValueError) as e:
            logger.error(f"Error parsing JOYSTICKS command '{cmd}': {e}")
            
    elif command_name == "BTN":
        try:
            if len(parts) < 3:
                logger.error("BTN command missing parameters")
                return
                
            button_id = parts[1]
            action = parts[2]
            logger.info(f"Button event: {button_id} {action}")
            
            # TODO: Implement button-specific actions
            # Example: if button_id == "0" and action == "DOWN": activate_gripper()
            
        except IndexError as e:
            logger.error(f"Error parsing BTN command '{cmd}': {e}")
            
    elif command_name == "MODE":
        try:
            if len(parts) < 2:
                logger.error("MODE command missing parameter")
                return
                
            new_mode = parts[1].upper()
            
            if new_mode in ["AUTO", "TELEOP", "STOPPED"]:
                robot_mode = new_mode
                logger.info(f"Mode changed to: {robot_mode}")
                
                if robot_mode == "STOPPED":
                    set_motor_speeds([0.0, 0.0, 0.0, 0.0])
            else:
                logger.error(f"Invalid mode: {new_mode}")
                
        except IndexError as e:
            logger.error(f"Error parsing MODE command '{cmd}': {e}")
            
    elif command_name == "RESET":
        robot_mode = "STOPPED"
        set_motor_speeds([0.0, 0.0, 0.0, 0.0])
        logger.info("Robot reset to STOPPED mode")
        
    else:
        logger.warning(f"Unknown command: {cmd}")


def handle_client(conn, addr):
    """
    Handle communication with a connected client.
    
    Args:
        conn: Socket connection object
        addr: Client address tuple
    """
    global current_client
    
    logger.info(f"New client connected: {addr}")
    buffer = ""
    
    try:
        conn.settimeout(5.0)  # 5 second timeout for recv
        
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    logger.info(f"Client {addr} closed connection")
                    break
                
                buffer += data.decode('utf-8')
                
                # Process complete commands (delimited by newlines)
                while '\n' in buffer:
                    command, buffer = buffer.split('\n', 1)
                    handle_command(command)
                    
                    # Send acknowledgment
                    try:
                        conn.sendall(b'ACK\n')
                    except (socket.error, OSError) as e:
                        logger.error(f"Error sending ACK: {e}")
                        raise
                        
            except socket.timeout:
                # Timeout is expected, continue loop
                continue
                
    except ConnectionResetError:
        logger.warning(f"Client {addr} forcefully disconnected")
    except Exception as e:
        logger.error(f"Error with client {addr}: {e}")
    finally:
        logger.info(f"Client disconnected: {addr}")
        
        with client_lock:
            if current_client == conn:
                current_client = None
                
        # Stop motors when client disconnects
        set_motor_speeds([0.0, 0.0, 0.0, 0.0])
        
        try:
            conn.close()
        except Exception as e:
            logger.error(f"Error closing connection: {e}")


def main():
    """Start the TCP server and handle incoming connections."""
    global current_client
    
    host = '0.0.0.0'
    port = 5000

    # Start watchdog thread
    watchdog = threading.Thread(target=watchdog_thread, daemon=True)
    watchdog.start()
    
    logger.info(f"🤖 Robot TCP server starting on {host}:{port}...")
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((host, port))
        server_socket.listen(1)  # Allow only 1 queued connection
        
        logger.info(f"🤖 Robot TCP server listening on port {port}")
        
        while True:
            try:
                conn, addr = server_socket.accept()
                
                # Close previous client if exists
                with client_lock:
                    if current_client:
                        logger.warning(f"Closing previous client to accept new connection from {addr}")
                        try:
                            current_client.close()
                        except Exception as e:
                            logger.error(f"Error closing previous client: {e}")
                    current_client = conn
                
                # Handle client in separate thread
                client_thread = threading.Thread(target=handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
                
            except KeyboardInterrupt:
                logger.info("Server shutdown requested")
                break
            except Exception as e:
                logger.error(f"Error accepting connection: {e}")


if __name__ == "__main__":
    main()