import socket
import threading
import time

HEARTBEAT_TIME = 0.5
last_heartbeat = time.time()
heartbeat_lock = threading.Lock()
connection_lost = False

class JoystickData:
    def __init__(self, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        self.lx = lx
        self.ly = ly
        self.rx = rx
        self.ry = ry

def all_stop():
    if(not connection_lost): print("!!!! Connection Stop Triggered !!!!")
    #Need to develope a better method to stop all motors safely

def watchdog_thread(): 
    global connection_lost
    while True:
        with heartbeat_lock:
            if time.time() - last_heartbeat > HEARTBEAT_TIME:
                if not connection_lost:
                    all_stop()
                    connection_lost = True
        time.sleep(0.1)

def calculate_motor_speeds(data: JoystickData) -> list[float]:
    """Calculates mecanum drive motor speeds."""
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

def handle_command(cmd: str):
    """Parses and handles a single command string from the client."""
    parts = cmd.strip().split()
    if not parts:
        return

    command_name = parts[0]
    
    if command_name == "PING":
        # print("Heartbeat received!")
        pass
    elif command_name == "JOYSTICKS":
        try:
            values_str = parts[1]
            speeds = [float(s) for s in values_str.split(',')]
            
            if len(speeds) == 4:
                joystick_data = JoystickData(lx=speeds[0], ly=speeds[1], rx=speeds[2], ry=speeds[3])
                motor_speeds = calculate_motor_speeds(joystick_data)
                print(
                    f"Setting motor speeds: "
                    f"FL={motor_speeds[0]:.2f}, FR={motor_speeds[1]:.2f}, "
                    f"RL={motor_speeds[2]:.2f}, RR={motor_speeds[3]:.2f}"
                )
            else:
                print(f"Error: Invalid number of joystick values received: {len(speeds)}")
        except (IndexError, ValueError) as e:
            print(f"Error parsing JOYSTICKS command: {cmd} -> {e}")
            
    elif command_name == "BTN":
        try:
            button_name = parts[1]
            action = parts[2]
            print(f"Button Event: {button_name} {action}")
        except IndexError as e:
            print(f"Error parsing BTN command: {cmd} -> {e}")

    else:
        print(f"Unknown command: {cmd}")

def handle_client(conn, addr):
    """
    This function runs in a separate thread for each connected client.
    """
    print(f"New client connected: {addr}")
    buffer = ""
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            
            buffer += data.decode('utf-8')
            
            while '\n' in buffer:
                command, buffer = buffer.split('\n', 1)
                handle_command(command)
                conn.sendall(b'ACK\n')

    except ConnectionResetError:
        print(f"Client {addr} forcefully disconnected.")
    except Exception as e:
        print(f"An error occurred with client {addr}: {e}")
    finally:
        print(f"Client disconnected: {addr}")
        is_stoped = True
        conn.close()

def main():
    """Main function to start the TCP server."""
    host = '0.0.0.0'
    port = 5000

    watchdog = threading.Thread(target=watchdog_thread, daemon=True)
    watchdog.start()
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((host, port))
        server_socket.listen()
        print(f"🤖 Robot TCP server listening on port {port}...")
        
        while True:
            is_stoped = False
            conn, addr = server_socket.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.daemon = True
            client_thread.start()

if __name__ == "__main__":
    main()