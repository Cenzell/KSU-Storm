import sys
import socket
import time
import pygame
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer
from PyQt6 import uic
import comm

GAMEPAD_POLL_RATE_MS = 20
PING_INTERVAL_S = 1
    
class AppWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("driver_station.ui", self)

        self.joystick = None
        self.init_pygame_and_joystick()

        self.conn_manager = comm.ConnectionManager()
        self.conn_manager.signals.connection_status.connect(self.update_connection_status)
        self.conn_manager.start()

        self.telemetry_receiver = comm.TelemetryReceiver(self.conn_manager)
        self.telemetry_receiver.signals.ping_response.connect(self.handle_ping_response)
        self.telemetry_receiver.start()
        
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.poll_gamepad)
        self.gamepad_timer.start(GAMEPAD_POLL_RATE_MS)

        self.last_ping_time = 0
        self.ping_sent_time = None
        self.joystick_values = {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0}
        self.last_sent_joystick_values = self.joystick_values.copy()
    
    def handle_ping_response(self):
        if self.ping_sent_time:
            rtt_ms = (time.time() - self.ping_sent_time) * 1000
            self.ping_label.setText(f"Ping: {rtt_ms:.1f} ms")
            self.ping_sent_time = None

    def init_pygame_and_joystick(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.gamepad_label.setText(f"Gamepad: {self.joystick.get_name()}")
            print(f"Found joystick: {self.joystick.get_name()}")
        else:
            self.gamepad_label.setText("Gamepad: Not Found")
            print("No joystick found.")

    def update_connection_status(self, is_connected, address):
        if is_connected:
            self.status_label.setText("Status: <b style='color: green;'>Connected</b>")
            self.address_label.setText(f"Address: {address}")
        else:
            self.status_label.setText("Status: <b style='color: red;'>Disconnected</b>")
            self.address_label.setText("Address: N/A")
            self.ping_label.setText("Ping: -- ms")
            self.button_a_label.setStyleSheet("color: lightgray")
            self.button_b_label.setStyleSheet("color: lightgray")
            self.button_x_label.setStyleSheet("color: lightgray")
            self.button_y_label.setStyleSheet("color: lightgray")

    def send_to_robot(self, message):
        sock = self.conn_manager.get_socket()
        if sock:
            try:
                sock.sendall(message.encode('utf-8'))
            except (socket.error, OSError) as e:
                print(f"Send error: {e}. Disconnecting.")
                self.conn_manager.disconnect()

    def poll_gamepad(self):
        if self.joystick is None:
            return

        pygame.event.pump()
        deadzone = 0.03
        self.joystick_values['lx'] = self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > deadzone else 0.0
        self.joystick_values['ly'] = -self.joystick.get_axis(1) if abs(self.joystick.get_axis(1)) > deadzone else 0.0
        self.joystick_values['rx'] = self.joystick.get_axis(2) if abs(self.joystick.get_axis(2)) > deadzone else 0.0
        self.joystick_values['ry'] = -self.joystick.get_axis(4) if abs(self.joystick.get_axis(4)) > deadzone else 0.0

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                self.send_to_robot(f"BTN {event.button} DOWN\n")
                if event.button == 0: self.button_a_label.setStyleSheet("color: green")
                if event.button == 1: self.button_b_label.setStyleSheet("color: red")
                if event.button == 2: self.button_x_label.setStyleSheet("color: blue")
                if event.button == 3: self.button_y_label.setStyleSheet("color: purple")
            if event.type == pygame.JOYBUTTONUP:
                self.send_to_robot(f"BTN {event.button} UP\n")
                if event.button == 0: self.button_a_label.setStyleSheet("color: lightgray")
                if event.button == 1: self.button_b_label.setStyleSheet("color: lightgray")
                if event.button == 2: self.button_x_label.setStyleSheet("color: lightgray")
                if event.button == 3: self.button_y_label.setStyleSheet("color: lightgray")

        self.lx_label.setText(f"LX: {self.joystick_values['lx']:.2f}")
        self.ly_label.setText(f"LY: {self.joystick_values['ly']:.2f}")
        self.rx_label.setText(f"RX: {self.joystick_values['rx']:.2f}")
        self.ry_label.setText(f"RY: {self.joystick_values['ry']:.2f}")

        if self.joystick_values != self.last_sent_joystick_values:
            msg = (f"JOYSTICKS {self.joystick_values['lx']},{self.joystick_values['ly']},"
                   f"{self.joystick_values['rx']},{self.joystick_values['ry']}\n")
            self.send_to_robot(msg)
            self.last_sent_joystick_values = self.joystick_values.copy()

        if time.time() - self.last_ping_time > PING_INTERVAL_S:
            self.ping_sent_time = time.time()
            self.send_to_robot("PING\n")
            self.last_ping_time = time.time()

    def closeEvent(self, event):
        print("Closing application...")
        self.telemetry_receiver.stop()
        self.conn_manager.stop()
        self.telemetry_receiver.join()
        self.conn_manager.join()
        pygame.quit()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = AppWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()