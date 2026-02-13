from gpiozero import PhaseEnableMotor, OutputDevice, RotaryEncoder, Servo
from time import time

class PwmMotor:
    def __init__(self, pwmPin: int, dirPin: int, isPwm: bool = True):
        self.motor: PhaseEnableMotor = PhaseEnableMotor(phase = dirPin, enable = pwmPin, pwm = isPwm)
    
    def set_speed(self, speed: int) -> None:
        """sets the speed of a motor

        Args:
            speed (int): the speed to set the motor to 0 = off, + = forward, - = backward (all values bw)
        """
        self.motor.value = speed
    
    def full_forward(self) -> None:
        self.motor.forward()

    def full_backward(self) -> None:
        self.motor.backward()


class EncoderMotor:
    def __init__(self, pwmPin: int, dirPin: int, encA: int, encB: int):
        self.motor: PhaseEnableMotor = PhaseEnableMotor(phase = dirPin, enable = pwmPin)
        self.encoder: RotaryEncoder = RotaryEncoder(encA, encB, max_steps=0)
        self.encoder.steps = 0
        self.min_position = None
        self.max_position = None
        self.position_tolerance = 20
        self.target_position = None
        self.default_speed = 0.4

    def set_speed(self, speed: int) -> None:
        """sets the speed of a motor

        Args:
            speed (int): the speed to set the motor to 0 = off, + = forward, - = backward (all values bw)
        """
        self.motor.value = speed

    def get_current_position(self) -> int:
        """returns the current position of the encoder"""
        return self.encoder.steps
    
    def set_min_position(self, min_position: int) -> None:
        """sets the minimum position of the encoder"""
        self.min_position = min_position

    def set_max_position(self, max_position: int) -> None:
        """sets the maximum position of the encoder"""
        self.max_position = max_position

    def min_pos(self) -> None:
        """Move the motor to the minimum position"""
        if self.min_position is None:
            raise ValueError("Minimum position not set")
        self.move_to_position(self.min_position)

    def max_pos(self) -> None:
        """Move the motor to the maximum position"""
        self.move_to_position(self.max_position)
    
    def move_to_position(self, target_position: int, speed: float = None) -> None:
        """moves the motor to the target position

        Args:
            target_position (int): the position to move the motor to
            speed (float, optional): the speed to move the motor at. Defaults to None.
        """
        if self.min_position is not None and target_position < self.min_position:
            raise ValueError("Target position is less than minimum position")
        if self.max_position is not None and target_position > self.max_position:
            raise ValueError("Target position is greater than maximum position")
        
        self.target_position = target_position
        if speed is None:
            speed = self.default_speed
        self._update_movement(speed)

    def move_steps(self, steps: int, speed: float = None) -> None:
        """moves the motor the desired number of steps

        Args:
            steps (int): the number of steps to move
            speed (float, optional): the speed to move the motor at. Defaults to None.
        """
        self.target_position = self.get_current_position() + steps
        if self.min_position is not None and self.target_position < self.min_position:
            raise ValueError("Target position is less than minimum position")
        if self.max_position is not None and self.target_position > self.max_position:
            raise ValueError("Target position is greater than maximum position")
        
        if speed is None:
            speed = self.default_speed
        self._update_movement(speed)

    def _update_movement(self, speed: float) -> None:
        """updates the movement of the motor based on the current and target position"""
        current_position = self.get_current_position()
        if self.target_position is None:
            self.set_speed(0)
            return

        distance_to_target = current_position - self.target_position

        if abs(distance_to_target) <= self.position_tolerance:
            self.set_speed(0)
            return
        
        direction = 1 if distance_to_target > 0 else -1
        self.set_speed(direction * speed)

    async def update(self) -> None:
        """updates the motor position"""
        self._update_movement(self.default_speed)


class ServoMotor:
    def __init__(self, pin: int):
        self.servo: Servo = Servo(pin)
        self.end_time = 0
        self.stop()
        
    def stop(self):
        """Stops the servo"""
        self.servo.value = None

    def set_min(self) -> None:
        """sets the servo to its minimum position"""
        self.servo.value = -1

    def set_max(self) -> None:
        """sets the servo to its maximum position"""
        self.servo.value = 0

    def set_value(self, value: float) -> None:
        """sets the servo to the desired value

        Args:
            value (float): the value to set the servo to (0-1)
        """
        self.servo.value = value

    def move(self, speed: float, length: float) -> None:
        print("Got here")
        self.servo.value = speed
        self.end_time = time() + length

    async def update(self) -> None:
        if time() >= self.end_time:
            self.stop()
