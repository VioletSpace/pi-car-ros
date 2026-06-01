try:
    from robot_hat import Motors, Servo, ADC
except ImportError as exc:
    raise RuntimeError(
        "robot_hat package not installed."
        "Install the SunFounder Robot HAT software first (https://github.com/sunfounder/robot-hat)."
    ) from exc

class RobotHatHardware:
    def __init__(self):
        motors = Motors()
        # Setup left and right motors
        motors.set_left_id(1)
        motors.set_right_id(2)
        self.left_motor = motors[1]
        self.right_motor = motors[2]

        self.servo1 = Servo("P0")
        self.servo2 = Servo("P1")

        self.battery_adc = ADC("A4")

    def set_motor_speeds(self, left: float, right: float):
        left = max(-100.0, min(100.0, left))
        right = max(-100.0, min(100.0, right))

        self.left_motor.speed(left)
        self.right_motor.speed(right)

    def stop(self):
        self.set_motor_speeds(0.0, 0.0)

    def set_servo(self, channel: int, angle: float):
        angle = max(-90.0, min(90.0, angle))

        if channel == 1:
            self.servo1.angle(angle)
        elif channel == 2:
            self.servo2.angle(angle)

    def battery_voltage(self):
        raw = self.battery_adc.read()
        return float(raw)