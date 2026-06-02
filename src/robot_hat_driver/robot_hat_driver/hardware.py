from .servo import Servo
from .motor import Motors
from .adc import ADC


class RobotHatHardware:
    def __init__(self, params):
        self.motors = Motors(params["lmid"], params["rmid"], params["lmrev"], params["rmrev"])
        self.servo1 = Servo("P0")
        self.servo2 = Servo("P1")
        self.battery_adc = ADC("A4")

    def set_motor_speeds(self, left: float, right: float):
        lefts = max(-100.0, min(100.0, left))
        rights = max(-100.0, min(100.0, right))
        self.motors.left.speed(lefts)
        self.motors.right.speed(rights)

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