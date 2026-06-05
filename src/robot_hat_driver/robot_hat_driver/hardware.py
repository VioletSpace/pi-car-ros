import os

from .servo import Servo
from .motor import Motors
from .adc import ADC


class RobotHatHardware:
    def __init__(self, params, logger):
        self.logger = logger

        if not os.path.exists("/dev/i2c-1"):
            self.logger.error("I2C bus missing (/dev/i2c-1)  - incorrect environment")
            raise RuntimeError("I2C bus not available")
        if not os.path.exists("/dev/gpiomem") and not os.path.exists("/dev/gpiochip0"):
            self.logger.error("GPIO interface missing (/dev/gpiomem or /dev/gpiochip0) - incorrect environment")
            raise RuntimeError("GPIO not available")
        
        self.motors = Motors(params["lmid"], params["rmid"], params["lmrev"], params["rmrev"])
        self.servos = [Servo(ch) for ch in params["servo_channels"]]
        self.battery_adc = ADC("A4")

        self.logger.info("Hardware ready")

    def set_motor_speeds(self, left: float, right: float):
        lefts = max(-100.0, min(100.0, left))
        rights = max(-100.0, min(100.0, right))
        self.motors.left.speed(lefts)
        self.motors.right.speed(rights)

    def stop(self):
        self.set_motor_speeds(0.0, 0.0)

    def set_servo(self, channel: int, angle: float):
        if channel < len(self.servos) and channel >= 0:
            self.servos[channel].angle(angle)
        else:
            self.logger.warn("Invalid servo index %d of %d" % (channel, len(self.servos)))

    def battery_voltage(self):
        raw = self.battery_adc.read()
        return float(raw)