import os
import time

from .servo import Servo
from .motor import Motors
from .adc import ADC
from .pin import Pin
from . import utils
from .modules import Grayscale_Module, Ultrasonic


class RobotHatHardware:
    def __init__(self, params, logger):
        self.logger = logger
        self.params = params

        # Sanity checks for hardware availability
        if not os.path.exists("/dev/i2c-1"):
            self.logger.error("I2C bus missing (/dev/i2c-1)  - incorrect environment")
            raise RuntimeError("I2C bus not available")
        if not (os.path.exists("/dev/gpiomem") and os.path.exists("/dev/gpiochip0")):
            self.logger.error("GPIO interface missing (/dev/gpiomem or /dev/gpiochip0) - incorrect environment")
            raise RuntimeError("GPIO not available")
        
        # Initialise hardware
        utils.reset_mcu() # Needed to reset broken states at boot
        time.sleep(0.2)
        
        self.motors = Motors(params["lmid"], params["rmid"], params["lmrev"], params["rmrev"])
        try: self.servos = [Servo(ch) for ch in params["servo_channels"]]
        except ValueError as err:
            self.logger.error(f"Could not initialise Servos: {err}")
            self.servos = []
        else: self.logger.info("Servos available")
        self.battery_adc = ADC("A4")
        self._led_active = False
        self._led = Pin('LED')
        self._usr_btn = Pin('USER', mode=Pin.IN, pull=Pin.PULL_UP, active_state=True)
        self._usr_btn_pressed = False
        self._rst_btn = Pin('RST', mode=Pin.IN, pull=Pin.PULL_UP, active_state=True)
        self._rst_btn_pressed = False

        if params['us_s']:
            try: self.ultrasonic = Ultrasonic(Pin(params["us_pins"][0]), Pin(params["us_pins"][1], mode=Pin.IN, pull=Pin.PULL_DOWN))
            except ValueError as err:
                self.logger.error(f"Could not initialise Sonar sensor, deactivating: {err}")
                self.params['us_s'] = False
            else: self.logger.info("Sonar sensor available")
        if params['gs_s']:
            try:
                adc0, adc1, adc2 = [ADC(pin) for pin in params['gs_pins']]
                self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
            except ValueError as err:
                self.logger.error(f"Could not initialise Grayscale sensor, deactivating: {err}")
                self.params['gs_s'] = False
            else: self.logger.info("Grayscale sensor available")

        self.logger.info("Hardware ready")
    
    def led(self, status=None):
        """Sets the indicator LED to status. If no status supplied, returns current status"""
        if status==None:
            return self._led_active
        self._led_active = status
        if self._led_active:
            self._led.on()
        else:
            self._led.off()

    def set_motor_speeds(self, left: float, right: float):
        self.motors.left.speed(left)
        self.motors.right.speed(right)

    def stop(self):
        """Set motors to 0, resets"""
        self.set_motor_speeds(0.0, 0.0)
        utils.reset_mcu()

    def set_servo(self, channel: int, angle: float):
        """Set the servo at channel to angle. Applies corrections defined in the `servo_correction` ROS2 parameter"""
        if channel < len(self.servos) and channel >= 0:
            self.servos[channel].angle(angle + self.params["servo_corr"][channel])
        else:
            self.logger.warn(f"Invalid servo index {channel} of {len(self.servos)}, ignoring.")

    def battery_voltage(self):
        raw = self.battery_adc.read_voltage()
        return float(raw * 3)
    
    def charge_perc(volt, n_cells=1):
        chargecurve = {
          4.2: 1.0,
          3.975: 0.55,
          3.9: 0.3,
          3.8: 0.2,
          3.6: 0.11,
          3.3: 0.0
        }
        v = volt / n_cells
        if v >= max(chargecurve.keys()): return 1.0
        if v <= min(chargecurve.keys()): return   0.0
        t = min([k for k in chargecurve.keys() if v <= k])
        b = max([k for k in chargecurve.keys() if v >= k])
        if t == b: return chargecurve[b]
        p = (v-b)/(t-b)*(chargecurve[t]-chargecurve[b])+chargecurve[b]
        return p
    
    def usr_button_pressed(self):
        """Has the USR button just been released from a press?"""
        cpressed = self._usr_btn.value() == 1
        res = self._usr_btn_pressed and not cpressed
        self._usr_btn_pressed = cpressed
        return res
    
    def rst_button_pressed(self):
        """Has the RST button just been released from a press?"""
        cpressed = self._rst_btn.value() == 1
        res = self._rst_btn_pressed and not cpressed
        self._rst_btn_pressed = cpressed
        return res