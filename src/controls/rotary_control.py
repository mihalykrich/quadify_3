import logging
import time
import RPi.GPIO as GPIO
from .gpio_setup_module import GPIOSetup  # Assumes proper setup with pull-up

class RotaryControl:
    def __init__(
        self,
        gpio_setup=None,
        rotation_callback=None,
        button_callback=None,
        long_press_callback=None,
        long_press_threshold=2.5
    ):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.DEBUG)

        if gpio_setup is None:
            self.gpio_setup = GPIOSetup(clk_pin=13, dt_pin=5, sw_pin=6)
        else:
            self.gpio_setup = gpio_setup

        self.rotation_callback = rotation_callback
        self.button_callback = button_callback
        self.long_press_callback = long_press_callback
        self.long_press_threshold = long_press_threshold

        self.CLK_PIN = self.gpio_setup.CLK_PIN
        self.DT_PIN = self.gpio_setup.DT_PIN
        self.SW_PIN = self.gpio_setup.SW_PIN

        self.last_encoded = self._read_encoder()
        self.full_cycle = 0
        self.button_last_state = self._read_button_state()

        self.logger.debug("RotaryControl initialized.")

    def _read_encoder(self):
        clk_state = GPIO.input(self.CLK_PIN)
        dt_state = GPIO.input(self.DT_PIN)
        return (clk_state << 1) | dt_state

    def _read_button_state(self):
        return GPIO.input(self.SW_PIN)

    def start(self):
        self.logger.debug("RotaryControl listening for events.")
        try:
            self.last_encoded = self._read_encoder()

            while True:
                # --- Rotary encoder logic ---
                current_encoded = self._read_encoder()
                if current_encoded != self.last_encoded:
                    if (self.last_encoded == 0b00 and current_encoded == 0b10) or \
                       (self.last_encoded == 0b10 and current_encoded == 0b11) or \
                       (self.last_encoded == 0b11 and current_encoded == 0b01) or \
                       (self.last_encoded == 0b01 and current_encoded == 0b00):
                        self.full_cycle += 1
                    elif (self.last_encoded == 0b00 and current_encoded == 0b01) or \
                         (self.last_encoded == 0b01 and current_encoded == 0b11) or \
                         (self.last_encoded == 0b11 and current_encoded == 0b10) or \
                         (self.last_encoded == 0b10 and current_encoded == 0b00):
                        self.full_cycle -= 1

                    if abs(self.full_cycle) == 4:
                        direction = 1 if self.full_cycle > 0 else -1
                        self.logger.debug(f"Rotated direction: {direction}")
                        if self.rotation_callback:
                            self.rotation_callback(direction)
                        self.full_cycle = 0

                    self.last_encoded = current_encoded

                # --- Button logic with debounce ---
                button_state = self._read_button_state()
                if button_state == GPIO.LOW and self.button_last_state == GPIO.HIGH:
                    time.sleep(0.05)  # Debounce delay
                    if GPIO.input(self.SW_PIN) == GPIO.LOW:
                        press_start_time = time.time()
                        while GPIO.input(self.SW_PIN) == GPIO.LOW:
                            time.sleep(0.01)
                            if time.time() - press_start_time > self.long_press_threshold:
                                if self.long_press_callback:
                                    self.logger.debug("Long button press detected.")
                                    self.long_press_callback()
                                break
                        else:
                            if time.time() - press_start_time < self.long_press_threshold:
                                if self.button_callback:
                                    self.logger.debug("Short button press detected.")
                                    self.button_callback()

                self.button_last_state = button_state
                time.sleep(0.01)

        except KeyboardInterrupt:
            self.logger.info("RotaryControl interrupted by user.")
            self.stop()

    def stop(self):
        self.gpio_setup.cleanup()
        self.logger.info("GPIO cleanup complete.")
