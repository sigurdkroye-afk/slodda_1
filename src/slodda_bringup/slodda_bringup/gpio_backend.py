#!/usr/bin/env python3
"""
GPIO abstraction layer for motor driver.

Implement a concrete subclass of GpioBackend for your chosen GPIO library
(e.g. RPi.GPIO, gpiozero, lgpio, pigpio) and swap StubGpioBackend in
motor_driver_node.py for your implementation.
"""


class PwmHandle:
    """Returned by GpioBackend.setup_pwm(). Controls one PWM channel."""
    def start(self, duty_percent: float) -> None: ...
    def set_duty(self, duty_percent: float) -> None: ...
    def stop(self) -> None: ...


class GpioBackend:
    """Abstract GPIO interface. Do not instantiate directly."""
    def setup_output(self, pin: int) -> None: ...
    def setup_input(self, pin: int, pull_up: bool = False) -> None: ...
    def setup_pwm(self, pin: int, frequency_hz: int) -> PwmHandle: ...
    def write(self, pin: int, value: bool) -> None: ...
    def read(self, pin: int) -> bool: ...
    def attach_interrupt(self, pin: int, edge: str, callback) -> None:
        """edge: 'rising' | 'falling' | 'both'"""
        ...
    def cleanup(self) -> None: ...


class StubPwmHandle(PwmHandle):
    def __init__(self, pin: int, logger):
        self._pin = pin
        self._log = logger

    def start(self, duty_percent: float) -> None:
        self._log(f'  PWM pin={self._pin} START duty={duty_percent:.1f}%')

    def set_duty(self, duty_percent: float) -> None:
        self._log(f'  PWM pin={self._pin} duty={duty_percent:.1f}%')

    def stop(self) -> None:
        self._log(f'  PWM pin={self._pin} STOP')


class StubGpioBackend(GpioBackend):
    """
    No-hardware backend. Logs all calls. Safe to run anywhere.
    Replace with a real backend once GPIO library and pins are chosen.
    """
    def __init__(self, log_fn=None):
        self._log = log_fn or print

    def setup_output(self, pin):
        self._log(f'[GPIO stub] setup_output pin={pin}')

    def setup_input(self, pin, pull_up=False):
        self._log(f'[GPIO stub] setup_input pin={pin} pull_up={pull_up}')

    def setup_pwm(self, pin, frequency_hz):
        self._log(f'[GPIO stub] setup_pwm pin={pin} freq={frequency_hz}Hz')
        return StubPwmHandle(pin, self._log)

    def write(self, pin, value):
        self._log(f'[GPIO stub] write pin={pin} value={value}')

    def read(self, pin):
        self._log(f'[GPIO stub] read pin={pin} → 0')
        return False

    def attach_interrupt(self, pin, edge, callback):
        self._log(f'[GPIO stub] attach_interrupt pin={pin} edge={edge}')

    def cleanup(self):
        self._log('[GPIO stub] cleanup')


class RpiPwmHandle(PwmHandle):
    def __init__(self, pwm):
        self._pwm = pwm

    def start(self, duty_percent: float) -> None:
        self._pwm.start(duty_percent)

    def set_duty(self, duty_percent: float) -> None:
        self._pwm.ChangeDutyCycle(duty_percent)

    def stop(self) -> None:
        self._pwm.stop()


class RpiGpioBackend(GpioBackend):
    """RPi.GPIO backend for Raspberry Pi hardware. Requires RPi.GPIO installed."""

    _EDGE = {'rising': None, 'falling': None, 'both': None}

    def __init__(self):
        import RPi.GPIO as GPIO  # noqa: N813
        self._GPIO = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self._EDGE = {
            'rising':  GPIO.RISING,
            'falling': GPIO.FALLING,
            'both':    GPIO.BOTH,
        }

    def setup_output(self, pin: int) -> None:
        self._GPIO.setup(pin, self._GPIO.OUT, initial=self._GPIO.LOW)

    def setup_input(self, pin: int, pull_up: bool = False) -> None:
        pud = self._GPIO.PUD_UP if pull_up else self._GPIO.PUD_DOWN
        self._GPIO.setup(pin, self._GPIO.IN, pull_up_down=pud)

    def setup_pwm(self, pin: int, frequency_hz: int) -> RpiPwmHandle:
        self._GPIO.setup(pin, self._GPIO.OUT)
        return RpiPwmHandle(self._GPIO.PWM(pin, frequency_hz))

    def write(self, pin: int, value: bool) -> None:
        self._GPIO.output(pin, self._GPIO.HIGH if value else self._GPIO.LOW)

    def read(self, pin: int) -> bool:
        return bool(self._GPIO.input(pin))

    def attach_interrupt(self, pin: int, edge: str, callback) -> None:
        self._GPIO.add_event_detect(pin, self._EDGE[edge], callback=callback, bouncetime=2)

    def cleanup(self) -> None:
        self._GPIO.cleanup()


class LgpioPwmHandle(PwmHandle):
    def __init__(self, handle, pin: int, frequency_hz: int):
        self._h = handle
        self._pin = pin
        self._freq = frequency_hz

    def start(self, duty_percent: float) -> None:
        import lgpio
        lgpio.tx_pwm(self._h, self._pin, self._freq, duty_percent)

    def set_duty(self, duty_percent: float) -> None:
        import lgpio
        lgpio.tx_pwm(self._h, self._pin, self._freq, duty_percent)

    def stop(self) -> None:
        import lgpio
        lgpio.tx_pwm(self._h, self._pin, 0, 0)
        # Explicitly drive pin LOW so motor gets no power if process dies mid-cycle
        lgpio.gpio_claim_output(self._h, self._pin, 0)


class LgpioBackend(GpioBackend):
    """lgpio backend — uses /dev/gpiochip0, no root required (user must be in dialout group)."""

    def __init__(self, chip: int = 0):
        import lgpio
        self._lg = lgpio
        self._h = lgpio.gpiochip_open(chip)
        self._callbacks = []  # must retain; GC cancels lgpio callbacks
        self._EDGE = {
            'rising':  lgpio.RISING_EDGE,
            'falling': lgpio.FALLING_EDGE,
            'both':    lgpio.BOTH_EDGES,
        }

    def setup_output(self, pin: int) -> None:
        self._lg.gpio_claim_output(self._h, pin, 0)

    def setup_input(self, pin: int, pull_up: bool = False) -> None:
        flags = self._lg.SET_PULL_UP if pull_up else self._lg.SET_PULL_NONE
        self._lg.gpio_claim_input(self._h, pin, flags)

    def setup_pwm(self, pin: int, frequency_hz: int) -> LgpioPwmHandle:
        self._lg.gpio_claim_output(self._h, pin, 0)
        return LgpioPwmHandle(self._h, pin, frequency_hz)

    def write(self, pin: int, value: bool) -> None:
        self._lg.gpio_write(self._h, pin, 1 if value else 0)

    def read(self, pin: int) -> bool:
        return bool(self._lg.gpio_read(self._h, pin))

    def attach_interrupt(self, pin: int, edge: str, callback) -> None:
        # gpio_claim_input doesn't enable edge alerts — must re-claim as alert
        self._lg.gpio_claim_alert(self._h, pin, self._EDGE[edge])
        cb = self._lg.callback(self._h, pin, self._EDGE[edge], callback)
        self._callbacks.append(cb)

    def cleanup(self) -> None:
        for cb in self._callbacks:
            cb.cancel()
        self._callbacks.clear()
        self._lg.gpiochip_close(self._h)
