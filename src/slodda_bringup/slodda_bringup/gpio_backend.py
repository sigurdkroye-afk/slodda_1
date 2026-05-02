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
