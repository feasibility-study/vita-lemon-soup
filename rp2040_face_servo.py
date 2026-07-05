"""
MicroPython servo controller for an RP2040.

Connect the RP2040 to the PC over USB. The PC script sends lines like:
    FACE 1 -0.250
    FACE 0 0.000

FACE 1 means a face is active, and the offset is normalized from -1.0 to 1.0
across the camera frame. The servo centers at 90 degrees when offset is 0.
"""

import sys
import time

import uselect
from machine import Pin, PWM


SERVO_PIN = 1
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_CENTER_ANGLE = 90.0
SERVO_SPAN_ANGLE = 70.0
NO_FACE_TIMEOUT_MS = 1000
POLL_TIMEOUT_MS = 50


class Servo:
    def __init__(self, pin_number):
        self.pwm = PWM(Pin(pin_number))
        self.pwm.freq(50)
        self.angle = None
        self.center()

    def _us_to_duty_u16(self, pulse_us):
        return int(pulse_us * 65535 / 20000)

    def set_angle(self, angle):
        angle = max(0.0, min(180.0, float(angle)))
        if self.angle is not None and abs(self.angle - angle) < 0.5:
            return

        pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180.0)
        self.pwm.duty_u16(self._us_to_duty_u16(pulse_us))
        self.angle = angle

    def center(self):
        self.set_angle(SERVO_CENTER_ANGLE)

    def set_offset(self, offset):
        offset = max(-1.0, min(1.0, float(offset)))
        self.set_angle(SERVO_CENTER_ANGLE + offset * SERVO_SPAN_ANGLE)


def parse_command(line):
    parts = line.strip().split()
    if len(parts) != 3:
        return None
    if parts[0].upper() != "FACE":
        return None

    try:
        face_present = parts[1] == "1"
        offset = float(parts[2])
    except ValueError:
        return None

    return face_present, offset


def main():
    servo = Servo(SERVO_PIN)
    poller = uselect.poll()
    poller.register(sys.stdin, uselect.POLLIN)

    last_message_ms = time.ticks_ms()
    print("RP2040 servo ready. Waiting for FACE commands over USB serial.")

    while True:
        if poller.poll(POLL_TIMEOUT_MS):
            line = sys.stdin.readline()
            if line:
                command = parse_command(line)
                if command is not None:
                    face_present, offset = command
                    last_message_ms = time.ticks_ms()
                    if face_present:
                        servo.set_offset(offset)
                    else:
                        servo.center()

        if time.ticks_diff(time.ticks_ms(), last_message_ms) > NO_FACE_TIMEOUT_MS:
            servo.center()

        time.sleep_ms(10)


main()