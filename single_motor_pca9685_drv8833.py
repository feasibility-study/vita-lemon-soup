"""
Very small MicroPython demo for one DC motor controlled through a PCA9685
and a DRV8833 driver.

Wiring idea:
- ESP32-S3 I2C SDA/SCL -> PCA9685 SDA/SCL
- PCA9685 channel 0 -> DRV8833 IN1
- PCA9685 channel 1 -> DRV8833 IN2
- DRV8833 motor outputs -> one DC motor
- All grounds common
- DRV8833 VM gets motor supply, VCC gets logic supply as required

This script assumes:
- channel 0 = forward input
- channel 1 = reverse input

For a DRV8833, this is better than servo-style 50 Hz PWM.
Use a few kHz PWM on the PCA9685.
"""

from machine import Pin, I2C
import time


PCA9685_ADDR = 0x40
PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06

I2C_SDA = 5
I2C_SCL = 6
I2C_ID = 0
I2C_FREQ = 400_000
PWM_FREQ = 1000

MOTOR_FORWARD_CH = 0
MOTOR_REVERSE_CH = 1


def clamp(value, low, high):
    return max(low, min(high, value))


class PCA9685:
    def __init__(self, i2c, address=PCA9685_ADDR):
        self.i2c = i2c
        self.address = address
        self.write_reg(PCA9685_MODE1, 0x00)
        time.sleep_ms(10)
        self.set_pwm_freq(PWM_FREQ)

    def write_reg(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value & 0xFF]))

    def read_reg(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def set_pwm_freq(self, freq_hz):
        prescale = int(round(25_000_000 / (4096 * freq_hz)) - 1)
        old_mode = self.read_reg(PCA9685_MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self.write_reg(PCA9685_MODE1, sleep_mode)
        self.write_reg(PCA9685_PRESCALE, prescale)
        self.write_reg(PCA9685_MODE1, old_mode)
        time.sleep_ms(5)
        self.write_reg(PCA9685_MODE1, old_mode | 0x80)

    def set_channel(self, channel, duty):
        duty = clamp(float(duty), 0.0, 1.0)
        off_count = int(duty * 4095)
        base = PCA9685_LED0_ON_L + 4 * channel
        self.i2c.writeto_mem(
            self.address,
            base,
            bytes([
                0,
                0,
                off_count & 0xFF,
                (off_count >> 8) & 0x0F,
            ]),
        )

    def off(self, channel):
        self.i2c.writeto_mem(self.address, PCA9685_LED0_ON_L + 4 * channel, b"\x00\x00\x00\x00")


class Motor:
    def __init__(self, pca, forward_ch, reverse_ch):
        self.pca = pca
        self.forward_ch = forward_ch
        self.reverse_ch = reverse_ch
        self.stop()

    def stop(self):
        self.pca.off(self.forward_ch)
        self.pca.off(self.reverse_ch)

    def forward(self, speed=1.0):
        speed = clamp(speed, 0.0, 1.0)
        self.pca.set_channel(self.reverse_ch, 0)
        self.pca.set_channel(self.forward_ch, speed)

    def reverse(self, speed=1.0):
        speed = clamp(speed, 0.0, 1.0)
        self.pca.set_channel(self.forward_ch, 0)
        self.pca.set_channel(self.reverse_ch, speed)



def main():
    i2c = I2C(I2C_ID, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=I2C_FREQ)
    pca = PCA9685(i2c)
    motor = Motor(pca, MOTOR_FORWARD_CH, MOTOR_REVERSE_CH)

    print("Running single motor demo")

    for _ in range(3):
        motor.forward(0.8)
        time.sleep(1)
        motor.stop()
        time.sleep(0.5)
        motor.reverse(0.8)
        time.sleep(1)
        motor.stop()
        time.sleep(0.5)

    motor.stop()
    print("Done")


main()
