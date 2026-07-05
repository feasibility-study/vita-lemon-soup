import machine
from pca9685 import PCA9685Driver
import time
import math

i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
pwm = PCA9685Driver(i2c)
pwm.set_pwm_frequency(50) # Set frequency to 50Hz for servos


pwm.servo_set_angle(0, 90) # Channel 0, angle 90°
time.sleep(1)
pwm.servo_set_angle(0, 0) # Channel 0, angle 0°
time.sleep(1)






# Motion profile parameters
ACCEL = 200          # %/s
DECEL = 400          # %/s (2x faster than accel)
MAX_SPEED = 100      # %
UPDATE_RATE = 0.01   # s


def set_motor(motor_id, percentage):
    """
    Set motor speed.
    Positive = forward
    Negative = reverse
    """

    forward = 2 + motor_id * 2
    reverse = forward + 1

    if percentage >= 0:
        pwm.set_pwm_dc_percent(forward, percentage)
        pwm.set_pwm_dc_percent(reverse, 0)
    else:
        pwm.set_pwm_dc_percent(forward, 0)
        pwm.set_pwm_dc_percent(reverse, -percentage)


def move_motor(motor_id, distance):
    """
    Move a motor a signed distance using a trapezoidal motion profile.

    distance:
        > 0 = forward
        < 0 = reverse

    Distance is in arbitrary units to be calibrated.
    """

    direction = 1 if distance >= 0 else -1
    distance = abs(distance)

    # Peak speed for a triangular profile
    peak_speed = math.sqrt((2 * distance * ACCEL * DECEL) /
                           (ACCEL + DECEL))

    if peak_speed > MAX_SPEED:
        peak_speed = MAX_SPEED

        t_accel = peak_speed / ACCEL
        t_decel = peak_speed / DECEL

        d_accel = 0.5 * ACCEL * t_accel**2
        d_decel = 0.5 * DECEL * t_decel**2

        cruise_distance = distance - d_accel - d_decel
        t_cruise = cruise_distance / peak_speed
    else:
        t_accel = peak_speed / ACCEL
        t_decel = peak_speed / DECEL
        t_cruise = 0

    # Accelerate
    t = 0
    while t < t_accel:
        speed = ACCEL * t
        set_motor(motor_id, direction * speed)
        time.sleep(UPDATE_RATE)
        t += UPDATE_RATE

    # Cruise
    t = 0
    while t < t_cruise:
        set_motor(motor_id, direction * peak_speed)
        time.sleep(UPDATE_RATE)
        t += UPDATE_RATE

    # Decelerate
    t = 0
    while t < t_decel:
        speed = peak_speed - DECEL * t
        set_motor(motor_id, direction * max(speed, 0))
        time.sleep(UPDATE_RATE)
        t += UPDATE_RATE

    set_motor(motor_id, 0)
    
    
    

# Wheel groups
LEFT = [0, 2]
RIGHT = [1, 3]


def normalize_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def shortest_angle(target):
    """
    Ensures we rotate via shortest path.
    (Negative = CW, Positive = CCW)
    """
    return normalize_angle(target)


def rotate(angle):
    """
    Rotate robot in place.

    Positive angle = CCW
    Negative angle = CW
    """

    # Left side goes opposite right side
    for m in LEFT:
        move_motor(m, -angle)

    for m in RIGHT:
        move_motor(m, angle)


def drive_straight(distance):
    """
    Drive forward OR backward depending on sign.
    All wheels move same signed distance.
    """

    for m in LEFT + RIGHT:
        move_motor(m, distance)


def move_robot(dx, dy, dtheta):
    """
    Skid-steer navigation:

    1. rotate toward travel direction
    2. drive straight (can be forward or reverse)
    3. rotate to final heading
    """

    # --- compute translation ---
    distance = math.hypot(dx, dy)

    if distance < 1e-6:
        # No translation, only rotation
        rotate(dtheta)
        return

    # Desired heading for translation
    target_heading = math.atan2(dy, dx)

    # --- Phase 1: rotate to face target ---
    rotate(shortest_angle(target_heading))

    # --- Phase 2: drive straight ---
    # move_motor already handles reverse if needed
    drive_straight(distance)

    # --- Phase 3: final orientation ---
    rotate(shortest_angle(dtheta))