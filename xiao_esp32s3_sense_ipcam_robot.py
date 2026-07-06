"""
ESP32-S3 Sense MicroPython server for IP camera + motor control.

What it does:
- Creates a Wi-Fi access point
- Streams the camera as MJPEG at /stream
- Accepts motor commands from the laptop at /move
- Stops the motors if commands stop arriving

Motor setup:
- PCA9685 board driving a DRV8833 motor driver
- Four motor slots are supported
- If one motor is just a backup, set its enabled flag to False below

Typical URLs:
- http://192.168.4.1:8080/
- http://192.168.4.1:8080/stream
- http://192.168.4.1:8080/move?left=0.2&right=0.3&present=1
"""

import time
import uasyncio as asyncio
import network
from machine import I2C, Pin

try:
    import camera
except ImportError:
    camera = None


AP_SSID = "XIAO-IPCAM"
AP_PASSWORD = "12345678"
HTTP_PORT = 8080
COMMAND_TIMEOUT_MS = 1200
STREAM_DELAY_MS = 70

I2C_ID = 0
I2C_SDA_PIN = 5
I2C_SCL_PIN = 6
I2C_FREQ = 400000
PCA9685_ADDR = 0x40
PCA9685_PWM_FREQ = 1000

# Four motor slots, each with two PCA9685 channels.
# Left side motors are 0 and 1, right side motors are 2 and 3.
# If one motor is just a backup, set its enabled flag to False.
MOTOR_SLOTS = [
    {"forward": 0, "reverse": 1, "enabled": True},
    {"forward": 2, "reverse": 3, "enabled": True},
    {"forward": 4, "reverse": 5, "enabled": True},
    {"forward": 6, "reverse": 7, "enabled": True},
]
LEFT_MOTOR_IDS = [0, 1]
RIGHT_MOTOR_IDS = [2, 3]

last_command_ms = time.ticks_ms()


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class PCA9685:
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, i2c, address=PCA9685_ADDR):
        self.i2c = i2c
        self.address = address
        self.pwm_freq = PCA9685_PWM_FREQ
        self.i2c.writeto_mem(self.address, self.MODE1, b"\x00")
        time.sleep_ms(10)
        self.set_pwm_freq(self.pwm_freq)

    def _write_reg(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value & 0xFF]))

    def _read_reg(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def set_pwm_freq(self, freq_hz):
        self.pwm_freq = freq_hz
        prescale = int(round(25000000 / (4096 * freq_hz)) - 1)
        old_mode = self._read_reg(self.MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self._write_reg(self.MODE1, sleep_mode)
        self._write_reg(self.PRESCALE, prescale)
        self._write_reg(self.MODE1, old_mode)
        time.sleep_ms(5)
        self._write_reg(self.MODE1, old_mode | 0x80)

    def set_channel_duty(self, channel, duty):
        duty = clamp(float(duty), 0.0, 1.0)
        off_count = int(duty * 4095)
        base = self.LED0_ON_L + 4 * channel
        self.i2c.writeto_mem(
            self.address,
            base,
            bytes([0, 0, off_count & 0xFF, (off_count >> 8) & 0x0F]),
        )

    def off(self, channel):
        self.i2c.writeto_mem(self.address, self.LED0_ON_L + 4 * channel, b"\x00\x00\x00\x00")


class Motor:
    def __init__(self, pca, forward_ch, reverse_ch, enabled=True):
        self.pca = pca
        self.forward_ch = forward_ch
        self.reverse_ch = reverse_ch
        self.enabled = enabled
        self.stop()

    def stop(self):
        self.pca.off(self.forward_ch)
        self.pca.off(self.reverse_ch)

    def drive(self, speed):
        if not self.enabled:
            self.stop()
            return

        speed = clamp(float(speed), -1.0, 1.0)
        duty = abs(speed)
        if duty <= 0:
            self.stop()
            return

        if speed > 0:
            self.pca.set_channel_duty(self.reverse_ch, 0)
            self.pca.set_channel_duty(self.forward_ch, duty)
        else:
            self.pca.set_channel_duty(self.forward_ch, 0)
            self.pca.set_channel_duty(self.reverse_ch, duty)


class MotorBank:
    def __init__(self, pca):
        self.motors = []
        for slot in MOTOR_SLOTS:
            self.motors.append(Motor(pca, slot["forward"], slot["reverse"], slot["enabled"]))

    def stop_all(self):
        for motor in self.motors:
            motor.stop()

    def drive_side(self, motor_ids, speed):
        for motor_id in motor_ids:
            self.motors[motor_id].drive(speed)

    def drive(self, left_speed, right_speed):
        self.drive_side(LEFT_MOTOR_IDS, left_speed)
        self.drive_side(RIGHT_MOTOR_IDS, right_speed)


class CameraShim:
    def __init__(self):
        self.module = camera
        self.device = None

    def init(self):
        if self.module is None:
            raise RuntimeError("This firmware does not expose a camera module.")

        init_fn = getattr(self.module, "init", None)
        if callable(init_fn):
            try:
                result = init_fn(0, format=getattr(self.module, "JPEG", 0))
            except TypeError:
                try:
                    result = init_fn()
                except TypeError:
                    result = None
            if result is not None:
                self.device = result
            return

        camera_cls = getattr(self.module, "Camera", None)
        if camera_cls is not None:
            self.device = camera_cls()
            if hasattr(self.device, "init"):
                try:
                    self.device.init()
                except TypeError:
                    pass
            return

        raise RuntimeError("Camera module found, but no known init method was available.")

    def capture_jpeg(self):
        candidates = [self.device, self.module]
        for candidate in candidates:
            if candidate is None:
                continue
            for method_name in ("capture", "snapshot", "take", "grab"):
                method = getattr(candidate, method_name, None)
                if callable(method):
                    frame = method()
                    if isinstance(frame, tuple) and len(frame) == 2:
                        frame = frame[1]
                    if isinstance(frame, memoryview):
                        frame = frame.tobytes()
                    elif isinstance(frame, bytearray):
                        frame = bytes(frame)
                    return frame
        raise RuntimeError("No known camera capture method was found.")


def parse_query(query_string):
    result = {}
    if not query_string:
        return result

    for pair in query_string.split("&"):
        if not pair:
            continue
        key, _, value = pair.partition("=")
        if key:
            result[key] = value
    return result


MOTORS = None
CAMERA = CameraShim()


def apply_motion_from_query(query):
    global last_command_ms

    params = parse_query(query)
    left = float(params.get("left", 0.0) or 0.0)
    right = float(params.get("right", 0.0) or 0.0)
    present = int(params.get("present", "1") or "1")

    last_command_ms = time.ticks_ms()
    if present:
        MOTORS.drive(left, right)
    else:
        MOTORS.stop_all()

    return left, right, present


async def send_text_response(writer, body, status="200 OK", content_type="text/plain"):
    body_bytes = body.encode("utf-8")
    header = (
        "HTTP/1.1 {status}\r\n"
        "Content-Type: {content_type}\r\n"
        "Content-Length: {length}\r\n"
        "Connection: close\r\n\r\n"
    ).format(status=status, content_type=content_type, length=len(body_bytes))
    writer.write(header.encode("utf-8") + body_bytes)
    await writer.drain()


async def send_html_response(writer, body):
    await send_text_response(writer, body, content_type="text/html")


async def send_mjpeg_stream(writer):
    header = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n"
    )
    writer.write(header.encode("utf-8"))
    await writer.drain()

    try:
        while True:
            frame = CAMERA.capture_jpeg()
            if frame is None:
                await asyncio.sleep_ms(50)
                continue

            if not isinstance(frame, (bytes, bytearray)):
                frame = bytes(frame)

            part_header = (
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: {length}\r\n\r\n"
            ).format(length=len(frame))
            writer.write(part_header.encode("utf-8"))
            writer.write(frame)
            writer.write(b"\r\n")
            await writer.drain()
            await asyncio.sleep_ms(STREAM_DELAY_MS)
    except OSError:
        pass


async def handle_client(reader, writer):
    try:
        request_line = await reader.readline()
        if not request_line:
            return

        parts = request_line.decode().strip().split()
        if len(parts) < 2:
            await send_text_response(writer, "Bad request", status="400 Bad Request")
            return

        method = parts[0]
        target = parts[1]
        path, _, query = target.partition("?")

        while True:
            header_line = await reader.readline()
            if header_line in (b"\r\n", b"\n", b""):
                break

        if path == "/":
            body = """
<!doctype html>
<html>
  <head><meta charset='utf-8'><title>XIAO IPCam</title></head>
  <body style='background:#111;color:#eee;font-family:sans-serif;'>
    <h1>XIAO ESP32-S3 Sense IP Cam</h1>
    <p>Stream: <a href='/stream'>/stream</a></p>
    <p>Move: /move?left=0.2&amp;right=0.3</p>
    <img src='/stream' style='max-width:100%;height:auto;border:1px solid #444'>
  </body>
</html>
"""
            await send_html_response(writer, body)
            return

        if path == "/move":
            left, right, present = apply_motion_from_query(query)
            await send_text_response(writer, f"ok left={left:.3f} right={right:.3f} present={present}")
            return

        if path == "/stop":
            MOTORS.stop_all()
            await send_text_response(writer, "stopped")
            return

        if path == "/stream":
            await send_mjpeg_stream(writer)
            return

        await send_text_response(writer, "Not found", status="404 Not Found")
    except Exception as exc:
        try:
            await send_text_response(writer, f"Error: {exc}", status="500 Internal Server Error")
        except Exception:
            pass


async def watchdog_task():
    global last_command_ms
    while True:
        if time.ticks_diff(time.ticks_ms(), last_command_ms) > COMMAND_TIMEOUT_MS:
            MOTORS.stop_all()
        await asyncio.sleep_ms(100)


def start_access_point():
    ap = network.WLAN(network.AP_IF)
    ap.active(True)

    try:
        ap.config(essid=AP_SSID, password=AP_PASSWORD)
    except TypeError:
        ap.config(essid=AP_SSID)

    while not ap.active():
        time.sleep_ms(100)

    print("Access point active")
    print(ap.ifconfig())
    return ap


async def main():
    global MOTORS

    start_access_point()
    CAMERA.init()
    print("Camera initialized")
    print(f"Open http://192.168.4.1:{HTTP_PORT}/ in your browser or use /stream")

    i2c = I2C(I2C_ID, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=I2C_FREQ)
    pca = PCA9685(i2c)
    MOTORS = MotorBank(pca)
    MOTORS.stop_all()

    server = await asyncio.start_server(handle_client, "0.0.0.0", HTTP_PORT)
    asyncio.create_task(watchdog_task())

    try:
        while True:
            await asyncio.sleep(1)
    finally:
        server.close()
        await server.wait_closed()
        MOTORS.stop_all()


try:
    asyncio.run(main())
finally:
    if MOTORS is not None:
        MOTORS.stop_all()
