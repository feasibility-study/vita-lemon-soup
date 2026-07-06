"""
ESP32-S3 Sense MicroPython camera streamer.

This is a stripped-down version of the IP cam server with all motor code removed.
It creates a Wi-Fi access point and serves the camera as MJPEG at /stream.

Typical URLs:
- http://192.168.4.1:8080/
- http://192.168.4.1:8080/stream

Notes:
- The exact camera API on XIAO ESP32-S3 Sense depends on the MicroPython build.
- This script uses a small shim that tries the common init/capture patterns.
"""

import time
import uasyncio as asyncio
import network

try:
    import camera
except ImportError:
    camera = None


AP_SSID = "XIAO-IPCAM"
AP_PASSWORD = "12345678"
HTTP_PORT = 8080
STREAM_DELAY_MS = 70


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


CAMERA = CameraShim()


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
    except (OSError, RuntimeError, ValueError) as exc:
        try:
            await send_text_response(writer, f"Stream error: {exc}", status="500 Internal Server Error")
        except Exception:
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
    <h1>XIAO ESP32-S3 Sense Camera</h1>
    <p>Stream: <a href='/stream'>/stream</a></p>
    <img src='/stream' style='max-width:100%;height:auto;border:1px solid #444'>
  </body>
</html>
"""
            await send_html_response(writer, body)
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


async def main():
    start_access_point()
    CAMERA.init()
    print("Camera initialized")
    print(f"Open http://192.168.4.1:{HTTP_PORT}/ in your browser or use /stream")

    server = await asyncio.start_server(handle_client, "0.0.0.0", HTTP_PORT)

    try:
        while True:
            await asyncio.sleep(1)
    finally:
        server.close()
        await server.wait_closed()


asyncio.run(main())
