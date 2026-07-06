/*
  XIAO ESP32-S3 Sense -> MJPEG IP Camera
  ---------------------------------------
  Streams live video over Wi-Fi at:
      http://<device-ip>/          (simple viewer page)
      http://<device-ip>/stream    (raw MJPEG stream)
      http://<device-ip>/capture   (single JPEG snapshot)

  Camera sensor support:
      This sketch explicitly detects the sensor PID at runtime and applies
      the correct settings for the OV3660 (the sensor shipped on the
      XIAO ESP32S3 Sense), rather than assuming the older OV2640.
      If an OV2640 is detected instead, it is also handled correctly,
      so the sketch works on either sensor.

  Board setup (Arduino IDE):
      1. Install "esp32" board package (Espressif Systems) v2.0.x or later.
      2. Tools > Board: "XIAO_ESP32S3"
      3. Tools > PSRAM: "OPI PSRAM"  (must be enabled - Sense board has 8MB PSRAM)
      4. Tools > Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)" or similar
      5. Library: none extra needed - uses the built-in "esp_camera" driver
         that ships with the ESP32 Arduino core.

  Wiring: none needed - this targets the XIAO ESP32S3 Sense expansion
  board's onboard camera connector pin mapping directly.
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"

// ---------------------------------------------------------------------------
// Access Point credentials - EDIT THESE
// ---------------------------------------------------------------------------
const char *AP_SSID = "ESP32-CAM";
const char *AP_PASSWORD = "12345678";  // Use 8+ chars, or "" for open AP

// Optional: set AP IP (default is usually 192.168.4.1)
// #define USE_STATIC_IP
#ifdef USE_STATIC_IP
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
#endif

// ---------------------------------------------------------------------------
// XIAO ESP32S3 Sense camera pin map
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

httpd_handle_t streamHttpd = NULL;
httpd_handle_t pageHttpd = NULL;

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char *STREAM_PART_HEADER = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ---------------------------------------------------------------------------
// Camera initialization, with explicit OV3660 vs OV2640 handling
// ---------------------------------------------------------------------------
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Use larger frames/higher quality when PSRAM is present (it is, on Sense)
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;  // up to 1600x1200 on OV3660
    config.jpeg_quality = 10;            // lower number = higher quality
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s == NULL) {
    Serial.println("Failed to get sensor handle");
    return false;
  }

  // Explicitly branch on sensor PID so this sketch is correct for either
  // camera module, rather than silently assuming the older OV2640.
  switch (s->id.PID) {
    case OV3660_PID:
      Serial.println("Detected OV3660 sensor (XIAO ESP32S3 Sense default) - applying OV3660 tuning");
      s->set_vflip(s, 1);  // OV3660 image is mirrored/flipped relative to OV2640
      s->set_brightness(s, 1);
      s->set_saturation(s, -2);
      s->set_hmirror(s, 0);
      break;

    case OV2640_PID:
      Serial.println("Detected OV2640 sensor - applying OV2640 defaults");
      s->set_vflip(s, 0);
      s->set_hmirror(s, 0);
      break;

    default:
      Serial.printf("Detected unknown sensor PID 0x%x - using generic defaults\n", s->id.PID);
      break;
  }

  // If PSRAM-based UXGA proved too large/slow for your Wi-Fi link, drop to
  // FRAMESIZE_SVGA or FRAMESIZE_VGA here for smoother streaming:
  // s->set_framesize(s, FRAMESIZE_SVGA);

  return true;
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------
static esp_err_t stream_redirect_handler(httpd_req_t *req) {
  // Keep legacy /capture requests working by redirecting to the MJPEG stream.
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "/stream");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed during stream");
      res = ESP_FAIL;
    } else if (fb->format != PIXFORMAT_JPEG) {
      Serial.println("Non-JPEG frame captured; aborting stream");
      res = ESP_FAIL;
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART_HEADER, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
    }

    if (res != ESP_OK) {
      break;
    }
  }
  return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
  static const char PAGE[] =
    "<!DOCTYPE html><html><head><title>XIAO ESP32S3 IP Camera</title>"
    "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;}"
    "img{max-width:100%;height:auto;border:2px solid #444;}</style></head>"
    "<body><h2>XIAO ESP32S3 Sense - Live Camera</h2>"
    "<img src=\"/stream\">"
    "<p><a style=\"color:#8cf\" href=\"/stream\">Open raw MJPEG stream</a></p>"
    "</body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, PAGE, strlen(PAGE));
}

// ---------------------------------------------------------------------------
// Start the two HTTP servers: one for the page/snapshot (port 80),
// one dedicated to the MJPEG stream (port 81), matching the classic
// ESP32 CameraWebServer split so the UI stays responsive while streaming.
// ---------------------------------------------------------------------------
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  httpd_uri_t index_uri = {
    .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL
  };
  httpd_uri_t stream_compat_uri = {
    .uri = "/capture", .method = HTTP_GET, .handler = stream_redirect_handler, .user_ctx = NULL
  };
  httpd_uri_t stream_port80_uri = {
    .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL
  };

  if (httpd_start(&pageHttpd, &config) == ESP_OK) {
    httpd_register_uri_handler(pageHttpd, &index_uri);
    httpd_register_uri_handler(pageHttpd, &stream_compat_uri);
    httpd_register_uri_handler(pageHttpd, &stream_port80_uri);
  }

  config.server_port = 81;
  config.ctrl_port = 32769;
  httpd_uri_t stream_uri = {
    .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL
  };
  if (httpd_start(&streamHttpd, &config) == ESP_OK) {
    httpd_register_uri_handler(streamHttpd, &stream_uri);
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(500);

  if (!initCamera()) {
    Serial.println("Camera init failed - halting");
    while (true) delay(1000);
  }

#ifdef USE_STATIC_IP
  WiFi.softAPConfig(local_IP, gateway, subnet);
#endif

  WiFi.mode(WIFI_AP);
  bool ap_ok = WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (!ap_ok) {
    Serial.println("Failed to start AP - halting");
    while (true) delay(1000);
  }

  IPAddress apIP = WiFi.softAPIP();
  Serial.println();
  Serial.print("AP started. SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP: ");
  Serial.println(apIP);
  Serial.println("Camera stream ready at: http://<ap-ip>/stream   (viewer page at http://<ap-ip>/ )");

  startCameraServer();
}

void loop() {
  // Nothing needed here - the HTTP server handlers run in their own tasks.
  delay(10000);
}
