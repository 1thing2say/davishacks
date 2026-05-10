/*
 * Unified WiFi + USB Serial controller for the Smart Robot Car
 * on the Arduino UNO R4 WiFi.
 *
 * WiFi is used as a STATION to connect to the ESP32-CAM's AP for
 * image capture (HTTP GET /capture). Motor commands arrive over USB
 * serial from a Raspberry Pi.
 *
 * USB Serial protocol:
 *   Pi -> Arduino:  "CMD:<DIR> <SPEED>\n"   motor command
 *   Pi -> Arduino:  "S"                     trigger image capture
 *   Arduino -> Pi:  text debug lines        (terminated by \n)
 *   Arduino -> Pi:  0xFF 0xAA 0xBB 0xCC + 4-byte-BE-length + JPEG body
 *   Arduino -> Pi:  "CMD_OK\n"              motor ack
 *
 * DIR: F B L R FL FR BL BR S   SPEED: 0..255 (default 200)
 */
#include "WiFiControl_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

#include <WiFiS3.h>

// ---- ESP32-CAM WiFi settings ----
static const char CAM_SSID[] = "ELEGOO-2CAF18A0ED30";
static const char CAM_SERVER[] = "192.168.4.1";
static const int  CAM_PORT = 80;

// ---- Motor watchdog ----
#define WATCHDOG_MS     300UL

static bool wifi_ready = false;
static unsigned long last_cmd_ms = 0; // 0 = watchdog disarmed

// ---- USB Serial command buffer ----
static char serial_buf[64];
static uint8_t serial_idx = 0;

// ---- WiFi client for camera ----
static WiFiClient cam_client;

/* ================================================================
 * Direction parser (shared between serial and legacy UDP)
 * d=F/B/L/R/FL/FR/BL/BR/S  ->  1..9
 * ================================================================ */
static uint8_t parse_direction(const char *p, uint8_t len)
{
  if (len == 1) {
    switch (p[0]) {
      case 'F': return 1;
      case 'B': return 2;
      case 'L': return 3;
      case 'R': return 4;
      case 'S': return 9;
    }
  } else if (len == 2) {
    if (p[0] == 'F' && p[1] == 'L') return 5;
    if (p[0] == 'B' && p[1] == 'L') return 6;
    if (p[0] == 'F' && p[1] == 'R') return 7;
    if (p[0] == 'B' && p[1] == 'R') return 8;
  }
  return 0;
}

/* Parse "<DIR>[ <SPEED>]" from a buffer. Returns dir code (0 = invalid). */
static uint8_t parse_motor_cmd(const char *buf, int n, uint16_t *out_speed)
{
  *out_speed = 200;

  int i = 0;
  while (i < n && (buf[i] == ' ' || buf[i] == '\t')) i++;

  char d[3] = {0, 0, 0};
  int dlen = 0;
  while (i < n && dlen < 2) {
    char c = buf[i];
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    if (c < 'A' || c > 'Z') break;
    d[dlen++] = c;
    i++;
  }

  while (i < n && (buf[i] == ' ' || buf[i] == ',' || buf[i] == ':' || buf[i] == '\t')) i++;

  if (i < n && buf[i] >= '0' && buf[i] <= '9') {
    int v = 0;
    while (i < n && buf[i] >= '0' && buf[i] <= '9') {
      v = v * 10 + (buf[i] - '0');
      i++;
      if (v > 9999) break;
    }
    if (v < 0)   v = 0;
    if (v > 255) v = 255;
    *out_speed = (uint16_t)v;
  }

  return parse_direction(d, (uint8_t)dlen);
}

/* Apply a motor command and arm/disarm the watchdog. */
static void apply_command(uint8_t dir, uint16_t speed)
{
  Application_FunctionSet.ApplicationFunctionSet_WiFiCommand(dir, (uint8_t)speed);
  if (dir == 9) {
    last_cmd_ms = 0; // explicit stop -> disarm watchdog
  } else {
    last_cmd_ms = millis();
    if (last_cmd_ms == 0) last_cmd_ms = 1;
  }
}

/* ================================================================
 * ESP32-CAM image capture over WiFi
 * Fetches JPEG from ESP32-CAM HTTP server and streams it over
 * USB Serial with the magic header protocol.
 * ================================================================ */
static void fetchAndStreamImage(void)
{
  if (!wifi_ready || WiFi.status() != WL_CONNECTED) {
    Serial.println("PHOTO_SKIP:wifi_disconnected");
    return;
  }

  Serial.println("PHOTO_DEBUG:connecting_to_camera");

  if (!cam_client.connect(CAM_SERVER, CAM_PORT)) {
    Serial.println("PHOTO_SKIP:connection_failed");
    return;
  }

  Serial.println("PHOTO_DEBUG:requesting_image");

  cam_client.println("GET /capture HTTP/1.0");
  cam_client.print("Host: ");
  cam_client.println(CAM_SERVER);
  cam_client.println("Connection: close");
  cam_client.println();

  unsigned long timeout = millis();
  while (cam_client.available() == 0) {
    if (millis() - timeout > 10000) {
      Serial.println("PHOTO_SKIP:timeout");
      cam_client.stop();
      return;
    }
  }

  long imageLength = 0;
  bool isBody = false;

  // Parse HTTP headers
  while (cam_client.connected() || cam_client.available()) {
    if (cam_client.available()) {
      String line = cam_client.readStringUntil('\n');
      line.trim();

      if (line.length() == 0) {
        isBody = true;
        break;
      }

      String lowerLine = line;
      lowerLine.toLowerCase();
      if (lowerLine.startsWith("content-length:")) {
        imageLength = line.substring(line.indexOf(':') + 1).toInt();
      }
    }
  }

  if (isBody && imageLength > 0) {
    Serial.print("PHOTO_DEBUG:image_size=");
    Serial.println(imageLength);

    // Flush any pending serial output before sending binary
    Serial.flush();
    delay(50);

    // Magic Header
    Serial.write(0xFF);
    Serial.write(0xAA);
    Serial.write(0xBB);
    Serial.write(0xCC);

    // 4-byte big-endian size
    Serial.write((imageLength >> 24) & 0xFF);
    Serial.write((imageLength >> 16) & 0xFF);
    Serial.write((imageLength >> 8) & 0xFF);
    Serial.write(imageLength & 0xFF);

    Serial.flush();

    uint8_t buffer[256];
    long bytesRemaining = imageLength;

    while ((cam_client.connected() || cam_client.available()) && bytesRemaining > 0) {
      if (cam_client.available()) {
        int bytesToRead = min(sizeof(buffer), (size_t)bytesRemaining);
        int bytesRead = cam_client.read(buffer, bytesToRead);

        if (bytesRead > 0) {
          Serial.write(buffer, bytesRead);
          bytesRemaining -= bytesRead;
          if (bytesRemaining % 2048 < 256) {
            Serial.flush();
          }
        }
      }
    }

    Serial.flush();
    delay(50);
    Serial.println("PHOTO_DONE");
  } else {
    Serial.println("PHOTO_SKIP:no_content_length_found");
  }

  cam_client.stop();
}

/* ================================================================
 * WiFi Init — connect to ESP32-CAM AP as a station
 * ================================================================ */
void WiFiControl_Init(void)
{
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("[WiFi] module not found"));
    return;
  }

  Serial.println(F("[WiFi] Connecting to ESP32-CAM..."));

  int status = WL_IDLE_STATUS;
  int attempts = 0;
  while (status != WL_CONNECTED) {
    attempts++;
    Serial.print(F("[WiFi] Attempt "));
    Serial.println(attempts);

    status = WiFi.begin(CAM_SSID);

    if (status != WL_CONNECTED) {
      if (attempts >= 10) {
        Serial.println(F("[WiFi] Could not connect to ESP32-CAM after 10 attempts"));
        Serial.println(F("[WiFi] Motor control will work but camera is unavailable"));
        return;
      }
      delay(1000);
    }
  }

  wifi_ready = true;
  Serial.print(F("[WiFi] Connected! IP: "));
  Serial.println(WiFi.localIP());
}

/* ================================================================
 * Main loop — USB serial command processing + motor watchdog
 * ================================================================ */
void WiFiControl_Loop(void)
{
  // ---- Motor watchdog: stop if no command within WATCHDOG_MS ----
  if (last_cmd_ms != 0 && (millis() - last_cmd_ms) > WATCHDOG_MS) {
    Application_FunctionSet.ApplicationFunctionSet_WiFiCommand(9, 0);
    last_cmd_ms = 0;
  }

  // ---- USB Serial command processing ----
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Handle bare 'S' for camera trigger (no newline required,
    // matching the existing capture_image.py protocol)
    if (serial_idx == 0 && (c == 'S' || c == 's')) {
      // Peek: if next char is not part of a CMD: line, treat as camera trigger.
      // We check if the buffer is empty (fresh command).
      // But we need to distinguish "S\n" (camera) from "S 200\n" (stop motor).
      // Solution: bare 'S' with no prefix = camera. "CMD:S" = motor stop.
      serial_buf[0] = c;
      serial_idx = 1;

      // Wait briefly to see if more chars follow
      delay(5);
      if (Serial.available() == 0) {
        // Bare 'S' with nothing following — camera trigger
        serial_idx = 0;
        fetchAndStreamImage();
        continue;
      }
      // More chars coming — fall through to normal line buffering
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (serial_idx > 0) {
        serial_buf[serial_idx] = '\0';

        // Check for CMD: prefix (motor command)
        if (serial_idx >= 5 && serial_buf[0] == 'C' && serial_buf[1] == 'M'
            && serial_buf[2] == 'D' && serial_buf[3] == ':') {
          uint16_t speed = 200;
          uint8_t dir = parse_motor_cmd(serial_buf + 4, serial_idx - 4, &speed);
          if (dir != 0) {
            apply_command(dir, speed);
            Serial.println("CMD_OK");
          } else {
            Serial.println("CMD_ERR:bad_direction");
          }
        }
        // Bare "S" that arrived with a newline
        else if (serial_idx == 1 && (serial_buf[0] == 'S' || serial_buf[0] == 's')) {
          fetchAndStreamImage();
        }
        else {
          Serial.print("CMD_ERR:unknown:");
          Serial.println(serial_buf);
        }

        serial_idx = 0;
      }
      continue;
    }

    if (serial_idx < sizeof(serial_buf) - 1) {
      serial_buf[serial_idx++] = c;
    }
  }
}
