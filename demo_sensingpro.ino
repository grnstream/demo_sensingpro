/*
  ThingsInNet Sensing Pro Board
  AP-mode Web Dashboard for onboard sensors

   Check the Dev Kit: https://thingsinnet.com/product/esp32-universal-iot-dev-kit-sensing-pro/  

  Sensors used:
  - DHT11/22  : GPIO4
  - BH1750    : I2C (GPIO21 SDA, GPIO22 SCL)
  - BMP180    : I2C (GPIO21 SDA, GPIO22 SCL)
  - MPU6050   : I2C (GPIO21 SDA, GPIO22 SCL)
  - DS1307 RTC: I2C (GPIO21 SDA, GPIO22 SCL)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#include "DHT.h"
#include <BH1750.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "RTClib.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <SPI.h>
#include <SD.h>

// array size 1024
const unsigned char logo[] PROGMEM  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x20, 0x0c, 0x00, 0x87, 0xbc, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x70, 0x1e, 0x00, 0x87, 0xfd, 0x80, 0xc0, 0x00, 0x00, 0x03, 0x00, 0x0c, 0x0c, 0x00, 0x40, 
  0x00, 0xdf, 0xd6, 0x00, 0x80, 0x41, 0x80, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0c, 0x08, 0x00, 0x60, 
  0x00, 0x70, 0x1e, 0x00, 0x80, 0x41, 0xb0, 0x01, 0x81, 0x81, 0x83, 0x17, 0x0e, 0x08, 0x38, 0xf0, 
  0x00, 0x00, 0x06, 0x00, 0x80, 0x41, 0xf8, 0xcf, 0xc7, 0x72, 0x63, 0x1f, 0x8b, 0x08, 0xcc, 0x60, 
  0x00, 0xc1, 0x02, 0x00, 0x80, 0x41, 0x8c, 0xcc, 0x66, 0x36, 0x03, 0x18, 0x89, 0xc8, 0x86, 0x60, 
  0x01, 0x8f, 0x03, 0x00, 0x80, 0x41, 0x8c, 0xcc, 0x6c, 0x33, 0x83, 0x18, 0x88, 0xe9, 0x86, 0x60, 
  0x01, 0x1b, 0x01, 0x80, 0x80, 0x41, 0x8c, 0xcc, 0x6c, 0x31, 0xc3, 0x18, 0x88, 0x69, 0xfe, 0x60, 
  0x03, 0x17, 0x01, 0x80, 0x80, 0x41, 0x8c, 0xcc, 0x64, 0x30, 0x63, 0x18, 0x88, 0x38, 0x80, 0x60, 
  0x0e, 0x3d, 0x00, 0x60, 0x80, 0x41, 0x8c, 0xcc, 0x66, 0x36, 0x23, 0x18, 0x88, 0x18, 0xc4, 0x60, 
  0x1e, 0x3b, 0x80, 0xf0, 0x80, 0x41, 0x8c, 0xcc, 0x63, 0xf3, 0xe3, 0x18, 0x88, 0x08, 0x7c, 0x38, 
  0x1a, 0x3a, 0xfc, 0xd0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x0e, 0x03, 0xb8, 0xf0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x78, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0x01, 0xd9, 0x80, 0x80, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x80, 0xf1, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x81, 0xc3, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xc0, 0x02, 0x00, 0x80, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 
  0x00, 0x60, 0x0c, 0x00, 0x81, 0xb6, 0x60, 0x92, 0x0a, 0xcd, 0x89, 0x61, 0xde, 0x66, 0x02, 0x40, 
  0x00, 0x70, 0x1e, 0x00, 0x81, 0x5d, 0xde, 0xd7, 0x19, 0xbf, 0xfe, 0x61, 0x9f, 0xdb, 0xff, 0xc0, 
  0x00, 0xd7, 0xf6, 0x00, 0x81, 0xdd, 0xe5, 0xd6, 0x0f, 0xd3, 0x7b, 0x21, 0x95, 0xfe, 0xf6, 0x80, 
  0x00, 0x70, 0x1c, 0x00, 0x80, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 
  0x00, 0x20, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


unsigned char fBuffer[1024] = {0};
  

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------------------------
// AP CONFIG
// -------------------------
const char *ap_ssid     = "TIN-Sensing";
const char *ap_password = "12345678";  // change if needed

// Input Push Buttons
const int pushBtn[4] = {34, 35, 39, 36};
bool btnState[4] = { false, false, false, false };  // Active High
const bool BTN_ACTIVE_HIGH = true;

// Buzzer & On board LED
const int Buzzer = 26;
const int LED2 = 2;

// microSD
File myFile;
const int CS = 5;
bool sdReady,rwReady = false;
float sdSize;
String rwStatusStr = "No Card";


unsigned long lastPingTime = 0;
IPAddress lastClientIP;
bool clientConnected = false;

WebServer server(80);

// -------------------------
// DHT (Temp & Humidity)
// -------------------------
#define DHTPIN 4

#define DHTTYPE DHT11   // or DHT22

DHT dht(DHTPIN, DHTTYPE);
float lastTempDHT = 0.0f;
float lastHumDHT  = 0.0f;
bool dhtReady = false;

// -------------------------
// BH1750 (Light)
// -------------------------
BH1750 lightMeter;
bool bhReady = false;
float lastLux = 0.0f;

// -------------------------
// BMP180 (Pressure & Temp)
// -------------------------
Adafruit_BMP085 bmp;
bool bmpReady = false;
float lastBmpTemp   = 0.0f;
float lastBmpPress  = 0.0f; // hPa

// -------------------------
// MPU6050 (Acceleration & Gyro)
// -------------------------
Adafruit_MPU6050 mpu;
bool mpuReady = false;
float lastAx = 0.0f, lastAy = 0.0f, lastAz = 0.0f;
float lastGx = 0.0f, lastGy = 0.0f, lastGz = 0.0f; // deg/s

// -------------------------
// DS1307 RTC
// -------------------------
RTC_DS1307 rtc;
bool rtcReady = false;
String lastRtcTime = "N/A";

// -------------------------
// HTML PAGE (SINGLE PAGE APP)
// -------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>ThingsInNet Sensing Pro Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">

  <style>
    :root {
      --bg: #050816;
      --bg-card: #0f172a;
      --accent: #22c55e;
      --accent-soft: rgba(34, 197, 94, 0.15);
      --danger: #ef4444;
      --text-main: #e5e7eb;
      --text-muted: #9ca3af;
      --border-subtle: rgba(148, 163, 184, 0.35);
      --shadow-soft: 0 18px 45px rgba(15, 23, 42, 0.85);
      --radius-card: 18px;
      --radius-pill: 999px;
    }

    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "SF Pro Text",
                   "Segoe UI", sans-serif;
    }

    body {
      background: radial-gradient(circle at top, #1e293b 0, #020617 45%, #000 100%);
      color: var(--text-main);
      min-height: 100vh;
      display: flex;
      align-items: stretch;
      justify-content: center;
    }

    /* -------- LOADER ---------- */
    #loader {
      position: fixed;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      background: radial-gradient(circle at top, #111827 0, #020617 55%, #000 100%);
      z-index: 999;
      transition: opacity 0.45s ease, visibility 0.45s ease;
    }

    #loader.hidden {
      opacity: 0;
      visibility: hidden;
    }

    .loader-card {
      background: rgba(15, 23, 42, 0.96);
      border-radius: 24px;
      padding: 32px 38px;
      border: 1px solid rgba(148, 163, 184, 0.45);
      box-shadow: var(--shadow-soft);
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 18px;
      backdrop-filter: blur(16px);
    }

    .loader-title {
      font-size: 26px;
      font-weight: 700;
      letter-spacing: 0.15em;
      text-transform: uppercase;
    }

    .loader-subtitle {
      font-size: 13px;
      color: var(--text-muted);
    }

    .spinner {
      margin-top: 10px;
      width: 40px;
      height: 40px;
      border-radius: 50%;
      border: 3px solid rgba(148, 163, 184, 0.4);
      border-top-color: var(--accent);
      animation: spin 1.1s linear infinite;
    }

    @keyframes spin {
      to { transform: rotate(360deg); }
    }

    /* ------- APP SHELL --------- */
    #app {
      width: 100%;
      max-width: 1100px;
      margin: 24px;
      display: flex;
      flex-direction: column;
      gap: 18px;
      opacity: 0;
      transform: translateY(12px);
      transition: opacity 0.5s ease, transform 0.5s ease;
    }

    #app.visible {
      opacity: 1;
      transform: translateY(0);
    }

    .app-header {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
      padding: 14px 18px;
      border-radius: 18px;
      background: linear-gradient(135deg, rgba(15, 23, 42, 0.96), rgba(15, 23, 42, 0.85));
      border: 1px solid rgba(148, 163, 184, 0.45);
      box-shadow: 0 12px 30px rgba(15, 23, 42, 0.9);
    }

    .brand {
      display: flex;
      align-items: center;
      gap: 10px;
    }

    .brand-mark {
      width: 32px;
      height: 32px;
      border-radius: 30%;
      background: radial-gradient(circle at 30% 0, #4ade80 0, #16a34a 40%, #064e3b 100%);
      display: flex;
      align-items: center;
      justify-content: center;
      font-weight: 800;
      font-size: 16px;
    }

    .brand-text {
      display: flex;
      flex-direction: column;
      gap: 3px;
    }

    .brand-title {
      font-weight: 600;
      font-size: 16px;
      letter-spacing: 0.12em;
      text-transform: uppercase;
    }

    .brand-subtitle {
      font-size: 12px;
      color: var(--text-muted);
    }

    .status-pill {
      padding: 6px 12px;
      border-radius: var(--radius-pill);
      font-size: 12px;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      background: rgba(22, 163, 74, 0.12);
      border: 1px solid rgba(34, 197, 94, 0.7);
      color: #bbf7d0;
    }

    .dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #22c55e;
      box-shadow: 0 0 12px rgba(34, 197, 94, 0.9);
    }

    .ip-info {
      font-size: 11px;
      color: var(--text-muted);
    }

    /* -------- DASHBOARD GRID ------- */
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 14px;
    }

    .card {
      background: rgba(15, 23, 42, 0.9);
      border-radius: var(--radius-card);
      padding: 14px 16px 16px;
      border: 1px solid var(--border-subtle);
      box-shadow: 0 12px 28px rgba(15, 23, 42, 0.85);
      display: flex;
      flex-direction: column;
      gap: 6px;
      position: relative;
      overflow: hidden;
    }

    .card::before {
      content: "";
      position: absolute;
      inset: -30%;
      opacity: 0.08;
      background: radial-gradient(circle at top left, #22c55e 0, transparent 55%);
      pointer-events: none;
    }

    .card-header {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      z-index: 1;
    }

    .card-title {
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.16em;
      color: var(--text-muted);
    }

    .sensor-tag {
      font-size: 11px;
      padding: 4px 10px;
      border-radius: var(--radius-pill);
      background: rgba(15, 23, 42, 0.95);
      border: 1px solid rgba(148, 163, 184, 0.5);
      color: var(--text-muted);
    }

    .reading {
      font-size: 28px;
      font-weight: 600;
      display: flex;
      align-items: baseline;
      gap: 6px;
      z-index: 1;
      flex-wrap: wrap;
    }

    .reading span.unit {
      font-size: 13px;
      color: var(--text-muted);
    }

    .trend {
      font-size: 11px;
      color: var(--text-muted);
      margin-top: 8px;
      z-index: 1;
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 4px;
    }

    .badge-soft {
      padding: 3px 8px;
      border-radius: var(--radius-pill);
      font-size: 10px;
      border: 1px solid rgba(148, 163, 184, 0.6);
      background: rgba(15, 23, 42, 0.95);
    }

    .footer-bar {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 11px;
      color: var(--text-muted);
      padding: 4px 2px 0;
      margin-top: 4px;
      border-top: 1px solid rgba(30, 64, 175, 0.7);
    }

    .time {
      font-feature-settings: "tnum" 1;
    }

    @media (max-width: 600px) {
      #app {
        margin: 14px;
      }
      .app-header {
        padding: 12px 14px;
      }
    }
  </style>
</head>
<body>
  <!-- LOADING SCREEN -->
  <div id="loader">
    <div class="loader-card">
      <div class="loader-title">THINGSINNET</div>
      <div class="loader-subtitle">
        Sensing Pro • Initializing sensors...
      </div>
      <div class="spinner"></div>
    </div>
  </div>

  <!-- APP -->
  <div id="app">
    <header class="app-header">
      <div class="brand">
        <div class="brand-mark">T</div>
        <div class="brand-text">
          <div class="brand-title">ThingsInNet</div>
          <div class="brand-subtitle">ESP32 Universal IoT Dev Kit – Sensing Pro</div>
        </div>
      </div>

      <div style="display:flex; flex-direction:column; align-items:flex-end; gap:4px;">
        <div class="status-pill">
          <span class="dot"></span>
          <span>Board Online</span>
        </div>
        <div class="ip-info">Access Point • 192.168.4.1</div>
      </div>
    </header>

    <main class="grid">
      <!-- DHT Temperature -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">Ambient Temp (DHT)</div>
          <div class="sensor-tag">DHT11/22 • °C</div>
        </div>
        <div class="reading">
          <span id="dhtTempValue">--</span>
          <span class="unit">°C</span>
        </div>
        <div class="trend">
          <span>Air temperature near board</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="dhtTempTime">--:--:--</span>
        </div>
      </article>

      <!-- DHT Humidity -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">Humidity (DHT)</div>
          <div class="sensor-tag">DHT11/22 • %RH</div>
        </div>
        <div class="reading">
          <span id="humidityValue">--</span>
          <span class="unit">%RH</span>
        </div>
        <div class="trend">
          <span>Relative humidity</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="humidityTime">--:--:--</span>
        </div>
      </article>

      <!-- BH1750 Light -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">Light Level</div>
          <div class="sensor-tag">BH1750 • lux</div>
        </div>
        <div class="reading">
          <span id="lightValue">--</span>
          <span class="unit">lux</span>
        </div>
        <div class="trend">
          <span>Illuminance at sensor</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="lightTime">--:--:--</span>
        </div>
      </article>

      <!-- BMP180 Pressure -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">Pressure</div>
          <div class="sensor-tag">BMP180 • hPa</div>
        </div>
        <div class="reading">
          <span id="pressureValue">--</span>
          <span class="unit">hPa</span>
        </div>
        <div class="trend">
          <span>Barometric pressure</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="pressureTime">--:--:--</span>
        </div>
      </article>

      <!-- BMP180 Temperature -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">Board Temp (BMP180)</div>
          <div class="sensor-tag">BMP180 • °C</div>
        </div>
        <div class="reading">
          <span id="bmpTempValue">--</span>
          <span class="unit">°C</span>
        </div>
        <div class="trend">
          <span>Temperature at pressure sensor</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="bmpTempTime">--:--:--</span>
        </div>
      </article>

      <!-- IMU Acceleration -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">Acceleration</div>
          <div class="sensor-tag">MPU6050 • m/s²</div>
        </div>
        <div class="reading">
          <span id="accelMag">--</span>
          <span class="unit">m/s²</span>
        </div>
        <div class="trend">
          <span id="accelXYZ">X: --, Y: --, Z: --</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="imuAccelTime">--:--:--</span>
        </div>
      </article>

      <!-- IMU Rotation -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">Rotation</div>
          <div class="sensor-tag">MPU6050 • °/s</div>
        </div>
        <div class="reading">
          <span id="gyroMag">--</span>
          <span class="unit">°/s</span>
        </div>
        <div class="trend">
          <span id="gyroXYZ">X: --, Y: --, Z: --</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="imuGyroTime">--:--:--</span>
        </div>
      </article>

      <!-- RTC Time -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">Board Time</div>
          <div class="sensor-tag">DS1307 RTC</div>
        </div>
        <div class="reading">
          <span id="rtcTimeValue">--</span>
        </div>
        <div class="trend">
          <span>Real-time clock on board</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="rtcTimeUpdated">--:--:--</span>
        </div>
      </article>

      <!-- microSD Card -->
      <article class="card">
        <div class="card-header">
          <div class="card-title">SD Status</div>
          <div class="sensor-tag">SDXC/SDHC</div>
        </div>
        <div class="reading">
          <span id="cardSize">--</span>
          <span class="unit">GB</span>
        </div>
        <div class="trend"> 
          <span id="rwStatus">Read/Write: --</span>
          <span class="badge-soft">Live</span>
        </div>
        <div class="footer-bar">
          <span>Updated</span>
          <span class="time" id="sdUpdateTime">--:--:--</span>
        </div>
      </article>

    </main>
  </div>

  <script>
    let appVisible = false;

    function showApp() {
      if (!appVisible) {
        document.getElementById('loader').classList.add('hidden');
        document.getElementById('app').classList.add('visible');
        appVisible = true;
      }
    }

    function formatTime() {
      const d = new Date();
      return d.toTimeString().split(" ")[0]; // HH:MM:SS
    }

    function updateCardNumeric(idBase, value, decimals) {
      const valueEl = document.getElementById(idBase + "Value");
      const timeEl  = document.getElementById(idBase + "Time");
      if (valueEl && timeEl) {
        valueEl.textContent = value.toFixed(decimals);
        timeEl.textContent  = formatTime();
      }
    }

    function updateSdCard(floatValue, stringValue) {
      if (typeof floatValue === "number" & floatValue != 0) {
        document.getElementById("cardSize").textContent = floatValue.toFixed(1);
      } else {
        document.getElementById("cardSize").textContent = "--";
      }

      document.getElementById("rwStatus").textContent =
        `Read/Write: ${(typeof stringValue === "string") ? stringValue : "N/A"}`;

      document.getElementById("sdUpdateTime").textContent = formatTime();
    }


    function sendPing() {
      fetch('/alive');
    }
    // Send a ping every 2 seconds
    setInterval(sendPing, 2000);
    sendPing();

    async function fetchReadings() {
      try {
        const response = await fetch('/readings');
        if (!response.ok) throw new Error('Network error');
        const data = await response.json();

        // Simple numeric cards
        updateCardNumeric("dhtTemp",  data.temp_dht,   1);
        updateCardNumeric("humidity", data.humidity,   1);
        updateCardNumeric("light",    data.light_lux,  1);
        updateCardNumeric("pressure", data.pressure_hpa, 1);
        updateCardNumeric("bmpTemp",  data.temp_bmp,   1);

        // SD card
        updateSdCard(data.sd_size, data.sd_rw);

        // Acceleration
        const accelMag = Math.sqrt(
          data.accel_x * data.accel_x +
          data.accel_y * data.accel_y +
          data.accel_z * data.accel_z
        );
        document.getElementById('accelMag').textContent = accelMag.toFixed(2);
        document.getElementById('accelXYZ').textContent =
          `X: ${data.accel_x.toFixed(2)}, Y: ${data.accel_y.toFixed(2)}, Z: ${data.accel_z.toFixed(2)}`;
        document.getElementById('imuAccelTime').textContent = formatTime();

        // Gyro
        const gyroMag = Math.sqrt(
          data.gyro_x * data.gyro_x +
          data.gyro_y * data.gyro_y +
          data.gyro_z * data.gyro_z
        );
        document.getElementById('gyroMag').textContent = gyroMag.toFixed(2);
        document.getElementById('gyroXYZ').textContent =
          `X: ${data.gyro_x.toFixed(2)}, Y: ${data.gyro_y.toFixed(2)}, Z: ${data.gyro_z.toFixed(2)}`;
        document.getElementById('imuGyroTime').textContent = formatTime();

        // RTC
        document.getElementById('rtcTimeValue').textContent = data.rtc_time;
        document.getElementById('rtcTimeUpdated').textContent = formatTime();

        showApp();
      } catch (err) {
        console.error('Error fetching readings:', err);
      }
    }

    window.addEventListener('load', () => {
      fetchReadings();
      setInterval(fetchReadings, 2000); // update every 2s
    });
  </script>
</body>
</html>
)rawliteral";

// -------------------------
// HELPER FUNCTIONS
// -------------------------

// ====== Push Button Handler ======
void handlePushBtn() {
  bool buzzerState = false;
  for (int i = 0; i < 4; i++) {
    bool state = !!digitalRead(pushBtn[i]);
    
    if(BTN_ACTIVE_HIGH) btnState[i] = state ? HIGH : LOW;
    else btnState[i] = state ? LOW : HIGH;

    buzzerState |= btnState[i];
  }

  digitalWrite(Buzzer, buzzerState);  // To activate Buzzer
  digitalWrite(LED2, buzzerState);
}

float readDhtTemperature() {
  float t = dht.readTemperature();
  if (!isnan(t)) {
    lastTempDHT = t;
    dhtReady = true;
  } else {
    dhtReady = false;
  }
  return lastTempDHT;
}

float readDhtHumidity() {
  float h = dht.readHumidity();
  if (!isnan(h)) {
    lastHumDHT = h;
    dhtReady = true;
  } else {
    dhtReady = false;
  }
  return lastHumDHT;
}

float readLightLux() {
  if (!bhReady) return lastLux;
  float lux = lightMeter.readLightLevel(); // returns lux
  if (lux >= 0.0f && lux < 120000.0f) {
    lastLux = lux;
  }
  return lastLux;
}

float readBmpTemperature() {
  if (!bmpReady) return lastBmpTemp;
  float t = bmp.readTemperature();
  lastBmpTemp = t;
  return lastBmpTemp;
}

float readBmpPressure() {
  if (!bmpReady) return lastBmpPress;
  float p = bmp.readPressure();   // Pa
  lastBmpPress = p / 100.0f;      // convert to hPa
  return lastBmpPress;
}

void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (!mpuReady) {
    ax = lastAx; ay = lastAy; az = lastAz;
    gx = lastGx; gy = lastGy; gz = lastGz;
    return;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Acceleration in m/s^2
  lastAx = a.acceleration.x;
  lastAy = a.acceleration.y;
  lastAz = a.acceleration.z;

  // Gyro is in rad/s, convert to deg/s
  const float RAD_to_DEG = 57.2957795f;
  lastGx = g.gyro.x * RAD_to_DEG;
  lastGy = g.gyro.y * RAD_to_DEG;
  lastGz = g.gyro.z * RAD_to_DEG;

  ax = lastAx; ay = lastAy; az = lastAz;
  gx = lastGx; gy = lastGy; gz = lastGz;
}

String getRtcTimeString() {
  if (!rtcReady) return lastRtcTime;
  DateTime now = rtc.now();
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  lastRtcTime = String(buf);
  return lastRtcTime;
}

// -------------------------
// HTTP HANDLERS
// -------------------------
void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleReadings() {
  float tempDht = readDhtTemperature();
  float hum     = readDhtHumidity();
  float lux     = readLightLux();
  float bmpT    = readBmpTemperature();
  float press   = readBmpPressure();

  float ax, ay, az, gx, gy, gz;
  readIMU(ax, ay, az, gx, gy, gz);

  String rtcStr = getRtcTimeString();

  String json = "{";
  json += "\"temp_dht\":"      + String(tempDht, 1)      + ",";
  json += "\"humidity\":"      + String(hum, 1)          + ",";
  json += "\"light_lux\":"     + String(lux, 1)          + ",";
  json += "\"temp_bmp\":"      + String(bmpT, 1)         + ",";
  json += "\"pressure_hpa\":"  + String(press, 1)        + ",";
  json += "\"sd_rw\":\""         + rwStatusStr             + "\",";
  json += "\"sd_size\":"       + String(sdSize, 1)       + ","; 
  json += "\"accel_x\":"       + String(ax, 2)           + ",";
  json += "\"accel_y\":"       + String(ay, 2)           + ",";
  json += "\"accel_z\":"       + String(az, 2)           + ",";
  json += "\"gyro_x\":"        + String(gx, 2)           + ",";
  json += "\"gyro_y\":"        + String(gy, 2)           + ",";
  json += "\"gyro_z\":"        + String(gz, 2)           + ",";
  json += "\"rtc_time\":\""    + rtcStr                  + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleNotFound() {
  String message = "Not found\n\n";
  message += "URI: " + server.uri() + "\n";
  server.send(404, "text/plain", message);
}

void handleAlive() {
  lastPingTime = millis();
  lastClientIP = server.client().remoteIP();

  if (!clientConnected) {
    clientConnected = true;
    Serial.printf("Client connected: %s\n", lastClientIP.toString().c_str());
  }

  server.send(200, "text/plain", "OK");
}

void displayLogo(bool k) {

  int i,j;

  display.clearDisplay();
  
    for (i=0; i<16; i++) {
      for (j=0; j<64; j++) {
        if(k==true) fBuffer[i+j*16] = logo[i+j*16];
        else fBuffer[i + j * 16] = 0;
      }
      display.clearDisplay();
      display.drawBitmap(0, 0, fBuffer, 128, 64, WHITE);
      display.display();
      delay(40);
    }
}

void displayInfo() {
  display.clearDisplay();  // Clear the display

  display.setTextColor(WHITE);  // Set text color to white
  display.setTextSize(1);       // Set text size to 1

  int16_t labelX = 0;   // Start from the left edge
  int16_t valueX = 35;  // Fixed position for values after the colon

  display.setCursor(labelX, 0);  
  display.print("Connect to network");      

  display.setCursor(labelX, 32);                    
  display.print("SSID");                        
  display.setCursor(valueX, 32);                     
  display.print(": ");
  display.print(ap_ssid); 

  display.setCursor(labelX, 44);  
  display.print("Pwd");    
  display.setCursor(valueX, 44);                     
  display.print(": ");
  display.print(ap_password); 
  display.setCursor(labelX, 56);  
  display.print("IP");   
  display.setCursor(valueX, 56);           
  display.print(": 192.168.4.1");           

  // Update the display
  display.display();
}

void displayStatus() {
  display.clearDisplay();  // Clear the display

  display.setTextColor(WHITE);  
  display.setTextSize(1);       

  // Fixed horizontal position for labels (text before the colon)
  int16_t labelX = 0;   // Start from the left edge
  int16_t valueX = 70;  // Fixed position for values after the colon

  String titleStr = "SENSOR STATUS";

  int16_t titleWidth = titleStr.length() * 6;                // 6 characters * 6 pixels per character (size 1)
  int16_t titlePosX = (display.width() - titleWidth) / 2;    // Center horizontally
  display.setCursor(titlePosX, 0);                           // Set cursor to calculated position
  display.print(titleStr);       

  display.setCursor(labelX, 12);                     
  display.print("DS1307");                        
  display.setCursor(valueX, 12);                     
  display.print(rtcReady ? ": OK" : ": Failed"); 

  display.setCursor(labelX, 22);                    
  display.print("DHT11/22");                        
  display.setCursor(valueX, 22);                    
  display.print(dhtReady ? ": OK" : ": Failed"); 

  display.setCursor(labelX, 32);                   
  display.print("BH1750");                       
  display.setCursor(valueX, 32);                    
  display.print(bhReady ? ": OK" : ": Failed"); 

  display.setCursor(labelX, 42);  
  display.print("MPU-6050");    
  display.setCursor(valueX, 42);                     
  display.print(mpuReady ? ": OK" : ": Failed"); 

  display.setCursor(labelX, 52); 
  display.print("BMP180");    
  display.setCursor(valueX, 52);            
  display.print(bmpReady ? ": OK" : ": Failed");   

  display.display();
}

void WriteFile(const char * path, const char * message){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(path, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.printf("Writing to %s ", path);
    myFile.println(message);
    myFile.close(); // close the file:
    Serial.println("completed.");
  }else{
    Serial.println("error opening file ");
    Serial.println(path);
  }
  
}

void ReadFile(const char * path){
  myFile = SD.open(path);
  if (myFile) {
     Serial.printf("Reading file from %s\n", path);
     // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close(); // close the file:
    rwReady = true;
  } 

  else {
    // if the file doesn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

// -------------------------
// SETUP & LOOP
// -------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Starting ThingsInNet Sensing Pro AP...");

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;
  }
  display.clearDisplay();
  display.display();
  displayLogo(true);
  delay(1000);

  // Initailze Push Buttons.
  for (int i = 0; i < 4; i++) {
    pinMode(pushBtn[i], INPUT);
  }

  // Initialize Buzzer & On board LED
  pinMode(LED2, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  // initailize I2C + sensors
  Wire.begin();
  dht.begin();

  // BH1750
  bhReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  if (!bhReady) {
    Serial.println("BH1750 not found.");
  } else {
    Serial.println("BH1750 ready.");
  }

  // BMP180
  bmpReady = bmp.begin();
  if (!bmpReady) {
    Serial.println("BMP180 not found.");
  } else {
    Serial.println("BMP180 ready.");
  }

  // MPU6050
  if (mpu.begin(0x68)) {
    mpuReady = true;
    Serial.println("MPU6050 ready.");
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else if (mpu.begin(0x69)) {
    mpuReady = true;
    Serial.println("MPU6050 ready.");
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else {
    Serial.println("MPU6050 not found.");
    mpuReady = false;
  }

  // DS1307 RTC
  if (!rtc.begin()) {
    Serial.println("RTC (DS1307) not found.");
    rtcReady = false;
  } else {
    rtcReady = true;
    Serial.println("RTC ready.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // microSD
  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
  }else{
    sdReady = true;

    sdSize = 1.0*SD.cardSize()/1073741824; // convert to GB

    WriteFile("/test.txt", "thingsinnet.com");
    ReadFile("/test.txt");

    if(rwReady) {
      rwStatusStr = "Ok";
    } else {
      rwStatusStr = "Failed";
    }

  }

  // Access Point
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(ap_ssid, ap_password);
  if (apStarted) {
    Serial.println("Access Point started.");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Failed to start AP!");
  }

  // HTTP routes
  server.on("/", handleRoot);
  server.on("/readings", handleReadings);
  server.on("/alive", handleAlive);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started.");
  Serial.println(rwStatusStr);
  displayLogo(false);

  displayInfo();
  
}

void loop() {
  server.handleClient();

  // Detect disconnect after 5 seconds without a ping
  if (clientConnected && millis() - lastPingTime > 5000) {
    Serial.printf("Client disconnected: %s\n", lastClientIP.toString().c_str());
    clientConnected = false; 
  }

  if (clientConnected) displayStatus();
  else displayInfo();

  handlePushBtn();
}

