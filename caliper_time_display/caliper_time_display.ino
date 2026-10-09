#include <WiFi.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ======================= settings =======================
const char *WIFI_SSID = "VT PRO";
const char *WIFI_PASS = "Cool$h1t";

const char *TZ_INFO = "PST8PDT,M3.2.0,M11.1.0";   // Los Angeles, daylight saving automatic
const char *NTP_1   = "pool.ntp.org";
const char *NTP_2   = "time.nist.gov";

const uint8_t SCREEN_ROTATION = 3;   // 1 or 3: flips the picture 180 degrees

// backlight brightness: 0 = off, 255 = full. Lower = longer battery life.
const uint8_t BACKLIGHT = 80;

// how often Wi-Fi wakes up to correct the time
const unsigned long RESYNC_EVERY_MS = 60UL * 60UL * 1000UL;  // 1 hour
const unsigned long RETRY_MS        = 5UL * 60UL * 1000UL;   // if a correction fails
const unsigned long FIRST_RETRY_MS  = 30UL * 1000UL;         // if we never got the time yet
const unsigned long WIFI_TIMEOUT_MS = 20UL * 1000UL;         // give up connecting after this
const unsigned long SYNC_TIMEOUT_MS = 15UL * 1000UL;         // give up waiting for NTP after this

// ======================= look =======================
const uint16_t C_BG  = 0x0000;   // black
const uint16_t C_ON  = 0xA7E0;   // digit green (this screen has red/blue swapped)
const uint16_t C_OFF = 0x0080;   // unlit segments, very dim

const int16_t DIG_W     = 30;    // one digit's width
const int16_t DIG_H     = 76;    // one digit's height
const int16_t SEG_T     = 7;     // segment thickness (odd number looks best)
const int16_t SEG_GAP   = 1;     // small gap where segments meet
const int16_t PAIR_GAP  = 5;     // gap between the two digits of HH / MM / SS
const int16_t COLON_W   = 14;    // space for each colon

// ======================= pins (TTGO T-Display) =======================
const int8_t PIN_TFT_MOSI = 19;
const int8_t PIN_TFT_SCLK = 18;
const int8_t PIN_TFT_CS   = 5;
const int8_t PIN_TFT_DC   = 16;
const int8_t PIN_TFT_RST  = 23;
const int8_t PIN_TFT_BL   = 4;

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GFXcanvas16 canvas(240, 135);  // draw here, then push in one go: no flicker

String lastDrawn = "";

// ======================= 7-segment drawing =======================
//   segments:  a = top, b = top right, c = bottom right, d = bottom,
//              e = bottom left, f = top left, g = middle
const uint8_t SEGMENTS[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
const uint8_t SEG_DASH = 0x40;   // only the middle segment

void hSeg(int16_t x1, int16_t x2, int16_t cy, uint16_t col) {
  const int16_t h = SEG_T / 2;
  x1 += SEG_GAP;
  x2 -= SEG_GAP;
  canvas.fillRect(x1 + h, cy - h, (x2 - x1) - 2 * h + 1, SEG_T, col);
  canvas.fillTriangle(x1, cy, x1 + h, cy - h, x1 + h, cy + h, col);
  canvas.fillTriangle(x2, cy, x2 - h, cy - h, x2 - h, cy + h, col);
}

void vSeg(int16_t cx, int16_t y1, int16_t y2, uint16_t col) {
  const int16_t h = SEG_T / 2;
  y1 += SEG_GAP;
  y2 -= SEG_GAP;
  canvas.fillRect(cx - h, y1 + h, SEG_T, (y2 - y1) - 2 * h + 1, col);
  canvas.fillTriangle(cx, y1, cx - h, y1 + h, cx + h, y1 + h, col);
  canvas.fillTriangle(cx, y2, cx - h, y2 - h, cx + h, y2 - h, col);
}

void drawDigit(int16_t x, int16_t y, uint8_t mask) {
  const int16_t h = SEG_T / 2;
  const int16_t L = x + h, R = x + DIG_W - 1 - h;
  const int16_t T = y + h, M = y + DIG_H / 2, B = y + DIG_H - 1 - h;
  hSeg(L, R, T, (mask & 0x01) ? C_ON : C_OFF);  // a
  vSeg(R, T, M, (mask & 0x02) ? C_ON : C_OFF);  // b
  vSeg(R, M, B, (mask & 0x04) ? C_ON : C_OFF);  // c
  hSeg(L, R, B, (mask & 0x08) ? C_ON : C_OFF);  // d
  vSeg(L, M, B, (mask & 0x10) ? C_ON : C_OFF);  // e
  vSeg(L, T, M, (mask & 0x20) ? C_ON : C_OFF);  // f
  hSeg(L, R, M, (mask & 0x40) ? C_ON : C_OFF);  // g
}

void drawColon(int16_t x, int16_t y, bool on) {
  const int16_t cx = x + COLON_W / 2 - SEG_T / 2;
  uint16_t col = on ? C_ON : C_OFF;
  canvas.fillRect(cx, y + DIG_H / 3 - SEG_T / 2, SEG_T, SEG_T, col);
  canvas.fillRect(cx, y + DIG_H * 2 / 3 - SEG_T / 2, SEG_T, SEG_T, col);
}

// text is "HH:MM:SS" (digits or '-'; a space instead of ':' = colon off)
void drawTime(const String &text) {
  if (text == lastDrawn) return;   // nothing changed on screen
  lastDrawn = text;

  canvas.fillScreen(C_BG);
  const int16_t totalW = 6 * DIG_W + 3 * PAIR_GAP + 2 * COLON_W;
  int16_t x = (canvas.width() - totalW) / 2;
  const int16_t y = (canvas.height() - DIG_H) / 2;

  for (int i = 0; i < 8; i++) {
    char c = text[i];
    if (i == 2 || i == 5) {
      drawColon(x, y, c == ':');
      x += COLON_W;
      continue;
    }
    uint8_t mask = (c >= '0' && c <= '9') ? SEGMENTS[c - '0'] : SEG_DASH;
    drawDigit(x, y, mask);
    x += DIG_W;
    if (i == 0 || i == 3 || i == 6) x += PAIR_GAP;
  }

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());
}

// ======================= time helpers =======================
String hhmmss(long t) {
  char buf[12];
  snprintf(buf, sizeof(buf), "%02ld:%02ld:%02ld", t / 3600, (t / 60) % 60, t % 60);
  return String(buf);
}

// ======================= Wi-Fi: on only to correct the time =======================
enum NetState { NET_OFF, NET_CONNECTING, NET_SYNCING };
NetState netState = NET_OFF;
unsigned long netStateStart = 0;
unsigned long nextSyncAt    = 0;
volatile bool gotSync       = false;
bool everSynced             = false;

void onTimeSync(struct timeval *tv) { gotSync = true; }

void wifiOff() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  netState = NET_OFF;
}

void syncFailed(const char *why) {
  Serial.print("time correction failed: ");
  Serial.println(why);
  wifiOff();
  nextSyncAt = millis() + (everSynced ? RETRY_MS : FIRST_RETRY_MS);
}

// runs a little every loop, never blocks, so the seconds keep ticking
void serviceNetwork() {
  unsigned long now = millis();
  switch (netState) {
    case NET_OFF:
      if ((long)(now - nextSyncAt) >= 0) {
        Serial.println("Wi-Fi on: correcting time...");
        gotSync = false;
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        netState = NET_CONNECTING;
        netStateStart = now;
      }
      break;

    case NET_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        configTzTime(TZ_INFO, NTP_1, NTP_2);
        netState = NET_SYNCING;
        netStateStart = now;
      } else if (now - netStateStart > WIFI_TIMEOUT_MS) {
        syncFailed("no Wi-Fi");
      }
      break;

    case NET_SYNCING:
      if (gotSync) {
        everSynced = true;
        wifiOff();
        nextSyncAt = now + RESYNC_EVERY_MS;
        Serial.println("time corrected, Wi-Fi off for 1 hour");
      } else if (now - netStateStart > SYNC_TIMEOUT_MS) {
        syncFailed("no answer from time server");
      }
      break;
  }
}

// ======================= screen content =======================
void render() {
  if (!everSynced) {
    drawTime("--:--:--");
    return;
  }
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t now = tv.tv_sec;
  struct tm lt;
  localtime_r(&now, &lt);
  String t = hhmmss(lt.tm_hour * 3600L + lt.tm_min * 60L + lt.tm_sec);
  if (tv.tv_usec >= 500000) {        // colons off for the second half of each second
    t.setCharAt(2, ' ');
    t.setCharAt(5, ' ');
  }
  drawTime(t);
}

// ======================= main =======================
void setup() {
  setCpuFrequencyMhz(80);   // slower CPU = less power; still plenty for this
  Serial.begin(115200);

  // dimmable backlight (PWM)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_TFT_BL, 5000, 8);
  ledcWrite(PIN_TFT_BL, BACKLIGHT);
#else
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PIN_TFT_BL, 0);
  ledcWrite(0, BACKLIGHT);
#endif
  SPI.begin(PIN_TFT_SCLK, -1, PIN_TFT_MOSI, PIN_TFT_CS);
  tft.init(135, 240);
  tft.setSPISpeed(40000000);
  tft.setRotation(SCREEN_ROTATION);
  tft.fillScreen(C_BG);

  sntp_set_time_sync_notification_cb(onTimeSync);
  setenv("TZ", TZ_INFO, 1);
  tzset();
  nextSyncAt = millis();    // correct the time right away
}

void loop() {
  serviceNetwork();
  render();
  delay(20);
}
