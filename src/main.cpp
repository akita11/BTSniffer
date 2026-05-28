#include <M5Unified.h>
#include <FastLED.h>
#include <WiFi.h>
#include <time.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <SD.h>
#include <SPI.h>
#include "esp_efuse.h"

// ピン番号は platformio.ini の build_flags で環境ごとに定義
// M5Capsule:   PIN_LED=21, SD_SCK=14, SD_MISO=39, SD_MOSI=12, SD_CS_PIN=11
// ATOM Lite:   PIN_LED=27, SD_SCK=23, SD_MISO=33, SD_MOSI=19, SD_CS_PIN=4

#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Circular buffer: BLE callback writes here, main loop drains to SD
#define LINE_LENGTH 128
#define LINE_BUF_SIZE 64
char lineBuf[LINE_BUF_SIZE][LINE_LENGTH];
volatile uint8_t pBuf_r = 0;
volatile uint8_t pBuf_w = 0;
volatile bool needScanRestart = false;

BLEScan* pBLEScan;
File logFile;
String logFilename = "";
bool isScanning = false;
int blinkState = 0;
unsigned long lastBlink = 0;

// ======== LED制御 ========
void setLED(uint32_t color, int brightness = 100, int duration = 0) {
  leds[0] = CRGB(
    ((color >> 16) & 0xFF) * brightness / 100,
    ((color >>  8) & 0xFF) * brightness / 100,
    ( color        & 0xFF) * brightness / 100
  );
  FastLED.show();
  if (duration > 0) {
    delay(duration);
    leds[0] = CRGB::Black;
    FastLED.show();
  }
}

// ======== NTP同期（タイムアウト付き） ========
void syncNTP() {
  configTime(9 * 3600, 0, "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");
  printf("Waiting for NTP time sync");
  int retry = 0;
  while (time(nullptr) < 100000 && retry < 20) {
    printf(".");
    delay(500);
    retry++;
  }
  printf("\n");
  printf("%s\n", time(nullptr) >= 100000 ? "NTP time synced" : "NTP sync timeout");
}

// ======== タイムスタンプ ========
String getTimeStamp(bool filenameMode = false) {
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char buf[32];
  if (filenameMode) {
    snprintf(buf, sizeof(buf), "log_%02d%02d%02d_%02d%02d%02d.csv",
             t->tm_year % 100, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
  } else {
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
  }
  return String(buf);
}

// ======== BLEコールバック：循環バッファに書くだけ（SD操作なし） ========
class BTCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (!isScanning) return;
    uint8_t next = (pBuf_w + 1) % LINE_BUF_SIZE;
    if (next == pBuf_r) return;  // バッファ満杯時はドロップ（ブロックしない）
    String t = getTimeStamp();
    snprintf(lineBuf[pBuf_w], LINE_LENGTH, "%s,%s,%d\n",
             t.c_str(), dev.getAddress().toString().c_str(), dev.getRSSI());
    pBuf_w = next;
  }
};

BTCallback btCallback;

// スキャン完了コールバック：メインループへ再スキャン要求を通知
void onScanComplete(BLEScanResults results) {
  needScanRestart = true;
}

// ======== スキャン開始 ========
void startScan() {
  isScanning = true;
  logFilename = "/" + getTimeStamp(true);
  printf("Logging to: %s\n", logFilename.c_str());
  setLED(0x000020, 8, 20);
  setLED(0x404040, 8);
  pBLEScan->clearResults();
  pBLEScan->start(3, onScanComplete, false);
  printf("Scan started.\n");
}

// ======== スキャン停止 ========
void stopScan() {
  isScanning = false;
  needScanRestart = false;
  pBLEScan->stop();
  setLED(0xFF0000, 16, 1000);
  setLED(0x000000);
  printf("Scan stopped.\n");
}

// ======== セットアップ ========
void setup() {
  M5.begin();
  FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, NUM_LEDS);
  Serial.begin(115200);
  delay(500);

  // SD初期化（wifi.txt / mac.txt の読み書きに必要なため先に行う）
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);  // ピンは platformio.ini の build_flags で定義
  if (!SD.begin(SD_CS_PIN)) {
    printf("SD card init failed!\n");
    setLED(0xFF0000, 16, 2000);
  } else {
    printf("SD card initialized.\n");
  }

  // デバイスMACアドレスを /mac.txt に記録（新規または変化時のみ書き込む）
  uint8_t mac[6];
  char mac_str[32];
  esp_efuse_mac_get_default(mac);
  sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  printf("Device MAC: %s\n", mac_str);

  bool fWrite_mac = true;
  File mf = SD.open("/mac.txt");
  if (mf) {
    char stored[32] = {0};
    int i = 0;
    while (mf.available() && i < 31) {
      char c = mf.read();
      if (c == 0x0d || c == 0x0a) break;
      stored[i++] = c;
    }
    mf.close();
    if (strcmp(stored, mac_str) == 0) fWrite_mac = false;
  }
  if (fWrite_mac) {
    File mfw = SD.open("/mac.txt", FILE_WRITE);
    if (mfw) {
      mfw.printf("%s\r\n", mac_str);
      mfw.close();
      printf("mac.txt written.\n");
    }
  }

  // /wifi.txt からSSID・パスワードを読み込む
  // フォーマット: 1行目=SSID, 2行目=パスワード (CR+LF区切り)
  char ssid[64] = {0};
  char pwd[64] = {0};
  bool hasWifi = false;
  File wf = SD.open("/wifi.txt");
  if (wf) {
    int i = 0;
    while (wf.available() && i < 63) {
      char c = wf.read();
      if (c == 0x0d || c == 0x0a) break;
      ssid[i++] = c;
    }
    // 改行文字を読み飛ばしてからパスワードを読む
    while (wf.available()) {
      char c = wf.read();
      if (c != 0x0d && c != 0x0a) { pwd[0] = c; break; }
    }
    int j = 1;
    while (wf.available() && j < 63) {
      char c = wf.read();
      if (c == 0x0d || c == 0x0a) break;
      pwd[j++] = c;
    }
    wf.close();
    if (strlen(ssid) > 0) hasWifi = true;
    printf("WiFi credentials loaded: SSID=%s\n", ssid);
  } else {
    printf("wifi.txt not found, skipping WiFi.\n");
  }

  // WiFi接続 + NTP同期
  if (hasWifi) {
    WiFi.begin(ssid, pwd);
    printf("Connecting to WiFi");
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
      delay(500);
      printf(".");
      retries++;
    }
    printf("\n");
    if (WiFi.status() == WL_CONNECTED) {
      printf("WiFi connected.\n");
      setLED(0x00FF00, 16);
      syncNTP();
    } else {
      printf("WiFi connection failed.\n");
      setLED(0xFFFF00, 16, 2000);
    }
  }

  // BLE初期化：コールバックを登録して準備
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(&btCallback);
  pBLEScan->setActiveScan(true);

  setLED(0xFFFFFF, 16);
}

// ======== メインループ ========
void loop() {
  M5.update();

  // ボタン押下でスキャン開始/停止
  if (M5.BtnA.wasPressed()) {
    if (!isScanning) {
      startScan();
    } else {
      stopScan();
    }
  }

  // スタンバイ時LED点滅
  if (!isScanning) {
    if (millis() - lastBlink > 500) {
      blinkState = !blinkState;
      lastBlink = millis();
      setLED(blinkState ? 0xFFFFFF : 0x000000, 16);
    }
  }

  // スキャン完了後の再スキャン（3秒スキャンを繰り返す）
  if (isScanning && needScanRestart) {
    needScanRestart = false;
    pBLEScan->clearResults();
    pBLEScan->start(3, onScanComplete, false);
  }

  // 循環バッファにデータがあればSDに一括書き込み（open→全件write→close）
  if (isScanning && pBuf_r != pBuf_w) {
    logFile = SD.open(logFilename.c_str(), FILE_APPEND);
    if (logFile) {
      while (pBuf_r != pBuf_w) {
        logFile.print(lineBuf[pBuf_r]);
        printf("%s", lineBuf[pBuf_r]);
        pBuf_r = (pBuf_r + 1) % LINE_BUF_SIZE;
      }
      logFile.close();
    }
    setLED(0x0000FF, 8, 20);  // 青フラッシュ（書き込み確認）
    setLED(0x404040, 8);
  }

  delay(1);  // WDT対策
}
