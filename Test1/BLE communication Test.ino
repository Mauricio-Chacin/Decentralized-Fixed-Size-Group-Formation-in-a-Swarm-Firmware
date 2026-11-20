#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"
#include <NimBLEDevice.h>
#include <cstring>
#include <string>

// ==========================================
// DECENTRALIZED FIXED-SIZE GROUP FORMATION IN A SWARM
// WAVESHARE WATCH - BLE CLIENT + 4 STATUS SCREENS
// ==========================================

HWCDC USBSerial;

// ---- Display setup (same as Waveshare 01_HelloWorld) ----
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, 0 /* rotation */,
                                      LCD_WIDTH, LCD_HEIGHT,
                                      22, 0, 0, 0);

// ==========================================
// BLE CONFIGURATION (must match the other ESP32)
// ==========================================
#define SERVICE_UUID "7d4a0001-8a11-4f22-9c33-123456789abc"
#define WRITE_UUID   "7d4a0002-8a11-4f22-9c33-123456789abc"
#define ECHO_UUID    "7d4a0003-8a11-4f22-9c33-123456789abc"
#define COLOR_BLACK      0x0000
#define COLOR_WHITE      0xFFFF
#define COLOR_RED        0xF800
#define COLOR_GREEN      0x07E0
#define COLOR_BLUE       0x001F
#define COLOR_YELLOW     0xFFE0
#define COLOR_CYAN       0x07FF
#define COLOR_MAGENTA    0xF81F
#define COLOR_ORANGE     0xFD20
#define COLOR_DARKGREY   0x52AA
#define COLOR_GREY       0x8410
#define COLOR_LIGHTGREY  0xBDF7
constexpr size_t PAYLOAD_SIZE = 78;
constexpr uint16_t BLE_MTU = 185;

uint8_t txPayload[PAYLOAD_SIZE];
uint8_t rxPayload[PAYLOAD_SIZE];

volatile bool echoReceived = false;
volatile uint32_t echoTimeMs = 0;
size_t rxLength = 0;

const NimBLEAdvertisedDevice *targetDevice = nullptr;
volatile bool deviceFound = false;
std::string peerName = "";
std::string peerAddr = "";

// ==========================================
// DISPLAY HELPERS
// ==========================================

// Progress bar geometry (set by showStartingBleScreen)
int barX, barY, barW, barH;

// Draw a string horizontally centered at the given y
void centerText(const char *s, int y, uint8_t size, uint16_t color) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor((gfx->width() - w) / 2 - x1, y);
  gfx->print(s);
}

// Clear screen and draw a small "n/4" step indicator at the top
void newScreen(uint8_t n) {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextWrap(false);
  char buf[8];
  snprintf(buf, sizeof(buf), "%u/4", n);
  centerText(buf, 30, 2, COLOR_DARKGREY);
}

// Replace the status line near the bottom of the current screen
void showStatus(const char *msg, uint16_t color) {
  int y = gfx->height() / 2 + 100;
  gfx->fillRect(0, y, gfx->width(), 20, COLOR_BLACK);
  centerText(msg, y, 2, color);
}

// Red error screen (replaces displayError)
void showErrorScreen(const char *msg) {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextWrap(false);
  int h = gfx->height();
  centerText("ERROR", h / 2 - 50, 4, COLOR_RED);
  centerText(msg, h / 2 + 10, 2, COLOR_WHITE);
  USBSerial.println(msg);
}

// ==========================================
// THE 4 SCREENS
// ==========================================

// Screen 1: project title
void showTitleScreen() {
  newScreen(1);
  int h = gfx->height();
  int y = h / 2 - 90;
  centerText("Decentralized",    y,       3, COLOR_CYAN);
  centerText("Fixed-Size",       y + 40,  3, COLOR_CYAN);
  centerText("Group Formation",  y + 80,  3, COLOR_CYAN);
  centerText("in a Swarm",       y + 120, 3, COLOR_CYAN);
  centerText("Project 1", h - 70, 2, COLOR_WHITE);
}

// Screen 2: starting BLE (progress bar is driven by real BLE steps)
void showStartingBleScreen() {
  newScreen(2);
  int w = gfx->width(), h = gfx->height();
  centerText("Starting BLE...", h / 2 - 40, 3, COLOR_YELLOW);

  barW = w - 100;
  barH = 20;
  barX = (w - barW) / 2;
  barY = h / 2 + 20;
  gfx->drawRect(barX, barY, barW, barH, COLOR_WHITE);
}

// Fill the bar to a percentage (0-100)
void drawProgress(int percent) {
  gfx->fillRect(barX + 2, barY + 2, barW - 4, barH - 4, COLOR_BLACK);
  gfx->fillRect(barX + 2, barY + 2, (barW - 4) * percent / 100, barH - 4, COLOR_YELLOW);
}

// Bouncing segment shown while scanning for the peer
void drawScanTick(int tick) {
  int inner = barW - 4;
  int seg = inner / 4;
  int range = inner - seg;
  int phase = tick % (2 * 20);
  int step = phase < 20 ? phase : 40 - phase;
  gfx->fillRect(barX + 2, barY + 2, inner, barH - 4, COLOR_BLACK);
  gfx->fillRect(barX + 2 + range * step / 20, barY + 2, seg, barH - 4, COLOR_YELLOW);
}

// Screen 3: another BLE device acknowledged
void showDeviceFoundScreen(const char *name, const char *addr) {
  newScreen(3);
  int h = gfx->height();
  centerText("BLE Device", h / 2 - 80, 3, COLOR_GREEN);
  centerText("Found!",     h / 2 - 40, 3, COLOR_GREEN);
  centerText(name,         h / 2 + 20, 2, COLOR_WHITE);
  centerText(addr,         h / 2 + 50, 2, COLOR_LIGHTGREY);
}

// Screen 4a: payload sent from watch to the other ESP
void showSentScreen() {
  newScreen(4);
  int h = gfx->height();
  char row[27];
  memcpy(row, txPayload, 26);
  row[26] = '\0';
  centerText("SENT", h / 2 - 70, 4, COLOR_ORANGE);
  centerText("Payload: 78 bytes", h / 2 + 10, 2, COLOR_LIGHTGREY);
  centerText(row, h / 2 + 40, 2, COLOR_WHITE);
  showStatus("Waiting for reply...", COLOR_DARKGREY);
}

// Screen 4b: payload returned from the other ESP
void showReturnedScreen(bool valid, uint32_t roundTripMs) {
  newScreen(4);
  centerText("SENT", 70, 3, COLOR_ORANGE);
  centerText("RETURNED", 115, 4, valid ? COLOR_GREEN : COLOR_RED);

  char buf[32];
  if (!valid) {
    centerText("Verification: FAIL", 170, 2, COLOR_RED);
    snprintf(buf, sizeof(buf), "Got %u bytes", (unsigned int)rxLength);
    centerText(buf, 200, 2, COLOR_LIGHTGREY);
    return;
  }

  centerText("PASS", 170, 3, COLOR_GREEN);
  snprintf(buf, sizeof(buf), "RTT: %lu ms", (unsigned long)roundTripMs);
  centerText(buf, 215, 2, COLOR_CYAN);
  centerText("Echo payload:", 250, 2, COLOR_LIGHTGREY);

  // All 78 ASCII bytes in 3 rows of 26
  for (int row = 0; row < 3; row++) {
    char line[27];
    memcpy(line, &rxPayload[row * 26], 26);
    line[26] = '\0';
    centerText(line, 280 + row * 30, 2, COLOR_WHITE);
  }
}

// ==========================================
// CREATE EXACTLY 78 BYTES
// ==========================================
void preparePayload() {
  // ABCDEFGHIJKLMNOPQRSTUVWXYZ repeated three times = 78 bytes
  for (size_t i = 0; i < PAYLOAD_SIZE; i++) {
    txPayload[i] = 'A' + (i % 26);
  }
  USBSerial.println("78-byte payload prepared:");
  USBSerial.write(txPayload, PAYLOAD_SIZE);
  USBSerial.println();
}

// ==========================================
// BLE CALLBACKS
// ==========================================
class SwarmScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *device) override {
    if (deviceFound) return;

    if (device->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) {
      targetDevice = device;
      peerName = device->getName();
      peerAddr = device->getAddress().toString();
      deviceFound = true;  // set last so the main task sees complete data

      NimBLEDevice::getScan()->stop();
    }
  }
};

SwarmScanCallbacks scanCallbacks;

void onEchoReceived(NimBLERemoteCharacteristic *characteristic,
                    uint8_t *data, size_t length, bool isNotify) {
  if (echoReceived) return;

  echoTimeMs = millis();
  rxLength = length;
  size_t copyLength = length < PAYLOAD_SIZE ? length : PAYLOAD_SIZE;
  memcpy(rxPayload, data, copyLength);

  echoReceived = true;  // main task displays the result
}

// ==========================================
// SETUP: runs the whole exchange once
// ==========================================
void setup() {
  USBSerial.begin(115200);

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    USBSerial.println("Display initialization failed!");
    return;
  }
  //gfx->setBrightness(200);  // remove if your Arduino_GFX version lacks it

  preparePayload();

  // ---- Screen 1: title ----
  showTitleScreen();
  delay(3000);

  // ---- Screen 2: starting BLE ----
  showStartingBleScreen();
  drawProgress(10);

  NimBLEDevice::init("SWARM-WATCH");
  NimBLEDevice::setMTU(BLE_MTU);
  drawProgress(30);

  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(60);

  USBSerial.println("Scanning for Swarm ESP32...");
  scan->start(0);  // 0 = scan until stopped
  showStatus("Scanning for peer...", COLOR_LIGHTGREY);

  int tick = 0;
  while (!deviceFound) {
    drawScanTick(tick++);
    delay(40);
  }
  drawProgress(100);
  USBSerial.println("Target ESP32 found.");
  delay(300);

  // ---- Screen 3: device found ----
  showDeviceFoundScreen(peerName.empty() ? "(no name)" : peerName.c_str(),
                        peerAddr.c_str());
  showStatus("Connecting...", COLOR_YELLOW);

  NimBLEClient *client = NimBLEDevice::createClient();
  client->setConnectTimeout(10000);

  if (!client->connect(targetDevice)) {
    showErrorScreen("BLE Connection Failed");
    return;
  }

  delay(1000);  // let the connection settle

  if (!client->isConnected()) {
    showErrorScreen("BLE Connection Lost");
    return;
  }

  // Connection must carry 78-byte characteristic values
  if (client->getMTU() < 81) {
    client->exchangeMTU();
  }
  if (client->getMTU() < 81) {
    showErrorScreen("BLE MTU Too Small");
    client->disconnect();
    return;
  }

  showStatus("Discovering service...", COLOR_YELLOW);

  NimBLERemoteService *service = client->getService(SERVICE_UUID);
  if (service == nullptr) {
    showErrorScreen("BLE Service Not Found");
    client->disconnect();
    return;
  }

  NimBLERemoteCharacteristic *writeChar = service->getCharacteristic(WRITE_UUID);
  NimBLERemoteCharacteristic *echoChar  = service->getCharacteristic(ECHO_UUID);

  if (writeChar == nullptr || echoChar == nullptr) {
    showErrorScreen("BLE Characteristic Error");
    client->disconnect();
    return;
  }

  if (!writeChar->canWrite() || !echoChar->canNotify()) {
    showErrorScreen("BLE Properties Error");
    client->disconnect();
    return;
  }

  // Subscribe BEFORE sending the payload
  if (!echoChar->subscribe(true, onEchoReceived)) {
    showErrorScreen("BLE Subscribe Failed");
    client->disconnect();
    return;
  }

  delay(300);

  if (!client->isConnected()) {
    showErrorScreen("BLE Connection Lost");
    return;
  }

  showStatus("Connected!", COLOR_GREEN);
  USBSerial.println("Connection established");
  delay(1200);

  // ---- Screen 4: sent ----
  showSentScreen();
  USBSerial.println("Sending 78-byte payload...");

  uint32_t t0 = millis();
  bool sent = writeChar->writeValue(txPayload, PAYLOAD_SIZE, true);

  if (!sent) {
    showErrorScreen("Payload Send Failed");
    client->disconnect();
    return;
  }
  USBSerial.println("Payload write completed.");

  // Wait for the other ESP32 to echo it back (10 s timeout)
  unsigned long startTime = millis();
  while (!echoReceived && (millis() - startTime < 10000)) {
    delay(20);
  }

  if (!echoReceived) {
    showErrorScreen("Echo Timeout");
    client->disconnect();
    return;
  }

  // ---- Screen 4: returned ----
  bool valid = (rxLength == PAYLOAD_SIZE) &&
               (memcmp(txPayload, rxPayload, PAYLOAD_SIZE) == 0);
  showReturnedScreen(valid, echoTimeMs - t0);

  USBSerial.println(valid ? "Payload verification PASS" : "Payload verification FAILED");
  USBSerial.println("Returned payload:");
  USBSerial.write(rxPayload, rxLength < PAYLOAD_SIZE ? rxLength : PAYLOAD_SIZE);
  USBSerial.println();
  USBSerial.println("Exchange finished.");

  // No reconnection or retransmission; final result stays on screen.
}

void loop() {
  delay(1000);
}
