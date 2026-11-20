#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Preferences.h>
#include <esp_system.h>
#include "HWCDC.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#if __has_include(<esp_mac.h>)
#include <esp_mac.h>  // esp_read_mac / ESP_MAC_WIFI_STA on core 3.x
#endif
#include <cstring>
#include <stddef.h>

// ==========================================
// ESP-NOW FEASIBILITY TEST - WAVESHARE WATCH (INITIATOR)
//
// 1. Broadcasts HELLO until the peer answers with HELLO_ACK (pairing).
// 2. Sends PING_COUNT unicast pings carrying a 78-byte payload.
// 3. The peer echoes each one back; the watch measures RTT, loss, RSSI
//    and verifies the payload byte for byte.
//
// Pair with ESPNOW_Peer.ino running on the second ESP32.
// ==========================================

HWCDC USBSerial;

// ---- Display setup (same as Waveshare 01_HelloWorld) ----
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, 0 /* rotation */,
                                      LCD_WIDTH, LCD_HEIGHT,
                                      22, 0, 0, 0);

#define COLOR_BLACK      0x0000
#define COLOR_WHITE      0xFFFF
#define COLOR_RED        0xF800
#define COLOR_GREEN      0x07E0
#define COLOR_YELLOW     0xFFE0
#define COLOR_CYAN       0x07FF
#define COLOR_MAGENTA    0xF81F
#define COLOR_ORANGE     0xFD20
#define COLOR_DARKGREY   0x52AA
#define COLOR_LIGHTGREY  0xBDF7

// ==========================================
// ESP-NOW CONFIGURATION (must match ESPNOW_Peer.ino)
// ==========================================
constexpr uint8_t  ESPNOW_CHANNEL   = 1;
constexpr size_t   UUID_SIZE        = 16;
constexpr size_t   PAYLOAD_SIZE     = 78;
constexpr int      PING_COUNT       = 50;
constexpr uint32_t PING_TIMEOUT_MS  = 200;   // wait this long for each echo
constexpr uint32_t PING_GAP_MS      = 50;    // pause between pings
constexpr uint32_t HELLO_INTERVAL_MS = 300;

enum MsgType : uint8_t {
  MSG_HELLO     = 1,  // broadcast by the watch while looking for a peer
  MSG_HELLO_ACK = 2,  // unicast reply from the peer
  MSG_PING      = 3,  // watch -> peer, carries the payload
  MSG_ECHO      = 4   // peer -> watch, same payload returned
};

struct __attribute__((packed)) Msg {
  uint8_t  magic0, magic1;       // 'S','W'
  uint8_t  type;
  uint16_t seq;
  uint8_t  uuid[UUID_SIZE];      // sender's agent UUID
  uint8_t  data[PAYLOAD_SIZE];   // only used by PING / ECHO
};
constexpr size_t HDR_SIZE = offsetof(Msg, data);

struct RxPacket {
  uint8_t  mac[6];
  uint8_t  len;
  int8_t   rssi;
  uint32_t rxMicros;             // timestamp taken inside the receive callback
  Msg      msg;
};

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

QueueHandle_t rxQueue;
uint8_t txPayload[PAYLOAD_SIZE];

uint8_t peerMac[6]  = {0};
uint8_t peerUuid[UUID_SIZE] = {0};

// ==========================================
// DISPLAY HELPERS
// ==========================================
constexpr uint8_t TOTAL_SCREENS = 4;
int barX, barY, barW, barH;

void centerText(const char *s, int y, uint8_t size, uint16_t color) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor((gfx->width() - w) / 2 - x1, y);
  gfx->print(s);
}

void newScreen(uint8_t n) {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextWrap(false);
  char buf[8];
  snprintf(buf, sizeof(buf), "%u/%u", n, (unsigned)TOTAL_SCREENS);
  centerText(buf, 30, 2, COLOR_DARKGREY);
}

void showStatus(const char *msg, uint16_t color) {
  int y = gfx->height() / 2 + 100;
  gfx->fillRect(0, y, gfx->width(), 20, COLOR_BLACK);
  centerText(msg, y, 2, color);
}

void showErrorScreen(const char *msg) {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextWrap(false);
  int h = gfx->height();
  centerText("ERROR", h / 2 - 50, 4, COLOR_RED);
  centerText(msg, h / 2 + 10, 2, COLOR_WHITE);
  USBSerial.println(msg);
}

void macToStr(const uint8_t *m, char *out) {  // out must hold 18 bytes
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

// ==========================================
// PERSISTENT AGENT ID: UUID VERSION 4 (stored in NVS flash)
// ==========================================
constexpr size_t UUID_STR_LEN = 36;

enum AddrSource { ADDR_LOADED, ADDR_NEW, ADDR_REGENERATED, ADDR_SAVE_FAILED };

uint8_t agentUuid[UUID_SIZE] = {0};
char agentUuidStr[UUID_STR_LEN + 1] = {0};
AddrSource addrSource = ADDR_NEW;

bool validUuidV4(const uint8_t *u) {
  if ((u[6] & 0xF0) != 0x40) return false;
  if ((u[8] & 0xC0) != 0x80) return false;
  for (size_t i = 0; i < UUID_SIZE; i++) {
    if (u[i] != 0) return true;
  }
  return false;
}

void generateUuidV4(uint8_t *u) {
  uint64_t mac = ESP.getEfuseMac();
  uint8_t macBytes[8];
  memcpy(macBytes, &mac, sizeof(macBytes));
  do {
    for (size_t i = 0; i < UUID_SIZE; i += 4) {
      uint32_t r = esp_random();
      memcpy(&u[i], &r, 4);
    }
    for (size_t i = 0; i < UUID_SIZE; i++) u[i] ^= macBytes[i % 6];
    u[6] = (u[6] & 0x0F) | 0x40;
    u[8] = (u[8] & 0x3F) | 0x80;
  } while (!validUuidV4(u));
}

void uuidToString(const uint8_t *u, char *out) {
  snprintf(out, UUID_STR_LEN + 1,
           "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
           u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7],
           u[8], u[9], u[10], u[11], u[12], u[13], u[14], u[15]);
}

bool saveUuid(Preferences &prefs, const uint8_t *u) {
  uint8_t inv[UUID_SIZE];
  for (size_t i = 0; i < UUID_SIZE; i++) inv[i] = ~u[i];
  prefs.putBytes("uuid", u, UUID_SIZE);
  prefs.putBytes("uuid_chk", inv, UUID_SIZE);
  uint8_t a[UUID_SIZE], b[UUID_SIZE];
  if (prefs.getBytes("uuid", a, UUID_SIZE) != UUID_SIZE) return false;
  if (prefs.getBytes("uuid_chk", b, UUID_SIZE) != UUID_SIZE) return false;
  return memcmp(a, u, UUID_SIZE) == 0 && memcmp(b, inv, UUID_SIZE) == 0;
}

void loadOrCreateUuid() {
  Preferences prefs;
  prefs.begin("swarm", false);

  if (prefs.isKey("uuid") && prefs.isKey("uuid_chk")) {
    uint8_t u[UUID_SIZE], c[UUID_SIZE];
    bool ok = prefs.getBytes("uuid", u, UUID_SIZE) == UUID_SIZE &&
              prefs.getBytes("uuid_chk", c, UUID_SIZE) == UUID_SIZE;
    if (ok) {
      for (size_t i = 0; i < UUID_SIZE; i++) {
        if (c[i] != (uint8_t)~u[i]) { ok = false; break; }
      }
    }
    if (ok && validUuidV4(u)) {
      memcpy(agentUuid, u, UUID_SIZE);
      uuidToString(agentUuid, agentUuidStr);
      addrSource = ADDR_LOADED;
      prefs.end();
      return;
    }
    addrSource = ADDR_REGENERATED;
  } else {
    addrSource = ADDR_NEW;
  }

  generateUuidV4(agentUuid);
  uuidToString(agentUuid, agentUuidStr);
  if (!saveUuid(prefs, agentUuid)) addrSource = ADDR_SAVE_FAILED;
  prefs.end();
}

// ==========================================
// ESP-NOW LAYER
// ==========================================

// Receive callback: runs in the Wi-Fi task, so it only timestamps and queues.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
  int8_t rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  int8_t rssi = 0;
#endif
  uint32_t now = micros();
  if (len < (int)HDR_SIZE) return;

  RxPacket p;
  memset(&p, 0, sizeof(p));
  memcpy(p.mac, mac, 6);
  p.len = (uint8_t)len;
  p.rssi = rssi;
  p.rxMicros = now;
  memcpy(&p.msg, data, len < (int)sizeof(Msg) ? len : sizeof(Msg));

  if (p.msg.magic0 != 'S' || p.msg.magic1 != 'W') return;
  xQueueSend(rxQueue, &p, 0);
}

bool addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;  // 0 = use the current channel
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

bool sendMsg(const uint8_t *mac, uint8_t type, uint16_t seq, const uint8_t *data) {
  Msg m;
  memset(&m, 0, sizeof(m));
  m.magic0 = 'S';
  m.magic1 = 'W';
  m.type = type;
  m.seq = seq;
  memcpy(m.uuid, agentUuid, UUID_SIZE);
  size_t len = HDR_SIZE;
  if (data) {
    memcpy(m.data, data, PAYLOAD_SIZE);
    len = sizeof(Msg);
  }
  return esp_now_send(mac, (const uint8_t *)&m, len) == ESP_OK;
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  return true;
}

// ==========================================
// SCREENS
// ==========================================

// Screen 1: title + persistent UUID
void showIdentityScreen() {
  newScreen(1);
  int h = gfx->height();
  char l1[16], l2[12], l3[16];
  memcpy(l1, &agentUuidStr[0], 13);  l1[13] = '\0';
  memcpy(l2, &agentUuidStr[14], 9);  l2[9]  = '\0';
  memcpy(l3, &agentUuidStr[24], 12); l3[12] = '\0';

  centerText("ESP-NOW TEST", h / 2 - 120, 3, COLOR_CYAN);
  centerText("AGENT UUID",   h / 2 - 70,  2, COLOR_MAGENTA);
  centerText(l1, h / 2 - 40, 3, COLOR_WHITE);
  centerText(l2, h / 2 - 5,  3, COLOR_WHITE);
  centerText(l3, h / 2 + 30, 3, COLOR_WHITE);

  switch (addrSource) {
    case ADDR_LOADED:       showStatus("Loaded from flash", COLOR_GREEN); break;
    case ADDR_NEW:          showStatus("First boot: new UUID saved", COLOR_YELLOW); break;
    case ADDR_REGENERATED:  showStatus("Flash corrupt: new UUID", COLOR_ORANGE); break;
    case ADDR_SAVE_FAILED:  showStatus("SAVE FAILED - not persistent", COLOR_RED); break;
  }
}

// Screen 2: pairing (bouncing bar while HELLO is broadcast)
void showPairingScreen() {
  newScreen(2);
  int w = gfx->width(), h = gfx->height();
  centerText("Pairing...", h / 2 - 70, 3, COLOR_YELLOW);

  char mac[18], line[24];
  uint8_t m[6];
  esp_read_mac(m, ESP_MAC_WIFI_STA);
  macToStr(m, mac);
  snprintf(line, sizeof(line), "ME %s", mac);
  centerText(line, h / 2 - 25, 2, COLOR_LIGHTGREY);

  barW = w - 100;
  barH = 20;
  barX = (w - barW) / 2;
  barY = h / 2 + 20;
  gfx->drawRect(barX, barY, barW, barH, COLOR_WHITE);
  showStatus("Broadcasting HELLO...", COLOR_DARKGREY);
}

void drawScanTick(int tick) {
  int inner = barW - 4;
  int seg = inner / 4;
  int range = inner - seg;
  int phase = tick % (2 * 20);
  int step = phase < 20 ? phase : 40 - phase;
  gfx->fillRect(barX + 2, barY + 2, inner, barH - 4, COLOR_BLACK);
  gfx->fillRect(barX + 2 + range * step / 20, barY + 2, seg, barH - 4, COLOR_YELLOW);
}

// Screen 3: paired, ping test running
void showPairedScreen() {
  newScreen(3);
  int h = gfx->height();
  char mac[18], uuidShort[16];
  macToStr(peerMac, mac);
  char full[UUID_STR_LEN + 1];
  uuidToString(peerUuid, full);
  memcpy(uuidShort, full, 8);
  memcpy(uuidShort + 8, "...", 4);  // includes '\0'

  centerText("PAIRED", h / 2 - 100, 4, COLOR_GREEN);
  centerText("Peer MAC:",  h / 2 - 30, 2, COLOR_LIGHTGREY);
  centerText(mac,          h / 2 - 2,  2, COLOR_WHITE);
  centerText("Peer UUID:", h / 2 + 30, 2, COLOR_LIGHTGREY);
  centerText(uuidShort,    h / 2 + 58, 2, COLOR_WHITE);
}

void fmtMs(uint32_t us, char *out, size_t n) {
  snprintf(out, n, "%lu.%02lu ms", (unsigned long)(us / 1000), (unsigned long)((us % 1000) / 10));
}

// Screen 4: final result
void showResultScreen(int sent, int received, int bad,
                      uint32_t minUs, uint32_t maxUs, uint64_t sumUs, int lastRssi) {
  newScreen(4);
  int lost = sent - received;
  int lossPct = sent > 0 ? (lost * 100) / sent : 0;

  const char *verdict;
  uint16_t col;
  if (received == 0)                 { verdict = "FAIL";    col = COLOR_RED; }
  else if (lost == 0 && bad == 0)    { verdict = "PASS";    col = COLOR_GREEN; }
  else                               { verdict = "PARTIAL"; col = COLOR_ORANGE; }

  char buf[32], a[16], b[16];
  centerText("ESP-NOW", 70, 3, COLOR_CYAN);
  centerText(verdict, 110, 4, col);

  snprintf(buf, sizeof(buf), "Sent %d  Rcvd %d", sent, received);
  centerText(buf, 170, 2, COLOR_WHITE);
  snprintf(buf, sizeof(buf), "Lost %d (%d%%)  Bad %d", lost, lossPct, bad);
  centerText(buf, 200, 2, lost ? COLOR_ORANGE : COLOR_LIGHTGREY);

  if (received > 0) {
    fmtMs((uint32_t)(sumUs / received), a, sizeof(a));
    snprintf(buf, sizeof(buf), "RTT avg %s", a);
    centerText(buf, 240, 2, COLOR_CYAN);
    fmtMs(minUs, a, sizeof(a));
    fmtMs(maxUs, b, sizeof(b));
    snprintf(buf, sizeof(buf), "min %s", a);
    centerText(buf, 270, 2, COLOR_LIGHTGREY);
    snprintf(buf, sizeof(buf), "max %s", b);
    centerText(buf, 300, 2, COLOR_LIGHTGREY);
    snprintf(buf, sizeof(buf), "RSSI %d dBm", lastRssi);
    centerText(buf, 340, 2, COLOR_YELLOW);
  }
}

// ==========================================
// 78-BYTE TEST PAYLOAD (ABC...Z repeated three times)
// ==========================================
void preparePayload() {
  for (size_t i = 0; i < PAYLOAD_SIZE; i++) txPayload[i] = 'A' + (i % 26);
}

// ==========================================
// SETUP: runs the whole test once
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

  loadOrCreateUuid();
  USBSerial.printf("Agent UUID: %s\n", agentUuidStr);
  preparePayload();

  rxQueue = xQueueCreate(8, sizeof(RxPacket));  // must exist before the callback

  // ---- Screen 1 ----
  showIdentityScreen();
  delay(3000);

  if (!initEspNow()) {
    showErrorScreen("ESP-NOW Init Failed");
    return;
  }
  if (!addPeer(BCAST)) {
    showErrorScreen("Broadcast Peer Failed");
    return;
  }

  // ---- Screen 2: pair ----
  showPairingScreen();
  USBSerial.println("Broadcasting HELLO until a peer answers...");

  bool paired = false;
  int tick = 0;
  uint32_t lastHello = 0;
  RxPacket p;

  while (!paired) {
    if (millis() - lastHello >= HELLO_INTERVAL_MS) {
      sendMsg(BCAST, MSG_HELLO, 0, nullptr);
      lastHello = millis();
    }
    while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {
      if (p.msg.type == MSG_HELLO_ACK) {
        memcpy(peerMac, p.mac, 6);
        memcpy(peerUuid, p.msg.uuid, UUID_SIZE);
        paired = addPeer(peerMac);
        break;
      }
    }
    drawScanTick(tick++);
    delay(40);
  }

  char macStr[18];
  macToStr(peerMac, macStr);
  USBSerial.printf("Paired with %s\n", macStr);

  // ---- Screen 3: ping test ----
  showPairedScreen();
  delay(1500);

  int sent = 0, received = 0, bad = 0, lastRssi = 0;
  uint32_t minUs = 0xFFFFFFFF, maxUs = 0;
  uint64_t sumUs = 0;

  USBSerial.println("seq,rtt_us,rssi");

  for (int seq = 1; seq <= PING_COUNT; seq++) {
    while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {}  // drop stale packets

    uint32_t t0 = micros();
    if (!sendMsg(peerMac, MSG_PING, (uint16_t)seq, txPayload)) {
      USBSerial.printf("%d,send_fail,0\n", seq);
      delay(PING_GAP_MS);
      continue;
    }
    sent++;

    bool got = false;
    uint32_t deadline = millis() + PING_TIMEOUT_MS;
    while (!got && (int32_t)(deadline - millis()) > 0) {
      if (xQueueReceive(rxQueue, &p, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (p.msg.type == MSG_ECHO && p.msg.seq == seq && memcmp(p.mac, peerMac, 6) == 0) {
          uint32_t rtt = p.rxMicros - t0;
          bool intact = (p.len == sizeof(Msg)) &&
                        (memcmp(p.msg.data, txPayload, PAYLOAD_SIZE) == 0);
          if (!intact) bad++;
          received++;
          sumUs += rtt;
          if (rtt < minUs) minUs = rtt;
          if (rtt > maxUs) maxUs = rtt;
          lastRssi = p.rssi;
          USBSerial.printf("%d,%lu,%d\n", seq, (unsigned long)rtt, (int)p.rssi);
          got = true;
        }
      }
    }
    if (!got) USBSerial.printf("%d,timeout,0\n", seq);

    char st[24];
    snprintf(st, sizeof(st), "Ping %d/%d", seq, PING_COUNT);
    showStatus(st, COLOR_YELLOW);
    delay(PING_GAP_MS);
  }

  // ---- Screen 4: result ----
  showResultScreen(sent, received, bad, minUs, maxUs, sumUs, lastRssi);
  USBSerial.printf("DONE sent=%d rcvd=%d lost=%d bad=%d\n", sent, received, sent - received, bad);
}

void loop() {
  delay(1000);
}
