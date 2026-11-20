#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#if __has_include(<esp_mac.h>)
#include <esp_mac.h>  // esp_read_mac / ESP_MAC_WIFI_STA on core 3.x
#endif
#include <cstring>
#include <stddef.h>

// ==========================================
// ESP-NOW FEASIBILITY TEST - SECOND ESP32 (RESPONDER)
//
// Waits for a HELLO broadcast from the watch, answers with HELLO_ACK
// (pairing), then echoes every PING back unchanged.
// Everything is logged on the serial monitor (115200 baud).
//
// Pair with ESPNOW_Watch.ino running on the Waveshare watch.
// ==========================================

// ==========================================
// ESP-NOW CONFIGURATION (must match ESPNOW_Watch.ino)
// ==========================================
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr size_t  UUID_SIZE      = 16;
constexpr size_t  PAYLOAD_SIZE   = 78;

enum MsgType : uint8_t {
  MSG_HELLO     = 1,
  MSG_HELLO_ACK = 2,
  MSG_PING      = 3,
  MSG_ECHO      = 4
};

struct __attribute__((packed)) Msg {
  uint8_t  magic0, magic1;       // 'S','W'
  uint8_t  type;
  uint16_t seq;
  uint8_t  uuid[UUID_SIZE];
  uint8_t  data[PAYLOAD_SIZE];
};
constexpr size_t HDR_SIZE = offsetof(Msg, data);

struct RxPacket {
  uint8_t  mac[6];
  uint8_t  len;
  int8_t   rssi;
  uint32_t rxMicros;
  Msg      msg;
};

QueueHandle_t rxQueue;
uint32_t echoCount = 0;

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
  peer.channel = 0;
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
// SETUP / LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== ESP-NOW peer (responder) ===");

  loadOrCreateUuid();
  Serial.printf("Agent UUID: %s (%s)\n", agentUuidStr,
    addrSource == ADDR_LOADED ? "loaded" :
    addrSource == ADDR_NEW ? "new, saved" :
    addrSource == ADDR_REGENERATED ? "regenerated" : "SAVE FAILED");

  rxQueue = xQueueCreate(8, sizeof(RxPacket));  // must exist before the callback

  if (!initEspNow()) {
    Serial.println("ESP-NOW init FAILED");
    while (true) delay(1000);
  }

  uint8_t m[6];
  esp_read_mac(m, ESP_MAC_WIFI_STA);
  char mac[18];
  macToStr(m, mac);
  Serial.printf("My MAC: %s  | channel %u\n", mac, ESPNOW_CHANNEL);
  Serial.println("Waiting for HELLO from the watch...");
}

void loop() {
  RxPacket p;
  if (xQueueReceive(rxQueue, &p, portMAX_DELAY) != pdTRUE) return;

  char mac[18];
  macToStr(p.mac, mac);

  switch (p.msg.type) {
    case MSG_HELLO: {
      bool ok = addPeer(p.mac) && sendMsg(p.mac, MSG_HELLO_ACK, 0, nullptr);
      char u[UUID_STR_LEN + 1];
      uuidToString(p.msg.uuid, u);
      Serial.printf("HELLO from %s (uuid %s, rssi %d) -> ACK %s\n",
                    mac, u, (int)p.rssi, ok ? "sent" : "FAILED");
      echoCount = 0;
      break;
    }
    case MSG_PING: {
      if (!addPeer(p.mac)) {
        Serial.println("addPeer failed for PING sender");
        break;
      }
      bool ok = sendMsg(p.mac, MSG_ECHO, p.msg.seq, p.msg.data);
      echoCount++;
      Serial.printf("PING seq=%u from %s rssi=%d -> echo %s (total %lu)\n",
                    p.msg.seq, mac, (int)p.rssi, ok ? "sent" : "FAILED",
                    (unsigned long)echoCount);
      break;
    }
    default:
      break;
  }
}
