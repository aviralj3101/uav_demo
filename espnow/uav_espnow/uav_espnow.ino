/*
 * UAV side (ESP32), ESP-NOW link, RAW image transfer (no encryption yet)
 * No nRF24 module needed.
 *
 * Flow:
 *   1. Base sends CMD over ESP-NOW.
 *   2. UAV writes 0x01 on USB serial. uav_webcam_responder.py replies with
 *        0xA5 0x5A | len(4, LE) | crc32(4, LE) | JPEG bytes
 *   3. UAV verifies the CRC, then sends the JPEG to the base:
 *        START (len, crc32, packet count) followed by DATA packets with sequence numbers.
 *      Every packet is sent to the base's MAC and only counts as sent when the
 *      send callback reports delivery; otherwise it is retried.
 *
 * Packet layouts (ESP-NOW max 250 bytes):
 *   CMD   : [0]=0x01 [1]=session
 *   START : [0]=0x10 [1]=session [2..5]=total_len [6..9]=crc32 [10..11]=num_pkts   (12 bytes)
 *   DATA  : [0]=0x11 [1]=session [2..3]=seq [4]=n_bytes [5..]=payload (<=240)
 *   ERR   : [0]=0x20 [1]=session [2]=error code                                    (3 bytes)
 *
 * NOTE: debug text printed on Serial must never contain byte 0x01
 *       (it is the request byte seen by the laptop script).
 *
 * The two callbacks take 'const void*' as first argument and are cast when registered.
 * This makes the same code compile on ESP32 Arduino core 2.x and 3.x.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ===================== USER SETTINGS =====================
#define SERIAL_BAUD    921600   // must match uav_webcam_responder.py (--baud)
#define ESPNOW_CHANNEL 1        // must be identical on base and UAV
// MAC of the BASE board (we send to this)
static const uint8_t PEER_MAC[6] = {0xE0, 0x8C, 0xFE, 0xE6, 0x6C, 0x20};
// =========================================================

#define MAX_IMG        60000
#define DATA_PAYLOAD   240
#define REQ_BYTE       0x01
#define CAM_TIMEOUT_MS 4000
#define APP_TRIES      10

enum : uint8_t { PKT_CMD = 0x01, PKT_START = 0x10, PKT_DATA = 0x11, PKT_ERR = 0x20 };
enum : uint8_t { ERR_CAMERA = 1, ERR_TIMEOUT = 2, ERR_TOO_BIG = 3, ERR_CRC = 4 };

static uint8_t* imgBuf = nullptr;
static uint8_t lastSession = 0;
static bool haveSession = false;

// ---------- receive ring (filled in the ESP-NOW callback, drained in loop) ----------
#define RING_SLOTS 16
struct Slot { uint8_t len; uint8_t data[250]; };
static Slot ring[RING_SLOTS];
static volatile uint8_t ringHead = 0, ringTail = 0;

void onRecv(const void* info, const uint8_t* data, int len) {
  (void)info;
  if (len <= 0 || len > 250) return;
  uint8_t next = (ringHead + 1) % RING_SLOTS;
  if (next == ringTail) return;              // full: drop
  memcpy(ring[ringHead].data, data, len);
  ring[ringHead].len = (uint8_t)len;
  ringHead = next;
}

bool ringPop(uint8_t* out, uint8_t& len) {
  if (ringTail == ringHead) return false;
  len = ring[ringTail].len;
  memcpy(out, ring[ringTail].data, len);
  ringTail = (ringTail + 1) % RING_SLOTS;
  return true;
}

// ---------- send status (set in the ESP-NOW send callback) ----------
static volatile bool sendDone = false, sendOk = false;

void onSent(const void* info, esp_now_send_status_t status) {
  (void)info;
  sendOk = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

// ---------- helpers ----------
static inline void put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline void put32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline uint32_t get32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint32_t crc32_calc(const uint8_t* d, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

// ---------- USB serial side (laptop webcam) ----------
bool readExact(uint8_t* dst, size_t n, uint32_t idleTimeoutMs) {
  size_t got = 0;
  uint32_t last = millis();
  while (got < n) {
    int avail = Serial.available();
    if (avail > 0) {
      size_t want = n - got;
      size_t take = (size_t)avail < want ? (size_t)avail : want;
      got += Serial.readBytes(dst + got, take);
      last = millis();
    } else {
      if (millis() - last > idleTimeoutMs) return false;
      delay(1);
    }
  }
  return true;
}

bool waitForHeader(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint8_t prev = 0;
  while (millis() - t0 < timeoutMs) {
    if (Serial.available()) {
      uint8_t b = Serial.read();
      if (prev == 0xA5 && b == 0x5A) return true;
      prev = b;
    } else {
      delay(1);
    }
  }
  return false;
}

// returns 0 on success, else an ERR_* code
uint8_t requestFrame(uint32_t& outLen) {
  while (Serial.available()) Serial.read();
  Serial.write((uint8_t)REQ_BYTE);
  Serial.flush();

  if (!waitForHeader(CAM_TIMEOUT_MS)) return ERR_TIMEOUT;
  uint8_t h[8];
  if (!readExact(h, 8, 1000)) return ERR_TIMEOUT;
  uint32_t n = get32(h);
  uint32_t crc = get32(h + 4);
  if (n == 0) return ERR_CAMERA;
  if (n > MAX_IMG) return ERR_TOO_BIG;
  if (!readExact(imgBuf, n, 1500)) return ERR_TIMEOUT;
  if (crc32_calc(imgBuf, n) != crc) return ERR_CRC;
  outLen = n;
  return 0;
}

// ---------- ESP-NOW side ----------
bool sendPacket(const uint8_t* pkt, size_t len, uint32_t& retries) {
  for (uint8_t a = 0; a < APP_TRIES; a++) {
    sendDone = false;
    if (esp_now_send(PEER_MAC, pkt, len) == ESP_OK) {
      uint32_t t0 = millis();
      while (!sendDone && (millis() - t0) < 100) yield();
      if (sendDone && sendOk) return true;
    }
    retries++;
    delay(2);
  }
  return false;
}

void sendError(uint8_t session, uint8_t code) {
  uint8_t pkt[3] = {PKT_ERR, session, code};
  uint32_t r = 0;
  sendPacket(pkt, 3, r);
}

bool sendImage(uint8_t session, uint32_t len) {
  uint8_t pkt[250];
  uint32_t retries = 0;
  uint16_t numPkts = (len + DATA_PAYLOAD - 1) / DATA_PAYLOAD;
  uint32_t crc = crc32_calc(imgBuf, len);
  uint32_t t0 = millis();

  memset(pkt, 0, 12);
  pkt[0] = PKT_START; pkt[1] = session;
  put32(pkt + 2, len);
  put32(pkt + 6, crc);
  put16(pkt + 10, numPkts);
  if (!sendPacket(pkt, 12, retries)) {
    Serial.println("[UAV] ABORT: START packet not delivered (base off / wrong MAC / different channel?)");
    return false;
  }

  for (uint16_t seq = 0; seq < numPkts; seq++) {
    uint32_t off = (uint32_t)seq * DATA_PAYLOAD;
    uint32_t left = len - off;
    uint8_t n = left > DATA_PAYLOAD ? DATA_PAYLOAD : (uint8_t)left;
    pkt[0] = PKT_DATA; pkt[1] = session;
    put16(pkt + 2, seq);
    pkt[4] = n;
    memcpy(pkt + 5, imgBuf + off, n);
    if (!sendPacket(pkt, 5 + n, retries)) {
      Serial.printf("[UAV] ABORT: DATA packet %u/%u failed after %u tries\n", seq, numPkts, APP_TRIES);
      return false;
    }
  }

  uint32_t dt = millis() - t0;
  Serial.printf("[UAV] sent %lu bytes in %u packets, %lu ms (%.1f kB/s), app retries: %lu\n",
                (unsigned long)len, numPkts, (unsigned long)dt,
                dt ? (len / 1.024f) / dt : 0.0f, (unsigned long)retries);
  return true;
}

void handleCapture(uint8_t session) {
  if (haveSession && session == lastSession) {
    Serial.println("[UAV] duplicate CMD ignored");
    return;
  }
  haveSession = true;
  lastSession = session;
  Serial.printf("[UAV] CMD received (session %u) - requesting frame from laptop\n", session);

  uint32_t t0 = millis();
  uint32_t len = 0;
  uint8_t err = requestFrame(len);

  if (err) {
    Serial.printf("[UAV] frame request failed, error code %u (1=camera 2=timeout 3=too big 4=crc)\n", err);
    sendError(session, err);
  } else {
    Serial.printf("[UAV] frame ready: %lu bytes (%lu ms)\n", (unsigned long)len,
                  (unsigned long)(millis() - t0));
    sendImage(session, len);
  }
  ringTail = ringHead;   // discard anything that arrived while busy
}

// ---------- Arduino ----------
void setup() {
  Serial.setRxBufferSize(8192);   // must be called before begin()
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("[UAV] boot (ESP-NOW)");

  imgBuf = (uint8_t*)malloc(MAX_IMG);
  if (!imgBuf) {
    while (true) { Serial.println("[UAV] FATAL: cannot allocate image buffer"); delay(1000); }
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    while (true) { Serial.println("[UAV] FATAL: esp_now_init failed"); delay(1000); }
  }
  esp_now_register_send_cb((esp_now_send_cb_t)onSent);
  esp_now_register_recv_cb((esp_now_recv_cb_t)onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;              // 0 = use the current channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    while (true) { Serial.println("[UAV] FATAL: could not add peer"); delay(1000); }
  }

  Serial.print("[UAV] my MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[UAV] ready on channel %d, waiting for capture command\n", ESPNOW_CHANNEL);
}

void loop() {
  uint8_t pkt[250];
  uint8_t len;
  if (ringPop(pkt, len)) {
    if (len >= 2 && pkt[0] == PKT_CMD) handleCapture(pkt[1]);
  }
}
