/*
 * Base station (ESP32), ESP-NOW link, RAW image transfer (no encryption yet)
 * No nRF24 module needed.
 *
 * Flow:
 *   - Laptop sends ASCII '1' over USB serial.
 *   - Base sends a CMD packet to the UAV over ESP-NOW (retried until delivered).
 *   - Base receives START + DATA packets, checks sequence numbers, rebuilds the
 *     image in RAM and verifies the CRC-32.
 *   - If OK, it forwards to the laptop as:
 *         "IMG_START\r\n" | len(4, LE) | JPEG bytes | "\r\n" "IMG_END\r\n"
 *     Everything else printed on Serial is plain debug text.
 *
 * Packet layouts: see uav_espnow.ino (must stay identical on both sides).
 *
 * The two callbacks take 'const void*' as first argument and are cast when registered.
 * This makes the same code compile on ESP32 Arduino core 2.x and 3.x.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ===================== USER SETTINGS =====================
#define SERIAL_BAUD    921600   // must match base_receiver.py (--baud)
#define ESPNOW_CHANNEL 1        // must be identical on base and UAV
// MAC of the UAV board (we send commands to this)
static const uint8_t PEER_MAC[6] = {0xFC, 0xE8, 0xC0, 0xE1, 0xB2, 0xE8};
// =========================================================

#define MAX_IMG          60000
#define DATA_PAYLOAD     240
#define CMD_TRIES        5
#define START_TIMEOUT_MS 10000  // time allowed for webcam capture + first packet
#define DATA_TIMEOUT_MS  1500   // max silence between packets mid-transfer

enum : uint8_t { PKT_CMD = 0x01, PKT_START = 0x10, PKT_DATA = 0x11, PKT_ERR = 0x20 };

static uint8_t* imgBuf = nullptr;
static uint8_t session = 0;

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

// ---------- send status ----------
static volatile bool sendDone = false, sendOk = false;

void onSent(const void* info, esp_now_send_status_t status) {
  (void)info;
  sendOk = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

// ---------- helpers ----------
static inline void put32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline uint16_t get16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
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

bool sendCommand(uint8_t sess) {
  uint8_t pkt[2] = {PKT_CMD, sess};
  for (int i = 0; i < CMD_TRIES; i++) {
    sendDone = false;
    if (esp_now_send(PEER_MAC, pkt, 2) == ESP_OK) {
      uint32_t t0 = millis();
      while (!sendDone && (millis() - t0) < 100) yield();
      if (sendDone && sendOk) return true;
    }
    delay(20);
  }
  return false;
}

void forwardToLaptop(uint32_t len) {
  uint8_t lb[4];
  put32(lb, len);
  Serial.println("IMG_START");
  Serial.write(lb, 4);
  uint32_t off = 0;
  while (off < len) {
    uint32_t n = len - off;
    if (n > 1024) n = 1024;
    Serial.write(imgBuf + off, n);
    off += n;
  }
  Serial.flush();
  Serial.println();
  Serial.println("IMG_END");
}

void receiveImage(uint8_t sess) {
  uint8_t pkt[250];
  uint8_t plen;
  bool started = false;
  uint32_t totalLen = 0, expectCrc = 0, got = 0, dups = 0;
  uint16_t numPkts = 0, expectSeq = 0;
  uint32_t tBegin = millis(), tLast = millis(), tFirst = 0;

  while (true) {
    if (ringPop(pkt, plen)) {
      tLast = millis();
      if (plen < 2 || pkt[1] != sess) continue;   // stale packet from an older session

      if (pkt[0] == PKT_ERR && plen >= 3) {
        Serial.printf("[BASE] UAV reported error code %u (1=camera 2=timeout 3=too big 4=crc)\n", pkt[2]);
        return;
      }

      if (pkt[0] == PKT_START && plen >= 12) {
        if (started) continue;                    // duplicate START (lost ACK) - ignore
        totalLen = get32(pkt + 2);
        expectCrc = get32(pkt + 6);
        numPkts = get16(pkt + 10);
        if (totalLen == 0 || totalLen > MAX_IMG) {
          Serial.printf("[BASE] FAILED: bad image length %lu\n", (unsigned long)totalLen);
          return;
        }
        started = true; got = 0; expectSeq = 0; tFirst = millis();
        continue;
      }

      if (pkt[0] == PKT_DATA && started && plen >= 5) {
        uint16_t seq = get16(pkt + 2);
        uint8_t n = pkt[4];
        if (n > DATA_PAYLOAD || plen < 5 + n) { Serial.println("[BASE] FAILED: corrupt DATA packet"); return; }
        if (seq == expectSeq) {
          if (got + n > totalLen) { Serial.println("[BASE] FAILED: data exceeds announced length"); return; }
          memcpy(imgBuf + got, pkt + 5, n);
          got += n;
          expectSeq++;
          if (got == totalLen) break;             // image complete
        } else if (seq < expectSeq) {
          dups++;                                 // retransmission of a packet we already have
        } else {
          Serial.printf("[BASE] FAILED: gap in sequence (expected %u, got %u)\n", expectSeq, seq);
          return;
        }
      }
    } else {
      uint32_t limit = started ? DATA_TIMEOUT_MS : START_TIMEOUT_MS;
      if (millis() - tLast > limit) {
        if (started)
          Serial.printf("[BASE] FAILED: transfer stalled at %lu/%lu bytes (packet %u/%u)\n",
                        (unsigned long)got, (unsigned long)totalLen, expectSeq, numPkts);
        else
          Serial.println("[BASE] FAILED: no response from UAV (timeout)");
        return;
      }
      yield();
    }
  }

  uint32_t dt = millis() - tFirst;
  uint32_t crc = crc32_calc(imgBuf, totalLen);
  if (expectSeq != numPkts || crc != expectCrc) {
    Serial.printf("[BASE] FAILED: integrity check (pkts %u/%u, crc %08lX vs %08lX)\n",
                  expectSeq, numPkts, (unsigned long)crc, (unsigned long)expectCrc);
    return;
  }
  Serial.printf("[BASE] image OK: %lu bytes, %u packets, %lu ms radio time (%.1f kB/s), dup pkts: %lu, total %lu ms\n",
                (unsigned long)totalLen, numPkts, (unsigned long)dt,
                dt ? (totalLen / 1.024f) / dt : 0.0f, (unsigned long)dups,
                (unsigned long)(millis() - tBegin));
  forwardToLaptop(totalLen);
}

void doCapture() {
  session++;
  ringTail = ringHead;   // discard stale packets
  Serial.printf("[BASE] sending capture command (session %u)\n", session);
  if (!sendCommand(session)) {
    Serial.println("[BASE] FAILED: command not delivered (UAV off? wrong MAC? different channel?)");
    return;
  }
  Serial.println("[BASE] command delivered, waiting for image...");
  receiveImage(session);
  ringTail = ringHead;
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("[BASE] boot (ESP-NOW)");

  imgBuf = (uint8_t*)malloc(MAX_IMG);
  if (!imgBuf) {
    while (true) { Serial.println("[BASE] FATAL: cannot allocate image buffer"); delay(1000); }
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    while (true) { Serial.println("[BASE] FATAL: esp_now_init failed"); delay(1000); }
  }
  esp_now_register_send_cb((esp_now_send_cb_t)onSent);
  esp_now_register_recv_cb((esp_now_recv_cb_t)onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;              // 0 = use the current channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    while (true) { Serial.println("[BASE] FATAL: could not add peer"); delay(1000); }
  }

  Serial.print("[BASE] my MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[BASE] ready on channel %d, send '1' to capture\n", ESPNOW_CHANNEL);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') doCapture();
  }
}
