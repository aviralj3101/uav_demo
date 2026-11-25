/*
 * Base station (ESP32 + nRF24L01), RAW image transfer (no encryption yet)
 *
 * Wiring (nRF24 -> ESP32, default VSPI): identical to the UAV board
 *   VCC -> 3.3V (+ 10uF cap), GND, CE -> 4, CSN -> 5, SCK -> 18, MISO -> 19, MOSI -> 23
 * USB -> base laptop running base_receiver.py
 *
 * Flow:
 *   - Laptop sends ASCII '1' over USB serial.
 *   - Base sends a CMD packet to the UAV (retried up to 3 times).
 *   - Base receives START + DATA packets, checks sequence numbers, rebuilds the image
 *     in RAM and verifies the CRC-32.
 *   - If OK, it forwards to the laptop as:
 *         "IMG_START\r\n" | len(4, LE) | JPEG bytes | "\r\n" "IMG_END\r\n"
 *     Everything else printed on Serial is plain debug text.
 *
 * Packet layouts: see uav_esp32.ino (must stay identical on both sides).
 */

#include <SPI.h>
#include <RF24.h>

// ===================== USER SETTINGS =====================
#define PIN_CE      4
#define PIN_CSN     5
#define PIN_SCK     18
#define PIN_MISO    19
#define PIN_MOSI    23
#define SERIAL_BAUD 921600      // must match base_receiver.py (--baud)
#define RF_CHANNEL  76          // must match UAV
// =========================================================

#define MAX_IMG          60000
#define DATA_PAYLOAD     27
#define CMD_TRIES        3
#define START_TIMEOUT_MS 10000  // time allowed for webcam capture + first packet
#define DATA_TIMEOUT_MS  1500   // max silence between packets mid-transfer

enum : uint8_t { PKT_CMD = 0x01, PKT_START = 0x10, PKT_DATA = 0x11, PKT_ERR = 0x20 };

RF24 radio(PIN_CE, PIN_CSN);
const byte ADDR_UAV[6]  = "UAV01";   // base transmits here
const byte ADDR_BASE[6] = "BAS01";   // base listens here

static uint8_t* imgBuf = nullptr;
static uint8_t session = 0;

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
  uint8_t pkt[32];
  memset(pkt, 0, 32);
  pkt[0] = PKT_CMD;
  pkt[1] = sess;
  radio.stopListening();
  bool ok = false;
  for (int i = 0; i < CMD_TRIES && !ok; i++) {
    ok = radio.write(pkt, 32);
    if (!ok) delay(20);
  }
  radio.startListening();
  return ok;
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
  uint8_t pkt[32];
  bool started = false;
  uint32_t totalLen = 0, expectCrc = 0, got = 0, dups = 0;
  uint16_t numPkts = 0, expectSeq = 0;
  uint32_t tBegin = millis(), tLast = millis(), tFirst = 0;

  while (true) {
    if (radio.available()) {
      radio.read(pkt, 32);
      tLast = millis();
      if (pkt[1] != sess) continue;          // stale packet from an older session

      if (pkt[0] == PKT_ERR) {
        Serial.printf("[BASE] UAV reported error code %u (1=camera 2=timeout 3=too big 4=crc)\n", pkt[2]);
        return;
      }

      if (pkt[0] == PKT_START) {
        if (started) continue;               // duplicate START (lost ACK) - ignore
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

      if (pkt[0] == PKT_DATA && started) {
        uint16_t seq = get16(pkt + 2);
        uint8_t n = pkt[4];
        if (n > DATA_PAYLOAD) { Serial.println("[BASE] FAILED: corrupt DATA packet"); return; }
        if (seq == expectSeq) {
          if (got + n > totalLen) { Serial.println("[BASE] FAILED: data exceeds announced length"); return; }
          memcpy(imgBuf + got, pkt + 5, n);
          got += n;
          expectSeq++;
          if (got == totalLen) break;        // image complete
        } else if (seq < expectSeq) {
          dups++;                            // retransmission of a packet we already have
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
  radio.flush_rx();
  Serial.printf("[BASE] sending capture command (session %u)\n", session);
  if (!sendCommand(session)) {
    Serial.println("[BASE] FAILED: UAV did not acknowledge the command (check power/wiring/channel)");
    return;
  }
  Serial.println("[BASE] command acknowledged, waiting for image...");
  receiveImage(session);
  radio.flush_rx();
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("[BASE] boot");

  imgBuf = (uint8_t*)malloc(MAX_IMG);
  if (!imgBuf) {
    while (true) { Serial.println("[BASE] FATAL: cannot allocate image buffer"); delay(1000); }
  }

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin()) {
    while (true) { Serial.println("[BASE] FATAL: nRF24 not responding - check wiring/power"); delay(1000); }
  }
  radio.setPALevel(RF24_PA_LOW);        // bench testing; raise to RF24_PA_MAX for range (needs solid 3.3V)
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(RF_CHANNEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(32);
  radio.openWritingPipe(ADDR_UAV);
  radio.openReadingPipe(1, ADDR_BASE);
  radio.startListening();

  Serial.println("[BASE] ready, send '1' to capture");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') doCapture();
  }
}
