/*
 * UAV side (ESP32 + nRF24L01), RAW image transfer (no encryption yet)
 *
 * Wiring (nRF24 -> ESP32, default VSPI):
 *   VCC -> 3.3V (+ 10uF cap across VCC/GND at the module)
 *   GND -> GND
 *   CE  -> GPIO 4
 *   CSN -> GPIO 5
 *   SCK -> GPIO 18
 *   MISO-> GPIO 19
 *   MOSI-> GPIO 23
 *   (IRQ not used)
 * USB -> laptop that has the webcam, running uav_webcam_responder.py
 *
 * Flow:
 *   1. Base sends CMD packet over nRF24.
 *   2. UAV writes 0x01 on USB serial. Laptop script replies with
 *        0xA5 0x5A | len(4, LE) | crc32(4, LE) | JPEG bytes
 *   3. UAV verifies CRC, then sends the JPEG to the base in 32-byte packets:
 *        START (len, crc32, packet count) then DATA packets with sequence numbers.
 *
 * Packet layouts (all 32 bytes, fixed payload):
 *   CMD   : [0]=0x01 [1]=session
 *   START : [0]=0x10 [1]=session [2..5]=total_len [6..9]=crc32 [10..11]=num_pkts
 *   DATA  : [0]=0x11 [1]=session [2..3]=seq [4]=n_bytes [5..31]=payload (<=27)
 *   ERR   : [0]=0x20 [1]=session [2]=error code
 *
 * NOTE: debug text printed on Serial must never contain byte 0x01
 *       (it is the request byte seen by the laptop script).
 */

#include <SPI.h>
#include <RF24.h>

// ===================== USER SETTINGS =====================
#define PIN_CE      4
#define PIN_CSN     5
#define PIN_SCK     18
#define PIN_MISO    19
#define PIN_MOSI    23
#define SERIAL_BAUD 921600      // must match uav_webcam_responder.py (--baud)
#define RF_CHANNEL  76          // must match base
// =========================================================

#define MAX_IMG        60000
#define DATA_PAYLOAD   27
#define REQ_BYTE       0x01
#define CAM_TIMEOUT_MS 4000
#define APP_TRIES      10       // app-level retries on top of the radio's hardware retries

enum : uint8_t { PKT_CMD = 0x01, PKT_START = 0x10, PKT_DATA = 0x11, PKT_ERR = 0x20 };
enum : uint8_t { ERR_CAMERA = 1, ERR_TIMEOUT = 2, ERR_TOO_BIG = 3, ERR_CRC = 4 };

RF24 radio(PIN_CE, PIN_CSN);
const byte ADDR_UAV[6]  = "UAV01";   // UAV listens here
const byte ADDR_BASE[6] = "BAS01";   // UAV transmits here

static uint8_t* imgBuf = nullptr;
static uint8_t lastSession = 0;
static bool haveSession = false;

// ---------- helpers ----------
static inline void put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline void put32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline uint32_t get32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Standard CRC-32 (same as zlib.crc32 in Python)
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
  while (Serial.available()) Serial.read();   // drop stale bytes
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

// ---------- radio side ----------
bool sendPacket(const uint8_t* pkt, uint32_t& retries) {
  for (uint8_t a = 0; a < APP_TRIES; a++) {
    if (radio.write(pkt, 32)) return true;
    retries++;
    delay(2);
  }
  return false;
}

void sendError(uint8_t session, uint8_t code) {
  uint8_t pkt[32];
  memset(pkt, 0, 32);
  pkt[0] = PKT_ERR; pkt[1] = session; pkt[2] = code;
  uint32_t r = 0;
  sendPacket(pkt, r);
}

bool sendImage(uint8_t session, uint32_t len) {
  uint8_t pkt[32];
  uint32_t retries = 0;
  uint16_t numPkts = (len + DATA_PAYLOAD - 1) / DATA_PAYLOAD;
  uint32_t crc = crc32_calc(imgBuf, len);
  uint32_t t0 = millis();

  memset(pkt, 0, 32);
  pkt[0] = PKT_START; pkt[1] = session;
  put32(pkt + 2, len);
  put32(pkt + 6, crc);
  put16(pkt + 10, numPkts);
  if (!sendPacket(pkt, retries)) {
    Serial.println("[UAV] ABORT: START packet not acknowledged (base unreachable?)");
    return false;
  }

  for (uint16_t seq = 0; seq < numPkts; seq++) {
    uint32_t off = (uint32_t)seq * DATA_PAYLOAD;
    uint32_t left = len - off;
    uint8_t n = left > DATA_PAYLOAD ? DATA_PAYLOAD : (uint8_t)left;
    memset(pkt, 0, 32);
    pkt[0] = PKT_DATA; pkt[1] = session;
    put16(pkt + 2, seq);
    pkt[4] = n;
    memcpy(pkt + 5, imgBuf + off, n);
    if (!sendPacket(pkt, retries)) {
      Serial.printf("[UAV] ABORT: DATA packet %u/%u failed after %u tries\n",
                    seq, numPkts, APP_TRIES);
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

  radio.stopListening();
  if (err) {
    Serial.printf("[UAV] frame request failed, error code %u (1=camera 2=timeout 3=too big 4=crc)\n", err);
    sendError(session, err);
  } else {
    Serial.printf("[UAV] frame ready: %lu bytes (%lu ms)\n", (unsigned long)len,
                  (unsigned long)(millis() - t0));
    sendImage(session, len);
  }
  radio.startListening();
  radio.flush_rx();
}

// ---------- Arduino ----------
void setup() {
  Serial.setRxBufferSize(8192);   // must be called before begin()
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("[UAV] boot");

  imgBuf = (uint8_t*)malloc(MAX_IMG);
  if (!imgBuf) {
    while (true) { Serial.println("[UAV] FATAL: cannot allocate image buffer"); delay(1000); }
  }

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin()) {
    while (true) { Serial.println("[UAV] FATAL: nRF24 not responding - check wiring/power"); delay(1000); }
  }
  radio.setPALevel(RF24_PA_LOW);        // bench testing; raise to RF24_PA_MAX for range (needs solid 3.3V)
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(RF_CHANNEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);              // 1.5 ms delay, 15 hardware retries
  radio.disableDynamicPayloads();
  radio.setPayloadSize(32);
  radio.openWritingPipe(ADDR_BASE);
  radio.openReadingPipe(1, ADDR_UAV);
  radio.startListening();

  Serial.println("[UAV] ready, waiting for capture command");
}

void loop() {
  if (radio.available()) {
    uint8_t pkt[32];
    radio.read(pkt, 32);
    if (pkt[0] == PKT_CMD) handleCapture(pkt[1]);
  }
}
