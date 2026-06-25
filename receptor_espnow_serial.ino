#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include(<esp_idf_version.h>)
#include <esp_idf_version.h>
#endif

// ============================================================
// RECEPTOR - ESP32
// Recebe pacotes binarios via ESP-NOW e repassa os mesmos
// pacotes ao PC em frames binarios pela Serial.
//
// O Python reorganiza os pacotes e salva CSV/XLSX. Isso evita
// imprimir 20.000 linhas no ESP32 durante a recepcao, reduzindo
// bastante perda de pacotes.
//
// Ajuste antes de carregar:
// - SERIAL_BAUD se seu cabo/placa nao sustentar 2.000.000 baud.
// - MASTER_MAC se quiser usar MAC fixo; broadcast funciona para teste.
// ============================================================

const uint32_t SERIAL_BAUD = 2000000;
const uint8_t ESPNOW_CHANNEL = 1;
const uint16_t SAMPLE_RATE_HZ = 2000;
const uint16_t DURATION_SECONDS = 10;
const uint32_t TOTAL_SAMPLES = (uint32_t)SAMPLE_RATE_HZ * DURATION_SECONDS;

const uint8_t SAMPLES_PER_PACKET = 60;
const uint16_t TOTAL_PACKETS = (TOTAL_SAMPLES + SAMPLES_PER_PACKET - 1) / SAMPLES_PER_PACKET;
const uint16_t MAX_PACKETS_PER_BLOCK = 800;
const uint8_t QUEUE_SIZE = 96;
const uint32_t START_REPEAT_MS = 3000;
const uint32_t START_INTERVAL_MS = 100;

// Broadcast facilita o primeiro teste. Para uso dedicado,
// substitua pelo MAC do ESP32 master.
uint8_t MASTER_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t DATA_MAGIC = 0x31415845;    // "EXA1"
const uint32_t CONTROL_MAGIC = 0x31525453; // "STR1"
const uint8_t CONTROL_START = 1;
const uint16_t SERIAL_FRAME_MAGIC = 0x5AA5;
const char RECEPTOR_ID[] = "ESP_RECEPTOR";

struct __attribute__((packed)) AccelRaw {
  int8_t x;
  int8_t y;
  int8_t z;
};

struct __attribute__((packed)) DataPacket {
  uint32_t magic;
  uint16_t blockId;
  uint16_t packetIndex;
  uint16_t totalPackets;
  uint32_t firstSample;
  uint16_t sampleCount;
  uint16_t sampleRateHz;
  uint32_t totalSamples;
  uint16_t durationSeconds;
  AccelRaw samples[SAMPLES_PER_PACKET];
};

struct __attribute__((packed)) ControlPacket {
  uint32_t magic;
  uint8_t command;
  uint16_t durationSeconds;
  uint16_t sampleRateHz;
};

struct __attribute__((packed)) SerialFrameHeader {
  uint16_t magic;
  uint16_t length;
};

const size_t PACKET_HEADER_SIZE = sizeof(DataPacket) - (SAMPLES_PER_PACKET * sizeof(AccelRaw));

struct PacketQueueItem {
  DataPacket packet;
  size_t len;
};

bool packetSeen[MAX_PACKETS_PER_BLOCK];
PacketQueueItem packetQueue[QUEUE_SIZE];

uint16_t activeBlockId = 0;
uint16_t receivedPackets = 0;
bool repeatingStart = false;
uint32_t repeatStartUntil = 0;
uint32_t lastStartMillis = 0;
volatile uint8_t queueHead = 0;
volatile uint8_t queueTail = 0;
volatile uint32_t droppedQueuePackets = 0;
portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;

String serialLine;

uint16_t checksum16(const uint8_t *data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return (uint16_t)(sum & 0xFFFF);
}

bool pushPacketFromCallback(const uint8_t *data, int len) {
  if (len < (int)PACKET_HEADER_SIZE || len > (int)sizeof(DataPacket)) {
    return false;
  }

  uint8_t nextHead = (queueHead + 1) % QUEUE_SIZE;
  if (nextHead == queueTail) {
    droppedQueuePackets++;
    return false;
  }

  memset(&packetQueue[queueHead].packet, 0, sizeof(DataPacket));
  memcpy(&packetQueue[queueHead].packet, data, len);
  packetQueue[queueHead].len = (size_t)len;
  queueHead = nextHead;
  return true;
}

bool popPacket(DataPacket &packet, size_t &len) {
  bool hasPacket = false;

  portENTER_CRITICAL(&queueMux);
  if (queueTail != queueHead) {
    packet = packetQueue[queueTail].packet;
    len = packetQueue[queueTail].len;
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    hasPacket = true;
  }
  portEXIT_CRITICAL(&queueMux);

  return hasPacket;
}

void clearQueue() {
  portENTER_CRITICAL(&queueMux);
  queueHead = 0;
  queueTail = 0;
  droppedQueuePackets = 0;
  portEXIT_CRITICAL(&queueMux);
}

void resetReceptionState() {
  clearQueue();
  memset(packetSeen, 0, sizeof(packetSeen));
  activeBlockId = 0;
  receivedPackets = 0;
}

void resetBlock(uint16_t blockId) {
  memset(packetSeen, 0, sizeof(packetSeen));
  activeBlockId = blockId;
  receivedPackets = 0;
}

bool sendStartPacket() {
  ControlPacket packet = {};
  packet.magic = CONTROL_MAGIC;
  packet.command = CONTROL_START;
  packet.durationSeconds = DURATION_SECONDS;
  packet.sampleRateHz = SAMPLE_RATE_HZ;

  return esp_now_send(MASTER_MAC, (const uint8_t *)&packet, sizeof(packet)) == ESP_OK;
}

void sendStartToMaster() {
  resetReceptionState();
  repeatingStart = true;
  repeatStartUntil = millis() + START_REPEAT_MS;
  lastStartMillis = 0;

  Serial.println("aguardando dados");
}

void handleSerialCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command == "ID?" || command == "ID") {
    Serial.println(RECEPTOR_ID);
  } else if (command == "START") {
    sendStartToMaster();
  }
}

void processSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        handleSerialCommand(serialLine);
        serialLine = "";
      }
    } else if (serialLine.length() < 32) {
      serialLine += c;
    }
  }
}

bool packetLooksValid(const DataPacket &packet, size_t len) {
  if (packet.magic != DATA_MAGIC) {
    return false;
  }

  if (packet.sampleRateHz != SAMPLE_RATE_HZ ||
      packet.durationSeconds != DURATION_SECONDS ||
      packet.totalSamples != TOTAL_SAMPLES ||
      packet.totalPackets != TOTAL_PACKETS ||
      packet.totalPackets > MAX_PACKETS_PER_BLOCK ||
      packet.packetIndex >= packet.totalPackets ||
      packet.sampleCount == 0 ||
      packet.sampleCount > SAMPLES_PER_PACKET ||
      packet.firstSample + packet.sampleCount > TOTAL_SAMPLES) {
    return false;
  }

  size_t expectedLen = PACKET_HEADER_SIZE + ((size_t)packet.sampleCount * sizeof(AccelRaw));
  return len == expectedLen;
}

void forwardPacketToPc(const DataPacket &packet, size_t len) {
  SerialFrameHeader header = {};
  header.magic = SERIAL_FRAME_MAGIC;
  header.length = (uint16_t)len;
  uint16_t checksum = checksum16((const uint8_t *)&packet, len);

  Serial.write((const uint8_t *)&header, sizeof(header));
  Serial.write((const uint8_t *)&packet, len);
  Serial.write((const uint8_t *)&checksum, sizeof(checksum));
}

void processDataPacket(const DataPacket &packet, size_t len) {
  if (!packetLooksValid(packet, len)) {
    return;
  }

  repeatingStart = false;

  if (packet.blockId != activeBlockId) {
    resetBlock(packet.blockId);
  }

  if (packetSeen[packet.packetIndex]) {
    return;
  }

  packetSeen[packet.packetIndex] = true;
  receivedPackets++;
  forwardPacketToPc(packet, len);

  if (receivedPackets >= packet.totalPackets) {
    Serial.println("buffer recebido");
  }
}

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len) {
  (void)recvInfo;
  portENTER_CRITICAL_ISR(&queueMux);
  pushPacketFromCallback(data, len);
  portEXIT_CRITICAL_ISR(&queueMux);
}
#else
void onDataRecv(const uint8_t *macAddr, const uint8_t *data, int len) {
  (void)macAddr;
  portENTER_CRITICAL_ISR(&queueMux);
  pushPacketFromCallback(data, len);
  portEXIT_CRITICAL_ISR(&queueMux);
}
#endif

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("erro de pacote");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MASTER_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("erro de pacote");
    while (true) delay(1000);
  }

  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) {
    Serial.println("erro de pacote");
    while (true) delay(1000);
  }
}

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  resetReceptionState();
  setupEspNow();
  Serial.println("aguardando dados");
}

void loop() {
  processSerialInput();

  if (repeatingStart) {
    uint32_t now = millis();
    if (now - lastStartMillis >= START_INTERVAL_MS) {
      sendStartPacket();
      lastStartMillis = now;
    }
    if ((int32_t)(now - repeatStartUntil) >= 0) {
      repeatingStart = false;
    }
  }

  DataPacket packet;
  size_t len = 0;
  while (popPacket(packet, len)) {
    processDataPacket(packet, len);
  }

  delay(1);
}
