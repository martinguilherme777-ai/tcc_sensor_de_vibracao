#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include(<esp_idf_version.h>)
#include <esp_idf_version.h>
#endif

// ============================================================
// MASTER - ESP32 + LIS3DH/HW-664
// Coleta 10 s a 2000 Hz por eixo e envia em blocos pequenos
// por ESP-NOW. Nao usa buffer completo na RAM do ESP32.
//
// Ajuste antes de carregar:
// - SDA_PIN e SCL_PIN conforme sua ligacao I2C.
// - ACCEL_RANGE_G conforme a vibracao esperada: 2, 4, 8 ou 16 g.
// - RECEIVER_MAC se quiser usar MAC fixo; broadcast funciona para teste.
// ============================================================

// ---------- I2C / LIS3DH ----------
#define SDA_PIN 23
#define SCL_PIN 18
#define LIS3DH_ADDR 0x19
#define WHO_AM_I_REG 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28

// ---------- Coleta ----------
const uint16_t SAMPLE_RATE_HZ = 2000;
const uint16_t DURATION_SECONDS = 10;
const uint32_t TOTAL_SAMPLES = (uint32_t)SAMPLE_RATE_HZ * DURATION_SECONDS;

// O LIS3DH em low-power fornece 8 bits efetivos.
// Para reduzir saturacao em regimes de maior vibracao, altere apenas
// ACCEL_RANGE_G para 8 ou 16 e use o mesmo valor no script Python
// salvar_coleta_10s.py com --escala-g.
#define ACCEL_RANGE_G 8

#if ACCEL_RANGE_G == 2
const uint8_t CTRL_REG4_VALUE = 0b00000000; // +/-2 g
const float RAW_TO_G = 0.016f;              // 16 mg/contagem
const char ACCEL_RANGE_TEXT[] = "+/-2 g";
#elif ACCEL_RANGE_G == 4
const uint8_t CTRL_REG4_VALUE = 0b00010000; // +/-4 g
const float RAW_TO_G = 0.032f;              // 32 mg/contagem
const char ACCEL_RANGE_TEXT[] = "+/-4 g";
#elif ACCEL_RANGE_G == 8
const uint8_t CTRL_REG4_VALUE = 0b00100000; // +/-8 g
const float RAW_TO_G = 0.064f;              // 64 mg/contagem
const char ACCEL_RANGE_TEXT[] = "+/-8 g";
#elif ACCEL_RANGE_G == 16
const uint8_t CTRL_REG4_VALUE = 0b00110000; // +/-16 g
const float RAW_TO_G = 0.192f;              // 192 mg/contagem
const char ACCEL_RANGE_TEXT[] = "+/-16 g";
#else
#error "ACCEL_RANGE_G deve ser 2, 4, 8 ou 16."
#endif

// ---------- ESP-NOW ----------
const uint8_t ESPNOW_CHANNEL = 1;
const uint8_t SAMPLES_PER_PACKET = 60; // Mantem o pacote abaixo de 250 bytes.
const uint8_t PACKET_REPEATS = 2;      // Reenvio simples para reduzir perdas.

// Broadcast facilita teste sem configurar MAC.
// Para uso dedicado, substitua pelo MAC do ESP32 receptor.
uint8_t RECEIVER_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t DATA_MAGIC = 0x31415845;    // "EXA1"
const uint32_t CONTROL_MAGIC = 0x31525453; // "STR1"
const uint8_t CONTROL_START = 1;

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

volatile bool startRequested = false;
volatile bool collecting = false;
uint16_t blockId = 1;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LIS3DH_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool readRawAcceleration(AccelRaw &sample) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(OUT_X_L | 0x80); // auto-incremento
  Wire.endTransmission(false);

  if (Wire.requestFrom(LIS3DH_ADDR, (uint8_t)6) != 6) {
    return false;
  }

  // Em low-power, os 8 bits uteis ficam no registrador alto.
  Wire.read();
  sample.x = (int8_t)Wire.read();
  Wire.read();
  sample.y = (int8_t)Wire.read();
  Wire.read();
  sample.z = (int8_t)Wire.read();
  return true;
}

void setupLis3dh() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t whoami = readRegister(WHO_AM_I_REG);
  if (whoami != 0x33) {
    Serial.println("erro sensor");
    while (true) delay(1000);
  }

  // CTRL_REG1:
  // ODR = 1001 e LPen = 1 -> 5.376 kHz em low-power.
  // X/Y/Z habilitados.
  writeRegister(CTRL_REG1, 0b10011111);

  // CTRL_REG4:
  // Escala selecionada em ACCEL_RANGE_G. HR permanece desligado porque
  // o modo low-power usa 8 bits e permite ODR alto.
  writeRegister(CTRL_REG4, CTRL_REG4_VALUE);
}

void handleControlPacket(const uint8_t *data, int len) {
  if (len != (int)sizeof(ControlPacket)) {
    return;
  }

  ControlPacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.magic != CONTROL_MAGIC || packet.command != CONTROL_START) {
    return;
  }

  if (packet.sampleRateHz != SAMPLE_RATE_HZ || packet.durationSeconds != DURATION_SECONDS) {
    return;
  }

  if (!collecting) {
    startRequested = true;
  }
}

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len) {
  (void)recvInfo;
  handleControlPacket(data, len);
}
#else
void onDataRecv(const uint8_t *macAddr, const uint8_t *data, int len) {
  (void)macAddr;
  handleControlPacket(data, len);
}
#endif

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("erro espnow");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("erro peer");
    while (true) delay(1000);
  }

  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) {
    Serial.println("erro callback");
    while (true) delay(1000);
  }
}

bool sendPacket(const DataPacket &packet, size_t packetSize) {
  bool queued = false;

  for (uint8_t repeat = 0; repeat < PACKET_REPEATS; repeat++) {
    if (esp_now_send(RECEIVER_MAC, (const uint8_t *)&packet, packetSize) == ESP_OK) {
      queued = true;
    }
    delayMicroseconds(250);
  }

  return queued;
}

void collectAndSendPackets() {
  Serial.println("coletando");

  const uint16_t totalPackets = (TOTAL_SAMPLES + SAMPLES_PER_PACKET - 1) / SAMPLES_PER_PACKET;
  const uint32_t intervalUs = 1000000UL / SAMPLE_RATE_HZ;
  uint16_t failedPackets = 0;
  uint32_t nextSampleAt = micros();

  for (uint16_t packetIndex = 0; packetIndex < totalPackets; packetIndex++) {
    uint32_t firstSample = (uint32_t)packetIndex * SAMPLES_PER_PACKET;
    uint32_t remaining = TOTAL_SAMPLES - firstSample;

    DataPacket packet = {};
    packet.magic = DATA_MAGIC;
    packet.blockId = blockId;
    packet.packetIndex = packetIndex;
    packet.totalPackets = totalPackets;
    packet.firstSample = firstSample;
    packet.sampleCount = remaining >= SAMPLES_PER_PACKET ? SAMPLES_PER_PACKET : remaining;
    packet.sampleRateHz = SAMPLE_RATE_HZ;
    packet.totalSamples = TOTAL_SAMPLES;
    packet.durationSeconds = DURATION_SECONDS;

    for (uint16_t i = 0; i < packet.sampleCount; i++) {
      AccelRaw sample = {0, 0, 0};
      readRawAcceleration(sample);
      packet.samples[i] = sample;

      nextSampleAt += intervalUs;
      while ((int32_t)(micros() - nextSampleAt) < 0) {
      }
    }

    size_t packetSize = sizeof(DataPacket) - sizeof(packet.samples) + (packet.sampleCount * sizeof(AccelRaw));
    if (!sendPacket(packet, packetSize)) {
      failedPackets++;
    }

    delay(0);
  }

  Serial.println(failedPackets == 0 ? "buffer enviado" : "erro no envio");
  blockId++;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupLis3dh();
  setupEspNow();
  Serial.print("escala ");
  Serial.println(ACCEL_RANGE_TEXT);
  Serial.println("aguardando");
}

void loop() {
  if (startRequested) {
    startRequested = false;
    collecting = true;
    collectAndSendPackets();
    collecting = false;
  }

  delay(5);
}
