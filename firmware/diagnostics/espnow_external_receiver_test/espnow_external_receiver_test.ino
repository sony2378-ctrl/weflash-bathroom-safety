#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// 다음 단계의 휴대폰 AP도 이 채널로 고정한다.
constexpr uint8_t WIFI_CHANNEL = 6;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t LINK_TIMEOUT_MS = 3000;

constexpr uint32_t PACKET_MAGIC = 0x57464C53;  // "WFLS"
constexpr uint8_t PACKET_VERSION = 1;
constexpr uint8_t PACKET_TEST = 1;
constexpr uint8_t PACKET_ACK = 2;

struct __attribute__((packed)) NowTestPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t simulatedFall;
  uint8_t reserved;
  uint32_t sequence;
  uint32_t uptimeMs;
};

struct ReceivedEvent {
  uint8_t sourceMac[6];
  NowTestPacket packet;
  int8_t rssi;
};

static_assert(sizeof(NowTestPacket) == 16,
              "ESP-NOW packet layout changed");

QueueHandle_t receiveQueue = nullptr;

uint32_t lastPacketMs = 0;
uint32_t lastSequence = 0;
uint32_t receivedCount = 0;
uint32_t missingCount = 0;
bool linkWasUp = false;

void printMac(const uint8_t *mac) {
  Serial.printf(
      "%02X:%02X:%02X:%02X:%02X:%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
  );
}

void queueReceivedPacket(
    const uint8_t *sourceMac,
    const uint8_t *data,
    int length,
    int8_t rssi
) {
  if (receiveQueue == nullptr ||
      sourceMac == nullptr ||
      data == nullptr ||
      length != static_cast<int>(sizeof(NowTestPacket))) {
    return;
  }

  ReceivedEvent event = {};
  memcpy(event.sourceMac, sourceMac, sizeof(event.sourceMac));
  memcpy(&event.packet, data, sizeof(event.packet));
  event.rssi = rssi;
  xQueueSend(receiveQueue, &event, 0);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int length
) {
  const int8_t rssi =
      (info != nullptr && info->rx_ctrl != nullptr)
          ? info->rx_ctrl->rssi
          : -127;

  queueReceivedPacket(
      info != nullptr ? info->src_addr : nullptr,
      data,
      length,
      rssi
  );
}
#else
void onDataReceived(
    const uint8_t *sourceMac,
    const uint8_t *data,
    int length
) {
  queueReceivedPacket(sourceMac, data, length, -127);
}
#endif

bool setFixedChannel() {
  const esp_err_t result = esp_wifi_set_channel(
      WIFI_CHANNEL,
      WIFI_SECOND_CHAN_NONE
  );

  if (result != ESP_OK) {
    Serial.printf("[FAIL] Wi-Fi channel error: %d\n", result);
    return false;
  }

  return true;
}

bool ensureSenderPeer(const uint8_t *senderMac) {
  if (esp_now_is_peer_exist(senderMac)) {
    return true;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, senderMac, sizeof(peer.peer_addr));
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  const esp_err_t result = esp_now_add_peer(&peer);

  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    Serial.printf("[ACK FAIL] Sender peer error: %d\n", result);
    return false;
  }

  return true;
}

void sendAck(const ReceivedEvent &event) {
  if (!ensureSenderPeer(event.sourceMac)) {
    return;
  }

  NowTestPacket ack = {};
  ack.magic = PACKET_MAGIC;
  ack.version = PACKET_VERSION;
  ack.type = PACKET_ACK;
  ack.simulatedFall = event.packet.simulatedFall;
  ack.sequence = event.packet.sequence;
  ack.uptimeMs = millis();

  const esp_err_t result = esp_now_send(
      event.sourceMac,
      reinterpret_cast<const uint8_t *>(&ack),
      sizeof(ack)
  );

  Serial.printf(
      "[ACK] seq=%lu %s\n",
      static_cast<unsigned long>(ack.sequence),
      result == ESP_OK ? "queued" : "FAILED"
  );
}

void processReceivedPackets() {
  ReceivedEvent event = {};

  while (xQueueReceive(receiveQueue, &event, 0) == pdTRUE) {
    const NowTestPacket &packet = event.packet;

    if (packet.magic != PACKET_MAGIC ||
        packet.version != PACKET_VERSION ||
        packet.type != PACKET_TEST) {
      Serial.println("[RX] Invalid or unrelated packet ignored");
      continue;
    }

    if (lastSequence != 0 && packet.sequence > lastSequence + 1) {
      missingCount += packet.sequence - lastSequence - 1;
    }

    lastSequence = packet.sequence;
    lastPacketMs = millis();
    receivedCount++;

    Serial.print("[RX] from=");
    printMac(event.sourceMac);
    Serial.printf(
        " seq=%lu state=%s rssi=%d\n",
        static_cast<unsigned long>(packet.sequence),
        packet.simulatedFall ? "TEST_FALL" : "NORMAL",
        event.rssi
    );

    sendAck(event);
  }
}

void updateLinkState() {
  const bool linkUp =
      lastPacketMs != 0 && millis() - lastPacketMs < LINK_TIMEOUT_MS;

  if (linkUp != linkWasUp) {
    linkWasUp = linkUp;
    Serial.println(linkUp ? "[LINK] CEILING ONLINE"
                          : "[LINK] CEILING OFFLINE");
  }
}

void handleSerialCommand() {
  while (Serial.available()) {
    const char command = Serial.read();

    if (command == 's' || command == 'S') {
      Serial.printf(
          "[STATUS] received=%lu missing=%lu link=%s\n",
          static_cast<unsigned long>(receivedCount),
          static_cast<unsigned long>(missingCount),
          (lastPacketMs != 0 &&
           millis() - lastPacketMs < LINK_TIMEOUT_MS)
              ? "ONLINE"
              : "OFFLINE"
      );
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("=== ESP-NOW EXTERNAL RECEIVER TEST ===");
  Serial.println("Command: S=status");

  receiveQueue = xQueueCreate(12, sizeof(ReceivedEvent));

  if (receiveQueue == nullptr) {
    Serial.println("[FATAL] Queue creation failed");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  delay(100);

  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!setFixedChannel()) {
    while (true) delay(1000);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[FATAL] ESP-NOW initialization failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataReceived);

  Serial.printf("Wi-Fi channel: %u\n", WIFI_CHANNEL);
  Serial.println("[READY] Waiting for ceiling packets");
}

void loop() {
  processReceivedPackets();
  updateLinkState();
  handleSerialCommand();
  delay(2);
}
