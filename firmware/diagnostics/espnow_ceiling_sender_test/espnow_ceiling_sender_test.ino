#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// 휴대폰 AP 기능을 붙일 때도 같은 채널을 사용한다.
constexpr uint8_t WIFI_CHANNEL = 6;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SEND_INTERVAL_MS = 1000;
constexpr uint32_t LINK_TIMEOUT_MS = 3000;

constexpr uint32_t PACKET_MAGIC = 0x57464C53;  // "WFLS"
constexpr uint8_t PACKET_VERSION = 1;
constexpr uint8_t PACKET_TEST = 1;
constexpr uint8_t PACKET_ACK = 2;

const uint8_t BROADCAST_MAC[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

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

uint32_t nextSequence = 1;
uint32_t lastSendMs = 0;
uint32_t lastAckMs = 0;
uint32_t lastSentSequence = 0;
uint32_t lastSentAtMs = 0;
uint32_t sentCount = 0;
uint32_t ackCount = 0;
uint8_t simulatedFall = 0;
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

bool addBroadcastPeer() {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  const esp_err_t result = esp_now_add_peer(&peer);

  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    Serial.printf("[FAIL] Broadcast peer error: %d\n", result);
    return false;
  }

  return true;
}

void sendTestPacket() {
  NowTestPacket packet = {};
  packet.magic = PACKET_MAGIC;
  packet.version = PACKET_VERSION;
  packet.type = PACKET_TEST;
  packet.simulatedFall = simulatedFall;
  packet.sequence = nextSequence++;
  packet.uptimeMs = millis();

  const esp_err_t result = esp_now_send(
      BROADCAST_MAC,
      reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet)
  );

  if (result == ESP_OK) {
    lastSentSequence = packet.sequence;
    lastSentAtMs = millis();
    sentCount++;

    Serial.printf(
        "[TX] seq=%lu state=%s queued\n",
        static_cast<unsigned long>(packet.sequence),
        simulatedFall ? "TEST_FALL" : "NORMAL"
    );
  } else {
    Serial.printf(
        "[TX FAIL] seq=%lu error=%d\n",
        static_cast<unsigned long>(packet.sequence),
        result
    );
  }
}

void processReceivedPackets() {
  ReceivedEvent event = {};

  while (xQueueReceive(receiveQueue, &event, 0) == pdTRUE) {
    const NowTestPacket &packet = event.packet;

    if (packet.magic != PACKET_MAGIC ||
        packet.version != PACKET_VERSION ||
        packet.type != PACKET_ACK) {
      Serial.println("[RX] Invalid or unrelated packet ignored");
      continue;
    }

    lastAckMs = millis();
    ackCount++;

    Serial.print("[ACK] from=");
    printMac(event.sourceMac);
    Serial.printf(
        " seq=%lu rssi=%d",
        static_cast<unsigned long>(packet.sequence),
        event.rssi
    );

    if (packet.sequence == lastSentSequence) {
      Serial.printf(
          " rtt=%lu ms",
          static_cast<unsigned long>(millis() - lastSentAtMs)
      );
    }

    Serial.println();
  }
}

void handleSerialCommand() {
  while (Serial.available()) {
    const char command = Serial.read();

    if (command == '1') {
      simulatedFall = 1;
      Serial.println("[MODE] Simulated fall ON");
    } else if (command == '0') {
      simulatedFall = 0;
      Serial.println("[MODE] Normal state");
    } else if (command == 's' || command == 'S') {
      Serial.printf(
          "[STATUS] sent=%lu ack=%lu link=%s\n",
          static_cast<unsigned long>(sentCount),
          static_cast<unsigned long>(ackCount),
          (lastAckMs != 0 && millis() - lastAckMs < LINK_TIMEOUT_MS)
              ? "OK"
              : "WAIT/LOST"
      );
    }
  }
}

void updateLinkState() {
  const bool linkUp =
      lastAckMs != 0 && millis() - lastAckMs < LINK_TIMEOUT_MS;

  if (linkUp != linkWasUp) {
    linkWasUp = linkUp;
    Serial.println(linkUp ? "[LINK] OK" : "[LINK] LOST");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("=== ESP-NOW CEILING SENDER TEST ===");
  Serial.println("Commands: 1=simulated fall, 0=normal, S=status");

  receiveQueue = xQueueCreate(8, sizeof(ReceivedEvent));

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

  if (!addBroadcastPeer()) {
    while (true) delay(1000);
  }

  Serial.printf("Wi-Fi channel: %u\n", WIFI_CHANNEL);
  Serial.println("[READY] Start the external receiver board");
}

void loop() {
  processReceivedPackets();
  handleSerialCommand();
  updateLinkState();

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    sendTestPacket();
  }

  delay(2);
}
