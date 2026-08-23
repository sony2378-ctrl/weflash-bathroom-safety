#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <mbedtls/md.h>
#include <stddef.h>
#include "project_config.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

// 1: 발표 시연(장시간 무반응 15초), 0: 일반 운용 시험(300초)
#define CONTEST_DEMO_MODE 1

namespace Config {
constexpr char FIRMWARE_VERSION[] = "3.5.0";

constexpr uint8_t DOOR_PIN = 4;
constexpr uint8_t SOS_PIN = 5;
constexpr uint8_t RELAY_PIN = 17;
constexpr uint8_t RELAY_ACTIVE_LEVEL = HIGH;

constexpr uint8_t WIFI_CHANNEL = 6;
constexpr char AP_SSID[] = "WEFLASH-ALARM";
constexpr char AP_PASSWORD[] = WEFLASH_AP_PASSWORD;

// project_config.h의 값은 천장부와 반드시 같아야 합니다.
constexpr uint8_t PACKET_AUTH_KEY[32] = WEFLASH_PACKET_AUTH_KEY;

constexpr uint32_t INPUT_DEBOUNCE_MS = 40;
constexpr uint32_t COMM_TIMEOUT_MS = 5000;
constexpr uint32_t STARTUP_GRACE_MS = 15000;
constexpr uint32_t OCCUPANCY_CONFIRM_MS = 1000;
constexpr uint32_t PRESENCE_LOST_GRACE_MS = 3000;
constexpr uint32_t FALL_CONFIRM_MS = 3000;
constexpr uint32_t LOW_POSITION_CONFIRM_MS = 5000;
#if CONTEST_DEMO_MODE
constexpr uint32_t NO_RESPONSE_ALARM_MS = 15000;
#else
constexpr uint32_t NO_RESPONSE_ALARM_MS = 300000;
#endif
constexpr uint32_t STATUS_PRINT_MS = 2000;
constexpr uint32_t SENDER_RESTART_GAP_MS = 1500;
}  // namespace Config

static_assert(CONTEST_DEMO_MODE == 0 || CONTEST_DEMO_MODE == 1,
              "CONTEST_DEMO_MODE must be 0 or 1");
static_assert(Config::DOOR_PIN != Config::SOS_PIN &&
                  Config::DOOR_PIN != Config::RELAY_PIN &&
                  Config::SOS_PIN != Config::RELAY_PIN,
              "GPIO assignments must be unique");
static_assert(Config::RELAY_ACTIVE_LEVEL == HIGH ||
                  Config::RELAY_ACTIVE_LEVEL == LOW,
              "Relay active level must be HIGH or LOW");
static_assert(sizeof(Config::AP_PASSWORD) - 1 >= 8 &&
                  sizeof(Config::AP_PASSWORD) - 1 <= 63,
              "SoftAP password must contain 8 to 63 characters");

constexpr uint16_t PACKET_MAGIC = 0xBADA;
constexpr uint8_t PACKET_VERSION = 2;

enum PacketFlag : uint8_t {
  FLAG_LD_OK = 1U << 0,
  FLAG_C1001_OK = 1U << 1,
  FLAG_LD_PRESENCE = 1U << 2,
  FLAG_DERIVED_MOVEMENT = 1U << 3,
  FLAG_C1001_FALL = 1U << 4,
  FLAG_C1001_STILL = 1U << 5,
  FLAG_LOW_POSITION = 1U << 6
};

enum PacketTestMode : uint8_t {
  TEST_AUTO = 0,
  TEST_FORCE_ALARM = 1,
  TEST_FORCE_NORMAL = 2
};

struct __attribute__((packed)) SensorPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t flags;
  uint32_t sequence;
  uint32_t uptimeMs;
  uint32_t noMovementMs;
  int16_t cMovementRange;
  uint16_t ldMovingDistance;
  uint16_t ldStaticDistance;
  uint16_t ldDetectionDistance;
  int8_t cPresence;
  int8_t cMovement;
  int8_t cFall;
  int8_t cStill;
  uint8_t ldState;
  uint8_t ldMovingEnergy;
  uint8_t ldStaticEnergy;
  uint8_t testMode;
  uint64_t authTag;
};

static_assert(offsetof(SensorPacket, authTag) == 32,
              "SensorPacket authenticated region must be 32 bytes");
static_assert(sizeof(SensorPacket) == 40,
              "SensorPacket total size must be 40 bytes");

enum class SystemState : uint8_t {
  STANDBY,
  OCCUPANCY_CONFIRM,
  MONITORING,
  FALL_CONFIRM,
  LOW_POSITION_CONFIRM,
  ALARM_LATCHED,
  ALARM_RELEASED,
  FORCED_NORMAL,
  COMM_ERROR,
  SENSOR_ERROR
};

enum class AlarmCause : uint8_t {
  NONE,
  SOS,
  WEB_TEST,
  CEILING_TEST,
  FALL_NO_RESPONSE,
  LOW_POSITION,
  LONG_NO_RESPONSE,
  PRESENCE_LOST_CLOSED_DOOR
};

class DebouncedInput {
 public:
  DebouncedInput(uint8_t pin, uint8_t activeLevel)
      : pin_(pin), activeLevel_(activeLevel) {}

  void begin() {
    rawLevel_ = digitalRead(pin_);
    stableLevel_ = rawLevel_;
    changedAtMs_ = millis();
  }

  bool update(uint32_t now) {
    const uint8_t level = digitalRead(pin_);
    if (level != rawLevel_) {
      rawLevel_ = level;
      changedAtMs_ = now;
    }
    if (stableLevel_ != rawLevel_ &&
        now - changedAtMs_ >= Config::INPUT_DEBOUNCE_MS) {
      stableLevel_ = rawLevel_;
    }
    return stableLevel_ == activeLevel_;
  }

 private:
  uint8_t pin_;
  uint8_t activeLevel_;
  uint8_t rawLevel_ = HIGH;
  uint8_t stableLevel_ = HIGH;
  uint32_t changedAtMs_ = 0;
};

WebServer server(80);
DebouncedInput doorOpenInput(Config::DOOR_PIN, HIGH);
DebouncedInput sosPressedInput(Config::SOS_PIN, LOW);

portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
SensorPacket callbackPacket = {};
uint8_t callbackSenderMac[6] = {};
volatile bool packetPending = false;
volatile uint32_t callbackPacketMs = 0;
volatile uint32_t rejectedLengthCount = 0;

SensorPacket sensorPacket = {};
uint8_t lockedSenderMac[6] = {};
bool senderLocked = false;
bool sequenceInitialized = false;
uint32_t lastAcceptedSequence = 0;
uint32_t lastSenderUptimeMs = 0;
uint32_t lastGoodPacketMs = 0;
uint32_t acceptedPacketCount = 0;
uint32_t rejectedHeaderCount = 0;
uint32_t rejectedAuthCount = 0;
uint32_t rejectedSenderCount = 0;
uint32_t rejectedReplayCount = 0;

SystemState currentState = SystemState::COMM_ERROR;
AlarmCause alarmCause = AlarmCause::NONE;
const char* alarmReason = "없음";
const char* faultReason = "천장부 연결 대기";

bool isDoorOpen = true;
bool isSosPressed = false;
bool lastSosPressed = false;
bool alarmLatched = false;
bool relayOutputOn = false;
bool webReleaseRequested = false;
bool webTestRequested = false;

bool ignoreFallUntilClear = false;
bool ignoreLowUntilClear = false;
bool ignoreForcedTestUntilClear = false;
bool ignoreNoResponseUntilMovement = false;
bool ignorePresenceLossUntilClear = false;

uint32_t bootStartedMs = 0;
uint32_t occupancyConfirmStartedMs = 0;
uint32_t monitoringBaselineMs = 0;
uint32_t presenceLostStartedMs = 0;
uint32_t fallConfirmStartedMs = 0;
uint32_t lowPositionStartedMs = 0;
uint32_t alarmStartedMs = 0;
uint32_t releasedMs = 0;
uint32_t lastStatusPrintMs = 0;
bool doorOpenedDuringPresenceLoss = false;

bool flagIsSet(uint8_t flag) {
  return (sensorPacket.flags & flag) != 0;
}

bool isLdOk() { return flagIsSet(FLAG_LD_OK); }
bool isC1001Ok() { return flagIsSet(FLAG_C1001_OK); }
bool isMoving() { return flagIsSet(FLAG_DERIVED_MOVEMENT); }
bool isFallSuspected() { return flagIsSet(FLAG_C1001_FALL); }
bool isStill() { return flagIsSet(FLAG_C1001_STILL); }
bool isLowPositionCandidate() { return flagIsSet(FLAG_LOW_POSITION); }

bool isPresent() {
  if (isLdOk()) return flagIsSet(FLAG_LD_PRESENCE);
  return isC1001Ok() && sensorPacket.cPresence > 0;
}

const char* stateName(SystemState state) {
  switch (state) {
    case SystemState::STANDBY: return "대기";
    case SystemState::OCCUPANCY_CONFIRM: return "재실 확인";
    case SystemState::MONITORING: return "정상 감시";
    case SystemState::FALL_CONFIRM: return "낙상 의심 확인";
    case SystemState::LOW_POSITION_CONFIRM: return "낮은 자세 확인";
    case SystemState::ALARM_LATCHED: return "경보 유지";
    case SystemState::ALARM_RELEASED: return "경보 해제";
    case SystemState::FORCED_NORMAL: return "강제 정상 시험";
    case SystemState::COMM_ERROR: return "천장부 통신 오류";
    case SystemState::SENSOR_ERROR: return "센서 오류";
  }
  return "알 수 없음";
}

String formatMac(const uint8_t* mac) {
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}

const char* testModeName(uint8_t mode) {
  switch (mode) {
    case TEST_AUTO: return "자동";
    case TEST_FORCE_ALARM: return "강제 경보";
    case TEST_FORCE_NORMAL: return "강제 정상";
    default: return "잘못된 값";
  }
}

const char* ldStateName(uint8_t state) {
  switch (state) {
    case 0: return "사람 없음";
    case 1: return "움직임";
    case 2: return "정지";
    case 3: return "움직임+정지";
    default: return "알 수 없음";
  }
}

bool computePacketAuthTag(const SensorPacket& source, uint8_t output[8]) {
  const mbedtls_md_info_t* info =
      mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (info == nullptr) return false;

  uint8_t digest[32] = {};
  const int result = mbedtls_md_hmac(
      info, Config::PACKET_AUTH_KEY, sizeof(Config::PACKET_AUTH_KEY),
      reinterpret_cast<const uint8_t*>(&source),
      offsetof(SensorPacket, authTag), digest);
  if (result != 0) return false;

  memcpy(output, digest, 8);
  return true;
}

bool constantTimeEqual8(const uint8_t left[8], const uint8_t right[8]) {
  uint8_t difference = 0;
  for (size_t i = 0; i < 8; ++i) difference |= left[i] ^ right[i];
  return difference == 0;
}

void storeIncomingPacket(const uint8_t* senderMac,
                         const uint8_t* data, int length) {
  if (senderMac == nullptr || data == nullptr ||
      length != static_cast<int>(sizeof(SensorPacket))) {
    ++rejectedLengthCount;
    return;
  }

  portENTER_CRITICAL(&packetMux);
  memcpy(&callbackPacket, data, sizeof(callbackPacket));
  memcpy(callbackSenderMac, senderMac, 6);
  callbackPacketMs = millis();
  packetPending = true;
  portEXIT_CRITICAL(&packetMux);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowReceive(const esp_now_recv_info_t* info,
                     const uint8_t* data, int length) {
  storeIncomingPacket(info == nullptr ? nullptr : info->src_addr,
                      data, length);
}
#else
void onEspNowReceive(const uint8_t* senderMac,
                     const uint8_t* data, int length) {
  storeIncomingPacket(senderMac, data, length);
}
#endif

void processReceivedPacket() {
  SensorPacket incoming = {};
  uint8_t senderMac[6] = {};
  uint32_t receivedAtMs = 0;
  bool available = false;

  portENTER_CRITICAL(&packetMux);
  if (packetPending) {
    incoming = callbackPacket;
    memcpy(senderMac, callbackSenderMac, 6);
    receivedAtMs = callbackPacketMs;
    packetPending = false;
    available = true;
  }
  portEXIT_CRITICAL(&packetMux);

  if (!available) return;

  if (incoming.magic != PACKET_MAGIC ||
      incoming.version != PACKET_VERSION ||
      incoming.testMode > TEST_FORCE_NORMAL) {
    ++rejectedHeaderCount;
    return;
  }

  uint8_t expectedTag[8] = {};
  uint8_t receivedTag[8] = {};
  memcpy(receivedTag, &incoming.authTag, sizeof(receivedTag));
  if (!computePacketAuthTag(incoming, expectedTag) ||
      !constantTimeEqual8(expectedTag, receivedTag)) {
    ++rejectedAuthCount;
    return;
  }

  // 별도 페어링 절차 없이 최초의 정상 HMAC 송신자에 RAM에서 잠급니다.
  // 다른 천장부를 사용하려면 외부 제어부를 재부팅하면 됩니다.
  if (!senderLocked) {
    memcpy(lockedSenderMac, senderMac, 6);
    senderLocked = true;
    sequenceInitialized = false;
    Serial.print("[보안] 정상 HMAC 천장부 자동 등록: ");
    Serial.println(formatMac(lockedSenderMac));
  } else if (memcmp(lockedSenderMac, senderMac, 6) != 0) {
    ++rejectedSenderCount;
    return;
  }

  const bool forwardSequence =
      !sequenceInitialized ||
      static_cast<int32_t>(incoming.sequence - lastAcceptedSequence) > 0;
  const bool senderRestarted =
      sequenceInitialized &&
      incoming.uptimeMs + Config::SENDER_RESTART_GAP_MS < lastSenderUptimeMs &&
      receivedAtMs - lastGoodPacketMs >= Config::SENDER_RESTART_GAP_MS;

  if (!forwardSequence && !senderRestarted) {
    ++rejectedReplayCount;
    return;
  }

  if (senderRestarted) {
    Serial.println("[ESP-NOW] 천장부 재부팅 감지 - 순번 기준 재설정");
  }

  sensorPacket = incoming;
  lastAcceptedSequence = incoming.sequence;
  lastSenderUptimeMs = incoming.uptimeMs;
  lastGoodPacketMs = receivedAtMs;
  sequenceInitialized = true;
  ++acceptedPacketCount;
}

bool communicationOk(uint32_t now) {
  return lastGoodPacketMs != 0 &&
         now - lastGoodPacketMs <= Config::COMM_TIMEOUT_MS;
}

void readHardwareInputs() {
  const uint32_t now = millis();
  // 문센서: 닫힘에서 GPIO-GND 접점 연결(LOW), 열림에서 HIGH
  isDoorOpen = doorOpenInput.update(now);
  // SOS: 누르면 GPIO-GND 접점 연결(LOW)
  isSosPressed = sosPressedInput.update(now);
}

void setRelayOutput(bool enabled) {
  if (relayOutputOn == enabled) return;
  relayOutputOn = enabled;
  const uint8_t inactive =
      Config::RELAY_ACTIVE_LEVEL == HIGH ? LOW : HIGH;
  digitalWrite(Config::RELAY_PIN,
               enabled ? Config::RELAY_ACTIVE_LEVEL : inactive);
  Serial.printf("[릴레이] %s\n", enabled ? "ON" : "OFF");
}

void enterStandby(uint32_t now) {
  currentState = SystemState::STANDBY;
  occupancyConfirmStartedMs = 0;
  monitoringBaselineMs = now;
  presenceLostStartedMs = 0;
  fallConfirmStartedMs = 0;
  lowPositionStartedMs = 0;
  doorOpenedDuringPresenceLoss = false;
}

uint32_t effectiveNoMovementMs(uint32_t now) {
  if (!isPresent()) return 0;
  const uint32_t locallyObserved = now - monitoringBaselineMs;
  return sensorPacket.noMovementMs < locallyObserved
             ? sensorPacket.noMovementMs
             : locallyObserved;
}

void startAlarm(AlarmCause cause, const char* reason, uint32_t now) {
  if (alarmLatched) return;
  alarmCause = cause;
  alarmReason = reason;
  alarmStartedMs = now;
  alarmLatched = true;
  currentState = SystemState::ALARM_LATCHED;
  Serial.print("[경보 발생] ");
  Serial.println(alarmReason);
}

void releaseAlarm(uint32_t now) {
  ignoreFallUntilClear = isFallSuspected();
  ignoreLowUntilClear = isLowPositionCandidate();
  ignoreForcedTestUntilClear = sensorPacket.testMode == TEST_FORCE_ALARM;
  ignoreNoResponseUntilMovement =
      isPresent() && effectiveNoMovementMs(now) >= Config::NO_RESPONSE_ALARM_MS;
  ignorePresenceLossUntilClear = !isPresent() && !isDoorOpen;

  alarmLatched = false;
  alarmCause = AlarmCause::NONE;
  alarmReason = "없음";
  webReleaseRequested = false;
  releasedMs = now;
  currentState = SystemState::ALARM_RELEASED;
  Serial.println("[경보 해제] 휴대폰 요청 처리 완료");
}

void clearAcknowledgedConditions() {
  if (!isFallSuspected()) ignoreFallUntilClear = false;
  if (!isLowPositionCandidate()) ignoreLowUntilClear = false;
  if (sensorPacket.testMode != TEST_FORCE_ALARM) {
    ignoreForcedTestUntilClear = false;
  }
  if (isMoving() || !isPresent()) ignoreNoResponseUntilMovement = false;
  if (isPresent() || isDoorOpen) ignorePresenceLossUntilClear = false;
}

void updateState() {
  const uint32_t now = millis();
  const bool sosPressEvent = isSosPressed && !lastSosPressed;
  lastSosPressed = isSosPressed;
  clearAcknowledgedConditions();

  if (alarmLatched) {
    currentState = SystemState::ALARM_LATCHED;
    if (webReleaseRequested) releaseAlarm(now);
    return;
  }

  if (webReleaseRequested) webReleaseRequested = false;

  // 외부 SOS는 천장부 통신/센서 상태와 무관하게 항상 최우선입니다.
  if (sosPressEvent) {
    startAlarm(AlarmCause::SOS, "SOS 긴급 버튼", now);
    return;
  }

  if (webTestRequested) {
    webTestRequested = false;
    startAlarm(AlarmCause::WEB_TEST, "휴대폰 경보 시험", now);
    return;
  }

  if (!communicationOk(now)) {
    currentState = SystemState::COMM_ERROR;
    faultReason = (lastGoodPacketMs == 0 &&
                   now - bootStartedMs < Config::STARTUP_GRACE_MS)
                      ? "천장부 연결 대기"
                      : "천장부 통신 끊김";
    // 순수 통신 고장은 표시만 하고 BF395를 울리지 않습니다.
    return;
  }

  if (sensorPacket.testMode == TEST_FORCE_ALARM) {
    faultReason = "없음";
    if (!ignoreForcedTestUntilClear) {
      startAlarm(AlarmCause::CEILING_TEST, "천장부 강제 경보 시험", now);
    }
    return;
  }

  if (sensorPacket.testMode == TEST_FORCE_NORMAL) {
    currentState = SystemState::FORCED_NORMAL;
    faultReason = "없음";
    occupancyConfirmStartedMs = 0;
    presenceLostStartedMs = 0;
    fallConfirmStartedMs = 0;
    lowPositionStartedMs = 0;
    monitoringBaselineMs = now;
    return;
  }

  if (!isLdOk()) {
    faultReason = "LD2410C 통신 오류";
    // 이미 낙상 확인에 진입했다면 C1001 판단 타이머는 끝까지 유지합니다.
    if (currentState != SystemState::FALL_CONFIRM) {
      currentState = SystemState::SENSOR_ERROR;
      return;
    }
  } else if (!isC1001Ok()) {
    // LD 기반 재실/무반응 감시는 계속하고 낙상 기능 저하만 표시합니다.
    faultReason = "C1001 통신 오류 - 낙상 감지 제한";
  } else {
    faultReason = "없음";
  }

  if (currentState == SystemState::COMM_ERROR ||
      currentState == SystemState::SENSOR_ERROR ||
      currentState == SystemState::FORCED_NORMAL) {
    enterStandby(now);
  }

  if (isMoving()) {
    monitoringBaselineMs = now;
    ignoreNoResponseUntilMovement = false;
  }

  switch (currentState) {
    case SystemState::STANDBY:
    case SystemState::ALARM_RELEASED:
      if (isPresent()) {
        occupancyConfirmStartedMs = now;
        monitoringBaselineMs = now;
        currentState = SystemState::OCCUPANCY_CONFIRM;
      } else if (currentState == SystemState::ALARM_RELEASED &&
                 now - releasedMs >= 1000) {
        enterStandby(now);
      }
      break;

    case SystemState::OCCUPANCY_CONFIRM:
      if (!isPresent()) {
        enterStandby(now);
      } else if (now - occupancyConfirmStartedMs >=
                 Config::OCCUPANCY_CONFIRM_MS) {
        currentState = SystemState::MONITORING;
        monitoringBaselineMs = now;
      }
      break;

    case SystemState::MONITORING:
      if (isC1001Ok() && isFallSuspected() && !ignoreFallUntilClear) {
        fallConfirmStartedMs = now;
        currentState = SystemState::FALL_CONFIRM;
        break;
      }

      if (isLowPositionCandidate() && !isMoving() &&
          !ignoreLowUntilClear) {
        lowPositionStartedMs = now;
        currentState = SystemState::LOW_POSITION_CONFIRM;
        break;
      }

      if (!isPresent()) {
        if (presenceLostStartedMs == 0) {
          presenceLostStartedMs = now;
          doorOpenedDuringPresenceLoss = isDoorOpen;
        }
        if (isDoorOpen) doorOpenedDuringPresenceLoss = true;

        if (now - presenceLostStartedMs >= Config::PRESENCE_LOST_GRACE_MS) {
          if (doorOpenedDuringPresenceLoss) {
            enterStandby(now);
          } else if (!ignorePresenceLossUntilClear) {
            startAlarm(AlarmCause::PRESENCE_LOST_CLOSED_DOOR,
                       "문 닫힘 중 재실 신호 소실", now);
          }
        }
        break;
      }

      presenceLostStartedMs = 0;
      doorOpenedDuringPresenceLoss = false;
      if (effectiveNoMovementMs(now) >= Config::NO_RESPONSE_ALARM_MS &&
          !ignoreNoResponseUntilMovement) {
        startAlarm(AlarmCause::LONG_NO_RESPONSE, "장시간 무반응", now);
      }
      break;

    case SystemState::FALL_CONFIRM:
      if (isMoving()) {
        ignoreFallUntilClear = isFallSuspected();
        currentState = SystemState::MONITORING;
        monitoringBaselineMs = now;
      } else if (now - fallConfirmStartedMs >= Config::FALL_CONFIRM_MS) {
        startAlarm(AlarmCause::FALL_NO_RESPONSE,
                   "낙상 의심 후 무반응", now);
      }
      break;

    case SystemState::LOW_POSITION_CONFIRM:
      if (isC1001Ok() && isFallSuspected() && !ignoreFallUntilClear) {
        fallConfirmStartedMs = now;
        currentState = SystemState::FALL_CONFIRM;
      } else if (!isLowPositionCandidate() || isMoving()) {
        currentState = SystemState::MONITORING;
        monitoringBaselineMs = now;
      } else if (now - lowPositionStartedMs >=
                 Config::LOW_POSITION_CONFIRM_MS) {
        startAlarm(AlarmCause::LOW_POSITION,
                   "낮은 위치에서 정지", now);
      }
      break;

    case SystemState::ALARM_LATCHED:
    case SystemState::FORCED_NORMAL:
    case SystemState::COMM_ERROR:
    case SystemState::SENSOR_ERROR:
      break;
  }
}

String stateGuidance(uint32_t now) {
  if (alarmLatched) return String("경보 원인: ") + alarmReason;

  switch (currentState) {
    case SystemState::STANDBY: return "입실을 기다리는 중입니다.";
    case SystemState::OCCUPANCY_CONFIRM: return "재실 신호를 확인하고 있습니다.";
    case SystemState::MONITORING: {
      const uint32_t elapsed = effectiveNoMovementMs(now);
      const uint32_t remain = elapsed >= Config::NO_RESPONSE_ALARM_MS
                                  ? 0
                                  : (Config::NO_RESPONSE_ALARM_MS - elapsed + 999) / 1000;
      return String("무반응 경보까지 ") + remain + "초";
    }
    case SystemState::FALL_CONFIRM:
      return String("낙상 추가 확인 ") +
             ((Config::FALL_CONFIRM_MS -
               (now - fallConfirmStartedMs < Config::FALL_CONFIRM_MS
                    ? now - fallConfirmStartedMs
                    : Config::FALL_CONFIRM_MS) + 999) / 1000) + "초";
    case SystemState::LOW_POSITION_CONFIRM:
      return String("낮은 자세 추가 확인 ") +
             ((Config::LOW_POSITION_CONFIRM_MS -
               (now - lowPositionStartedMs < Config::LOW_POSITION_CONFIRM_MS
                    ? now - lowPositionStartedMs
                    : Config::LOW_POSITION_CONFIRM_MS) + 999) / 1000) + "초";
    case SystemState::ALARM_RELEASED: return "경보가 해제되었습니다.";
    case SystemState::FORCED_NORMAL: return "천장부 강제 정상 시험 중입니다.";
    case SystemState::COMM_ERROR: return faultReason;
    case SystemState::SENSOR_ERROR: return faultReason;
    case SystemState::ALARM_LATCHED: break;
  }
  return "상태 확인 중";
}

String jsonEscape(String value) {
  value.replace("\\", "\\\\");
  value.replace("\"", "\\\"");
  value.replace("\r", "");
  value.replace("\n", "\\n");
  return value;
}

String makeStatusJson() {
  const uint32_t now = millis();
  String json;
  json.reserve(1400);
  json += "{";
  json += "\"firmware\":\"" + String(Config::FIRMWARE_VERSION) + "\"";
  json += ",\"demoMode\":" + String(CONTEST_DEMO_MODE ? "true" : "false");
  json += ",\"state\":\"" + jsonEscape(stateName(currentState)) + "\"";
  json += ",\"guidance\":\"" + jsonEscape(stateGuidance(now)) + "\"";
  json += ",\"alarm\":" + String(alarmLatched ? "true" : "false");
  json += ",\"alarmReason\":\"" + jsonEscape(alarmReason) + "\"";
  json += ",\"faultReason\":\"" + jsonEscape(faultReason) + "\"";
  json += ",\"relay\":" + String(relayOutputOn ? "true" : "false");
  json += ",\"doorOpen\":" + String(isDoorOpen ? "true" : "false");
  json += ",\"sosPressed\":" + String(isSosPressed ? "true" : "false");
  json += ",\"online\":" + String(communicationOk(now) ? "true" : "false");
  json += ",\"packetAgeMs\":" +
          String(lastGoodPacketMs == 0 ? -1L
                                       : static_cast<long>(now - lastGoodPacketMs));
  json += ",\"sender\":\"" +
          (senderLocked ? formatMac(lockedSenderMac) : String("연결 대기")) + "\"";
  json += ",\"acceptedPackets\":" + String(acceptedPacketCount);
  json += ",\"authRejects\":" + String(rejectedAuthCount);
  json += ",\"replayRejects\":" + String(rejectedReplayCount);
  json += ",\"flags\":" + String(sensorPacket.flags);
  json += ",\"sequence\":" + String(sensorPacket.sequence);
  json += ",\"ceilingUptimeMs\":" + String(sensorPacket.uptimeMs);
  json += ",\"noMovementMs\":" + String(sensorPacket.noMovementMs);
  json += ",\"ldOk\":" + String(isLdOk() ? "true" : "false");
  json += ",\"c1001Ok\":" + String(isC1001Ok() ? "true" : "false");
  json += ",\"presence\":" + String(isPresent() ? "true" : "false");
  json += ",\"movement\":" + String(isMoving() ? "true" : "false");
  json += ",\"fallCandidate\":" + String(isFallSuspected() ? "true" : "false");
  json += ",\"still\":" + String(isStill() ? "true" : "false");
  json += ",\"lowPosition\":" + String(isLowPositionCandidate() ? "true" : "false");
  // int8_t는 String(char)로 해석될 수 있으므로 반드시 숫자로 승격합니다.
  json += ",\"cPresence\":" + String(static_cast<int>(sensorPacket.cPresence));
  json += ",\"cMovement\":" + String(static_cast<int>(sensorPacket.cMovement));
  json += ",\"cMovementRange\":" + String(sensorPacket.cMovementRange);
  json += ",\"cFall\":" + String(static_cast<int>(sensorPacket.cFall));
  json += ",\"cStill\":" + String(static_cast<int>(sensorPacket.cStill));
  json += ",\"ldState\":" + String(sensorPacket.ldState);
  json += ",\"ldStateName\":\"" + String(ldStateName(sensorPacket.ldState)) + "\"";
  json += ",\"ldMovingDistance\":" + String(sensorPacket.ldMovingDistance);
  json += ",\"ldMovingEnergy\":" + String(sensorPacket.ldMovingEnergy);
  json += ",\"ldStaticDistance\":" + String(sensorPacket.ldStaticDistance);
  json += ",\"ldStaticEnergy\":" + String(sensorPacket.ldStaticEnergy);
  json += ",\"ldDetectionDistance\":" + String(sensorPacket.ldDetectionDistance);
  json += ",\"testMode\":" + String(sensorPacket.testMode);
  json += ",\"testModeName\":\"" + String(testModeName(sensorPacket.testMode)) + "\"";
  json += ",\"phones\":" + String(WiFi.softAPgetStationNum());
  json += "}";
  return json;
}

const char MAIN_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WEFLASH 화장실 안전 모니터</title><style>
:root{color-scheme:light}*{box-sizing:border-box}body{margin:0;padding:16px;background:#eef2f7;color:#172033;font-family:Arial,'Noto Sans KR',sans-serif}.wrap{max-width:620px;margin:auto}h1{text-align:center;font-size:24px;margin:8px 0 4px}.sub{text-align:center;color:#687386;font-size:13px;margin-bottom:14px}.mode{text-align:center;padding:9px;border-radius:10px;background:#fff4cc;color:#6b4b00;font-weight:800;margin-bottom:12px}.banner{padding:18px;border-radius:15px;background:#daf3e3;color:#146c36;text-align:center;font-size:24px;font-weight:900}.banner.alarm{background:#d71920;color:#fff;animation:pulse 1s infinite}.banner.fault{background:#fff0d7;color:#925400}@keyframes pulse{50%{opacity:.72}}.guide{text-align:center;font-weight:700;padding:10px 4px 2px}.card{background:#fff;border-radius:15px;padding:16px;margin-top:12px;box-shadow:0 4px 16px #17203315}.card h2{font-size:17px;margin:0 0 8px}.row{display:flex;justify-content:space-between;gap:14px;padding:8px 0;border-bottom:1px solid #edf0f4}.row:last-child{border:0}.row b{text-align:right;word-break:break-all}.good{color:#16803c}.bad{color:#c62828}.warn{color:#b05a00}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:10px}.buttons button{border:0;border-radius:11px;padding:15px 8px;font-size:17px;font-weight:900;color:#fff;cursor:pointer}.test{background:#d95c11}.reset{background:#1769aa}.muted{font-size:12px;color:#687386}.privacy{display:inline-block;padding:4px 9px;border-radius:999px;background:#e8f6ed;color:#126b35;font-size:12px;font-weight:800}
</style></head><body><main class="wrap"><h1>WEFLASH 화장실 안전 모니터</h1><div class="sub"><span class="privacy">카메라·영상 저장 없음</span> 센서 융합 · ESP-NOW</div><div id="mode" class="mode" hidden></div><div id="banner" class="banner">연결 확인 중</div><div id="guide" class="guide">잠시 기다려 주세요.</div>
<section class="card"><h2>경보 및 외부 입력</h2><div class="row"><span>경보 원인</span><b id="alarmReason">-</b></div><div class="row"><span>시스템 알림</span><b id="faultReason">-</b></div><div class="row"><span>릴레이 / BF395</span><b id="relay">-</b></div><div class="row"><span>문</span><b id="door">-</b></div><div class="row"><span>SOS 버튼</span><b id="sos">-</b></div></section>
<section class="card"><h2>천장부 통신</h2><div class="row"><span>ESP-NOW</span><b id="online">-</b></div><div class="row"><span>송신 보드</span><b id="sender">-</b></div><div class="row"><span>수신 순번 / 패킷 나이</span><b id="packet">-</b></div><div class="row"><span>인증 실패 / 재전송 차단</span><b id="security">-</b></div><div class="row"><span>천장부 시험 모드</span><b id="testMode">-</b></div></section>
<section class="card"><h2>판정값</h2><div class="row"><span>재실 / 움직임</span><b id="presenceMovement">-</b></div><div class="row"><span>낙상 / 정지</span><b id="fallStill">-</b></div><div class="row"><span>낮은 위치 후보</span><b id="low">-</b></div><div class="row"><span>무움직임 시간</span><b id="noMovement">-</b></div></section>
<section class="card"><h2>C1001 원시 측정값</h2><div class="row"><span>통신</span><b id="cOk">-</b></div><div class="row"><span>재실 / 움직임</span><b id="cPresenceMovement">-</b></div><div class="row"><span>움직임 범위</span><b id="cRange">-</b></div><div class="row"><span>낙상 / 정지</span><b id="cFallStill">-</b></div></section>
<section class="card"><h2>LD2410C 원시 측정값</h2><div class="row"><span>통신 / 상태</span><b id="ldOkState">-</b></div><div class="row"><span>이동 거리 / 에너지</span><b id="ldMoving">-</b></div><div class="row"><span>정지 거리 / 에너지</span><b id="ldStatic">-</b></div><div class="row"><span>최종 감지 거리</span><b id="ldDetection">-</b></div></section>
<section class="card"><div class="buttons"><button class="test" onclick="post('/api/test')">경보 시험</button><button class="reset" onclick="post('/api/release')">경보 해제</button></div><p class="muted">경보는 해제 전까지 유지됩니다. 센서·통신 오류만으로는 사이렌이 울리지 않습니다.</p></section>
<section class="card muted">펌웨어 <span id="firmware">-</span> · 휴대폰 <span id="phones">-</span>대 · http://192.168.4.1</section></main><script>
const el=id=>document.getElementById(id);function set(id,text,cls=''){el(id).textContent=text;el(id).className=cls}function yn(v,a='감지',b='없음'){return v?a:b}async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'});const s=await r.json();const banner=el('banner');banner.textContent=s.alarm?'경보 발생':(s.online?s.state:'통신 확인 필요');banner.className='banner'+(s.alarm?' alarm':(!s.online||s.faultReason!=='없음'?' fault':''));set('guide',s.guidance);set('alarmReason',s.alarmReason,s.alarm?'bad':'good');set('faultReason',s.faultReason,s.faultReason==='없음'?'good':'warn');set('relay',s.relay?'ON':'OFF',s.relay?'bad':'good');set('door',s.doorOpen?'열림':'닫힘');set('sos',s.sosPressed?'눌림':'정상',s.sosPressed?'bad':'good');set('online',s.online?'정상':'끊김',s.online?'good':'bad');set('sender',s.sender);set('packet',s.sequence+' / '+(s.packetAgeMs<0?'-':s.packetAgeMs+' ms'));set('security',s.authRejects+' / '+s.replayRejects);set('testMode',s.testModeName,s.testMode===0?'good':'warn');set('presenceMovement',yn(s.presence,'사람 있음','비어 있음')+' / '+yn(s.movement),s.movement?'good':'');set('fallStill',yn(s.fallCandidate)+' / '+yn(s.still),s.fallCandidate?'bad':'');set('low',yn(s.lowPosition),s.lowPosition?'warn':'good');set('noMovement',(s.noMovementMs/1000).toFixed(1)+'초');set('cOk',s.c1001Ok?'정상':'오류',s.c1001Ok?'good':'bad');set('cPresenceMovement',s.cPresence+' / '+s.cMovement);set('cRange',s.cMovementRange);set('cFallStill',s.cFall+' / '+s.cStill);set('ldOkState',(s.ldOk?'정상':'오류')+' / '+s.ldStateName,s.ldOk?'good':'bad');set('ldMoving',s.ldMovingDistance+' cm / '+s.ldMovingEnergy);set('ldStatic',s.ldStaticDistance+' cm / '+s.ldStaticEnergy);set('ldDetection',s.ldDetectionDistance+' cm');set('firmware',s.firmware);set('phones',s.phones);const mode=el('mode');mode.hidden=!s.demoMode;mode.textContent='공모전 시연 모드 · 장시간 무반응 15초';}catch(e){const banner=el('banner');banner.textContent='페이지 통신 오류';banner.className='banner fault'}}async function post(path){try{await fetch(path,{method:'POST'});setTimeout(refresh,100)}catch(e){alert('요청 전송 실패')}}refresh();setInterval(refresh,750);
</script></body></html>
)HTML";

void sendNoCacheJson(int status, const String& body) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.send(status, "application/json; charset=UTF-8", body);
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=UTF-8", MAIN_PAGE);
  });
  server.on("/api/status", HTTP_GET, []() {
    sendNoCacheJson(200, makeStatusJson());
  });
  server.on("/api/test", HTTP_POST, []() {
    // 이미 래치된 경보 중 눌린 시험 요청이 해제 뒤 재경보로 남지 않게 합니다.
    if (!alarmLatched) webTestRequested = true;
    sendNoCacheJson(200, "{\"ok\":true}");
  });
  server.on("/api/release", HTTP_POST, []() {
    if (alarmLatched) webReleaseRequested = true;
    sendNoCacheJson(200, "{\"ok\":true}");
  });
  server.on("/favicon.ico", HTTP_GET, []() {
    server.send(204, "text/plain", "");
  });
  server.onNotFound([]() {
    sendNoCacheJson(404, "{\"error\":\"not found\"}");
  });
  server.begin();
}

bool initializeWireless() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);

  if (!WiFi.softAP(Config::AP_SSID, Config::AP_PASSWORD,
                   Config::WIFI_CHANNEL)) {
    Serial.println("[Wi-Fi] SoftAP 시작 실패");
    return false;
  }

  Serial.print("[Wi-Fi] 이름: ");
  Serial.println(Config::AP_SSID);
  Serial.print("[Wi-Fi] 주소: http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("[Wi-Fi] SoftAP MAC: ");
  Serial.println(WiFi.softAPmacAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] 초기화 실패");
    return false;
  }
  esp_now_register_recv_cb(onEspNowReceive);
  Serial.println("[ESP-NOW] 채널 6 HMAC 수신 준비 완료");
  return true;
}

void printStatus() {
  const uint32_t now = millis();
  if (now - lastStatusPrintMs < Config::STATUS_PRINT_MS) return;
  lastStatusPrintMs = now;

  Serial.printf(
      "[외부부] 상태:%s 문:%s 재실:%u 움직임:%u 낙상:%u 낮은위치:%u "
      "통신:%s LD:%s C1001:%s 경보:%s seq:%lu age:%lums\n",
      stateName(currentState), isDoorOpen ? "열림" : "닫힘",
      isPresent(), isMoving(), isFallSuspected(), isLowPositionCandidate(),
      communicationOk(now) ? "정상" : "오류",
      isLdOk() ? "정상" : "오류", isC1001Ok() ? "정상" : "오류",
      alarmLatched ? alarmReason : "없음",
      static_cast<unsigned long>(sensorPacket.sequence),
      lastGoodPacketMs == 0
          ? 0UL
          : static_cast<unsigned long>(now - lastGoodPacketMs));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  bootStartedMs = millis();

  Serial.println();
  Serial.printf("===== 외부 제어부 최종 코드 v%s =====\n",
                Config::FIRMWARE_VERSION);
#if CONTEST_DEMO_MODE
  Serial.println("[모드] 공모전 시연 - 장시간 무반응 15초");
#else
  Serial.println("[모드] 일반 운용 시험 - 장시간 무반응 300초");
#endif

  const uint8_t inactive =
      Config::RELAY_ACTIVE_LEVEL == HIGH ? LOW : HIGH;
  digitalWrite(Config::RELAY_PIN, inactive);
  pinMode(Config::RELAY_PIN, OUTPUT);
  relayOutputOn = false;

  pinMode(Config::DOOR_PIN, INPUT_PULLUP);
  pinMode(Config::SOS_PIN, INPUT_PULLUP);
  doorOpenInput.begin();
  sosPressedInput.begin();
  readHardwareInputs();
  lastSosPressed = false;  // 부팅 중 눌린 SOS도 첫 루프에서 경보 처리

  if (!initializeWireless()) {
    Serial.println("[치명적 오류] 무선 기능 시작 실패 - 3초 후 재부팅");
    delay(3000);
    ESP.restart();
  }

  setupWebServer();
  Serial.println("[준비 완료] 휴대폰과 천장부 연결을 기다립니다.");
}

void loop() {
  server.handleClient();
  processReceivedPacket();
  readHardwareInputs();
  updateState();

  // 통신/센서 오류 상태는 표시만 합니다. 사람 위급 경보만 BF395를 켭니다.
  setRelayOutput(alarmLatched);
  printStatus();
  delay(5);
}
