#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#include <DFRobot_HumanDetection.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mbedtls/md.h>
#include <stddef.h>
#include "project_config.h"

// =============================================================================
// WEFLASH 천장 센서부 최종 코드 v3.5.3
//
// 배선
//   C1001 TX -> GPIO4  (ESP32 RX)
//   C1001 RX -> GPIO5  (ESP32 TX)
//   LD2410C TX -> GPIO16 (ESP32 RX)
//   LD2410C RX -> GPIO17 (ESP32 TX)
//   두 센서의 5V/GND는 천장부 PCB 전원 레일을 공통 사용
//
// 시리얼 명령 (115200 baud)
//   A : 실제 센서 자동 모드
//   1 : 천장부 강제 경보 시험
//   0 : 강제 정상 시험
//   S : 현재 원본 센서값 즉시 출력
// =============================================================================

namespace Config {
constexpr char FIRMWARE_VERSION[] = "3.5.3";

constexpr int C1001_RX_PIN = 4;
constexpr int C1001_TX_PIN = 5;
constexpr int LD2410_RX_PIN = 16;
constexpr int LD2410_TX_PIN = 17;

constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t C1001_BAUD = 115200;
constexpr uint32_t LD2410_BAUD = 256000;

constexpr uint8_t WIFI_CHANNEL = 6;
constexpr uint32_t SEND_INTERVAL_MS = 250;
constexpr uint32_t STATUS_INTERVAL_MS = 2000;
constexpr uint32_t SENSOR_TIMEOUT_MS = 3000;
constexpr uint32_t C1001_SAMPLE_TIMEOUT_MS = 15000;

// 실제 설치 시 센서 감지면부터 바닥까지 잰 높이로 수정합니다.
constexpr uint16_t C1001_INSTALL_HEIGHT_CM = 240;
// DFRobot 공식 낙상 예제값입니다. 실기에서 2초 설정은 센서가 거부하고
// 기본값 60초를 유지했으므로, 실제 저장이 확인된 5초를 사용합니다.
constexpr uint8_t C1001_FALL_CONFIRM_SECONDS = 5;
constexpr uint8_t C1001_FALL_SENSITIVITY = 3;
constexpr uint16_t C1001_STILL_DWELL_SECONDS = 15;
constexpr uint32_t C1001_POLL_INTERVAL_MS = 1000;
constexpr uint32_t C1001_RETRY_INTERVAL_MS = 10000;
constexpr uint8_t C1001_MAX_CONSECUTIVE_FAILURES = 3;
constexpr uint8_t C1001_SETTING_RETRIES = 3;
constexpr uint32_t C1001_TASK_STACK_SIZE = 8192;

// LD2410C의 정지 대상 값이 이 정도 이상 변하면 작은 움직임으로 봅니다.
constexpr uint16_t LD_STATIONARY_DISTANCE_DELTA_CM = 8;
constexpr uint8_t LD_STATIONARY_ENERGY_DELTA = 5;

// 센서가 천장을 향해 아래로 보는 설치 기준입니다. 센서에서 대상까지의
// 거리가 커질수록 바닥에 가까운 자세이므로 170 cm 이상을 초기 기준으로 둡니다.
constexpr uint16_t LOW_POSITION_MIN_CM = 170;

// project_config.h의 값은 외부 제어부와 반드시 같아야 합니다.
constexpr uint8_t PACKET_AUTH_KEY[32] = WEFLASH_PACKET_AUTH_KEY;
}  // namespace Config

static_assert(Config::WIFI_CHANNEL >= 1 && Config::WIFI_CHANNEL <= 13,
              "Wi-Fi channel must be between 1 and 13");
static_assert(Config::C1001_RX_PIN != Config::C1001_TX_PIN &&
                  Config::C1001_RX_PIN != Config::LD2410_RX_PIN &&
                  Config::C1001_RX_PIN != Config::LD2410_TX_PIN &&
                  Config::C1001_TX_PIN != Config::LD2410_RX_PIN &&
                  Config::C1001_TX_PIN != Config::LD2410_TX_PIN &&
                  Config::LD2410_RX_PIN != Config::LD2410_TX_PIN,
              "UART GPIO assignments must be unique");
static_assert(Config::C1001_FALL_SENSITIVITY <= 3,
              "C1001 sensitivity must be 0..3");
static_assert(Config::SENSOR_TIMEOUT_MS > Config::SEND_INTERVAL_MS,
              "Sensor timeout must exceed send interval");
static_assert(Config::C1001_SAMPLE_TIMEOUT_MS >
                  Config::C1001_POLL_INTERVAL_MS,
              "C1001 timeout must exceed its polling interval");

constexpr uint16_t PACKET_MAGIC = 0xBADA;
constexpr uint8_t PACKET_VERSION = 2;

enum PacketFlag : uint8_t {
  // LD_OK는 최근 3초 이내 정상 기본 출력 프레임을 받았다는 뜻입니다.
  FLAG_LD_OK = 1U << 0,
  // C1001_OK는 낙상 상태 UART 응답을 포함한 최신 샘플이 15초 이내라는 뜻입니다.
  FLAG_C1001_OK = 1U << 1,
  FLAG_LD_PRESENCE = 1U << 2,
  FLAG_DERIVED_MOVEMENT = 1U << 3,
  FLAG_C1001_FALL = 1U << 4,
  FLAG_C1001_STILL = 1U << 5,
  // LOW_POSITION은 낙상 확정이 아니라 낮은 자세 후보입니다.
  FLAG_LOW_POSITION = 1U << 6
};

enum TestMode : uint8_t {
  TEST_AUTO = 0,
  TEST_FORCE_ALARM = 1,
  TEST_FORCE_NORMAL = 2
};

// 외부 제어부의 SystemPacket과 필드 순서, 자료형, packed 지정이 모두 같아야 합니다.
struct __attribute__((packed)) SystemPacket {
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

static_assert(offsetof(SystemPacket, authTag) == 32,
              "SystemPacket authenticated region mismatch");
static_assert(sizeof(SystemPacket) == 40, "SystemPacket size mismatch");

const uint8_t BROADCAST_MAC[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

HardwareSerial c1001Serial(1);
HardwareSerial ld2410Serial(2);
DFRobot_HumanDetection c1001(&c1001Serial);

// -----------------------------------------------------------------------------
// C1001: 공식 라이브러리 호출은 응답이 없으면 수 초간 기다릴 수 있으므로
// 전용 FreeRTOS 태스크에서만 실행합니다.
// -----------------------------------------------------------------------------

struct C1001SharedSample {
  int16_t movementRange;
  int8_t presence;
  int8_t movement;
  int8_t fall;
  int8_t still;
  uint32_t lastOkMs;
  // 설정 전체가 일치한다는 뜻이 아니라, 초기화 후 정상 런타임 샘플을
  // 한 번 이상 발행했다는 뜻입니다. lastOkMs와 함께 최신성을 판정합니다.
  bool configured;
};

portMUX_TYPE c1001Mux = portMUX_INITIALIZER_UNLOCKED;
C1001SharedSample c1001Shared = {-1, -1, -1, -1, -1, 0, false};

void markC1001Offline() {
  portENTER_CRITICAL(&c1001Mux);
  c1001Shared.configured = false;
  c1001Shared.lastOkMs = 0;
  portEXIT_CRITICAL(&c1001Mux);
}

void publishC1001Sample(int presence, int movement, int movementRange,
                        int fall, int still) {
  portENTER_CRITICAL(&c1001Mux);
  c1001Shared.presence = static_cast<int8_t>(presence);
  c1001Shared.movement = static_cast<int8_t>(movement);
  c1001Shared.movementRange = static_cast<int16_t>(movementRange);
  c1001Shared.fall = static_cast<int8_t>(fall);
  c1001Shared.still = static_cast<int8_t>(still);
  c1001Shared.lastOkMs = millis();
  c1001Shared.configured = true;
  portEXIT_CRITICAL(&c1001Mux);
}

C1001SharedSample snapshotC1001() {
  C1001SharedSample sample;
  portENTER_CRITICAL(&c1001Mux);
  sample = c1001Shared;
  portEXIT_CRITICAL(&c1001Mux);
  return sample;
}

bool initializeC1001() {
  Serial.println("[C1001] 초기화 시작 (센서 부팅 대기 포함)");

  if (c1001.begin() != 0) {
    Serial.println("[C1001] 연결 실패 - 5V/GND와 TX/RX를 확인하세요.");
    return false;
  }

  if (c1001.configWorkMode(c1001.eFallingMode) != 0) {
    Serial.println("[C1001] 낙상 모드 설정 실패");
    return false;
  }

  // 설정 명령은 일부가 반환값이 없는 라이브러리 API입니다. 설정 후 센서를
  // 재시작하고 읽어온 값으로 최종 검증합니다.
  c1001.configLEDLight(c1001.eFALLLed, 1);
  c1001.configLEDLight(c1001.eHPLed, 1);
  c1001.dmInstallHeight(Config::C1001_INSTALL_HEIGHT_CM);
  // upstream API가 설정 성공 여부를 반환하지 않으므로 여러 번 전송하고
  // 센서 재시작 뒤 실제 저장값을 별도로 확인합니다.
  for (uint8_t attempt = 0;
       attempt < Config::C1001_SETTING_RETRIES; ++attempt) {
    c1001.dmFallTime(Config::C1001_FALL_CONFIRM_SECONDS);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  c1001.dmUnmannedTime(1);

  if (c1001.dmFallConfig(c1001.eResidenceTime,
                         Config::C1001_STILL_DWELL_SECONDS) != 0 ||
      c1001.dmFallConfig(c1001.eFallSensitivityC,
                         Config::C1001_FALL_SENSITIVITY) != 0) {
    Serial.println("[C1001] 정지시간/낙상 민감도 설정 실패");
    return false;
  }

  if (c1001.sensorRet() != 0 || c1001.begin() != 0) {
    Serial.println("[C1001] 설정 저장 후 재연결 실패");
    return false;
  }

  // 리셋 후 낙상 모드를 다시 보장합니다. configWorkMode()는 단순 getter와
  // 달리 현재 모드를 확인하고 필요하면 전환한 뒤 성공 여부를 반환합니다.
  if (c1001.configWorkMode(c1001.eFallingMode) != 0) {
    Serial.println("[C1001] 재시작 후 낙상 모드 확인 실패");
    return false;
  }

  // 일부 C1001 펌웨어는 설정 저장 직후 연속 조회 중 한 항목의 응답을
  // 간헐적으로 놓칩니다. 아래 getter는 진단 출력에만 사용하며, 실제 통신
  // 상태는 이어지는 getFallStateChecked() 성공 여부로 판정합니다.
  const uint8_t savedMode = c1001.getWorkMode();
  const uint16_t savedHeight = c1001.dmGetInstallHeight();
  uint32_t savedFallTime = 0;
  for (uint8_t attempt = 0;
       attempt < Config::C1001_SETTING_RETRIES; ++attempt) {
    savedFallTime = c1001.getFallTime();
    if (savedFallTime == Config::C1001_FALL_CONFIRM_SECONDS) break;
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  const uint32_t savedDwell = c1001.getStaticResidencyTime();
  const uint16_t savedSensitivity =
      c1001.getFallData(c1001.eFallSensitivity);

  Serial.printf(
      "[C1001] 저장 설정: 모드=%u/%u, 높이=%u/%u cm, 낙상지연=%lu/%u초, "
      "정지=%lu/%u초, 민감도=%u/%u\n",
      savedMode, c1001.eFallingMode,
      savedHeight, Config::C1001_INSTALL_HEIGHT_CM,
      static_cast<unsigned long>(savedFallTime),
      Config::C1001_FALL_CONFIRM_SECONDS,
      static_cast<unsigned long>(savedDwell),
      Config::C1001_STILL_DWELL_SECONDS,
      savedSensitivity, Config::C1001_FALL_SENSITIVITY);

  if (savedFallTime != Config::C1001_FALL_CONFIRM_SECONDS) {
    Serial.println(
        "[C1001] 낙상 지연 설정 실패 - 10초 후 다시 설정합니다.");
    return false;
  }

  const bool allSettingsMatch =
      savedMode == c1001.eFallingMode &&
      savedHeight == Config::C1001_INSTALL_HEIGHT_CM &&
      savedFallTime == Config::C1001_FALL_CONFIRM_SECONDS &&
      savedDwell == Config::C1001_STILL_DWELL_SECONDS &&
      savedSensitivity == Config::C1001_FALL_SENSITIVITY;
  if (!allSettingsMatch) {
    Serial.println(
        "[C1001] 경고: 일부 설정값 불일치/조회 누락 - "
        "낙상 상태 실시간 응답으로 동작을 확인합니다.");
  }

  Serial.printf("[C1001] 준비 완료: 높이=%u cm, 민감도=%u, 정지=%u초\n",
                Config::C1001_INSTALL_HEIGHT_CM,
                Config::C1001_FALL_SENSITIVITY,
                Config::C1001_STILL_DWELL_SECONDS);
  return true;
}

void c1001Task(void *) {
  bool initialized = false;
  uint8_t consecutiveFailures = 0;

  for (;;) {
    if (!initialized) {
      markC1001Offline();
      initialized = initializeC1001();

      if (!initialized) {
        Serial.printf("[C1001] %lu초 후 다시 연결합니다.\n",
                      static_cast<unsigned long>(
                          Config::C1001_RETRY_INTERVAL_MS / 1000UL));
        vTaskDelay(pdMS_TO_TICKS(Config::C1001_RETRY_INTERVAL_MS));
        continue;
      }
      consecutiveFailures = 0;
    }

    // 수정된 DFRobot 라이브러리의 checked API로 UART 성공 여부를 보존합니다.
    uint16_t fall = 0;
    if (c1001.getFallStateChecked(fall) != 0 || fall > 1) {
      ++consecutiveFailures;
      Serial.printf("[C1001] 낙상 상태 읽기 실패 (%u/%u)\n",
                    consecutiveFailures,
                    Config::C1001_MAX_CONSECUTIVE_FAILURES);

      if (consecutiveFailures >= Config::C1001_MAX_CONSECUTIVE_FAILURES) {
        markC1001Offline();
        initialized = false;
        consecutiveFailures = 0;
      }

      vTaskDelay(pdMS_TO_TICKS(Config::C1001_POLL_INTERVAL_MS));
      continue;
    }

    const int presence = c1001.smHumanData(c1001.eHumanPresence);
    const int movement = c1001.smHumanData(c1001.eHumanMovement);
    const int movementRange = c1001.smHumanData(c1001.eHumanMovingRange);
    const int still = c1001.getFallData(c1001.estaticResidencyState);

    const bool valuesValid =
        presence >= 0 && presence <= 1 && movement >= 0 && movement <= 2 &&
        movementRange >= 0 && movementRange <= INT16_MAX &&
        still >= 0 && still <= 1;

    if (!valuesValid) {
      ++consecutiveFailures;
      Serial.printf("[C1001] 범위를 벗어난 응답 (%u/%u)\n",
                    consecutiveFailures,
                    Config::C1001_MAX_CONSECUTIVE_FAILURES);
      if (consecutiveFailures >= Config::C1001_MAX_CONSECUTIVE_FAILURES) {
        markC1001Offline();
        initialized = false;
        consecutiveFailures = 0;
      }
    } else {
      consecutiveFailures = 0;
      publishC1001Sample(presence, movement, movementRange, fall, still);
    }

    vTaskDelay(pdMS_TO_TICKS(Config::C1001_POLL_INTERVAL_MS));
  }
}

bool startC1001Task() {
  const BaseType_t result =
      xTaskCreate(c1001Task, "c1001", Config::C1001_TASK_STACK_SIZE,
                  nullptr, 1, nullptr);
  if (result != pdPASS) {
    Serial.println("[C1001] 전용 태스크 생성 실패");
    return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
// LD2410C: 길이 필드와 헤더/꼬리를 모두 확인하는 스트림 파서입니다.
// 노이즈나 잘린 프레임이 들어와도 다음 정상 헤더부터 자동 복구합니다.
// -----------------------------------------------------------------------------

struct LdSample {
  uint8_t state = 0;
  uint16_t movingDistance = 0;
  uint8_t movingEnergy = 0;
  uint16_t staticDistance = 0;
  uint8_t staticEnergy = 0;
  uint16_t detectionDistance = 0;
  uint32_t lastFrameMs = 0;
  uint32_t lastActivityMs = 0;
  bool derivedMovement = false;
};

LdSample ldSample;
uint8_t ldBuffer[128] = {};
size_t ldBufferLength = 0;
bool hasStationarySample = false;
uint16_t previousStationaryDistance = 0;
uint8_t previousStationaryEnergy = 0;

void discardLdBytes(size_t count) {
  if (count >= ldBufferLength) {
    ldBufferLength = 0;
    return;
  }
  memmove(ldBuffer, ldBuffer + count, ldBufferLength - count);
  ldBufferLength -= count;
}

bool ldHeaderAt(size_t index) {
  return index + 3 < ldBufferLength && ldBuffer[index] == 0xF4 &&
         ldBuffer[index + 1] == 0xF3 && ldBuffer[index + 2] == 0xF2 &&
         ldBuffer[index + 3] == 0xF1;
}

void acceptBasicLdFrame(const uint8_t *frame) {
  const uint32_t now = millis();
  const uint8_t state = frame[8];
  const bool present = state != 0;
  const bool previousFrameFresh =
      ldSample.lastFrameMs != 0 &&
      now - ldSample.lastFrameMs <= Config::SENSOR_TIMEOUT_MS;
  const bool wasPresent = previousFrameFresh && ldSample.state != 0;
  const bool movingTarget = state == 1 || state == 3;
  bool stationaryActivity = false;

  const uint16_t staticDistance =
      static_cast<uint16_t>(frame[12]) |
      (static_cast<uint16_t>(frame[13]) << 8);
  const uint8_t staticEnergy = frame[14];

  if (state == 2 || state == 3) {
    if (hasStationarySample) {
      const uint16_t distanceDelta =
          staticDistance > previousStationaryDistance
              ? staticDistance - previousStationaryDistance
              : previousStationaryDistance - staticDistance;
      const uint8_t energyDelta =
          staticEnergy > previousStationaryEnergy
              ? staticEnergy - previousStationaryEnergy
              : previousStationaryEnergy - staticEnergy;
      stationaryActivity =
          distanceDelta >= Config::LD_STATIONARY_DISTANCE_DELTA_CM ||
          energyDelta >= Config::LD_STATIONARY_ENERGY_DELTA;
    }
    previousStationaryDistance = staticDistance;
    previousStationaryEnergy = staticEnergy;
    hasStationarySample = true;
  } else if (!present) {
    hasStationarySample = false;
  }

  ldSample.state = state;
  ldSample.movingDistance =
      static_cast<uint16_t>(frame[9]) |
      (static_cast<uint16_t>(frame[10]) << 8);
  ldSample.movingEnergy = frame[11];
  ldSample.staticDistance = staticDistance;
  ldSample.staticEnergy = staticEnergy;
  ldSample.detectionDistance =
      static_cast<uint16_t>(frame[15]) |
      (static_cast<uint16_t>(frame[16]) << 8);
  ldSample.derivedMovement = movingTarget || stationaryActivity;

  // 송신 주기 사이에 짧게 나타난 움직임도 무반응 타이머에 반영되도록
  // 패킷을 만들 때가 아니라 각 정상 LD 프레임을 받은 순간 기록합니다.
  if (!present || !wasPresent || ldSample.derivedMovement) {
    ldSample.lastActivityMs = now;
  }
  ldSample.lastFrameMs = now;
}

void processLdFrames() {
  for (;;) {
    if (ldBufferLength < 4) return;

    size_t headerIndex = 0;
    while (headerIndex + 3 < ldBufferLength && !ldHeaderAt(headerIndex)) {
      ++headerIndex;
    }

    if (headerIndex + 3 >= ldBufferLength) {
      // 다음 읽기에서 헤더가 완성될 수 있도록 마지막 3바이트를 남깁니다.
      if (ldBufferLength > 3) discardLdBytes(ldBufferLength - 3);
      return;
    }

    if (headerIndex > 0) discardLdBytes(headerIndex);
    if (ldBufferLength < 6) return;

    const uint16_t dataLength =
        static_cast<uint16_t>(ldBuffer[4]) |
        (static_cast<uint16_t>(ldBuffer[5]) << 8);
    const size_t frameLength = static_cast<size_t>(dataLength) + 10U;

    if (dataLength < 3 || frameLength > sizeof(ldBuffer)) {
      discardLdBytes(1);
      continue;
    }
    if (ldBufferLength < frameLength) return;

    const size_t footer = frameLength - 4;
    const bool footerOk =
        ldBuffer[footer] == 0xF8 && ldBuffer[footer + 1] == 0xF7 &&
        ldBuffer[footer + 2] == 0xF6 && ldBuffer[footer + 3] == 0xF5;

    if (!footerOk) {
      discardLdBytes(1);
      continue;
    }

    // 기본 출력 데이터: 길이 13, 02 AA, 상태 0~3, 데이터 꼬리 55 00
    const bool basicFrame =
        dataLength == 13 && ldBuffer[6] == 0x02 && ldBuffer[7] == 0xAA &&
        ldBuffer[8] <= 3 && ldBuffer[17] == 0x55 && ldBuffer[18] == 0x00;
    if (basicFrame) acceptBasicLdFrame(ldBuffer);

    discardLdBytes(frameLength);
  }
}

void readLd2410() {
  while (ld2410Serial.available() > 0) {
    if (ldBufferLength == sizeof(ldBuffer)) {
      processLdFrames();
      if (ldBufferLength == sizeof(ldBuffer)) discardLdBytes(1);
    }
    ldBuffer[ldBufferLength++] =
        static_cast<uint8_t>(ld2410Serial.read());

    // 정상 기본 프레임보다 충분히 모였을 때 수시로 비워 버퍼 지연을 줄입니다.
    if (ldBufferLength >= 23) processLdFrames();
  }
  processLdFrames();
}

bool ldHealthy(uint32_t now) {
  return ldSample.lastFrameMs != 0 &&
         now - ldSample.lastFrameMs <= Config::SENSOR_TIMEOUT_MS;
}

const char *ldStateName(uint8_t state) {
  switch (state) {
    case 0: return "없음";
    case 1: return "이동";
    case 2: return "정지";
    case 3: return "이동+정지";
    default: return "오류";
  }
}

// -----------------------------------------------------------------------------
// ESP-NOW와 HMAC
// -----------------------------------------------------------------------------

volatile uint32_t sendSuccessCount = 0;
volatile uint32_t sendFailureCount = 0;
uint32_t immediateSendFailureCount = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
using EspNowSendInfo = const wifi_tx_info_t *;
#else
using EspNowSendInfo = const uint8_t *;
#endif

void onEspNowSent(EspNowSendInfo, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    ++sendSuccessCount;
  } else {
    ++sendFailureCount;
  }
}

bool computePacketAuthTag(const SystemPacket &source, uint64_t &tag) {
  const mbedtls_md_info_t *info =
      mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (info == nullptr) return false;

  uint8_t digest[32] = {};
  const int result = mbedtls_md_hmac(
      info, Config::PACKET_AUTH_KEY, sizeof(Config::PACKET_AUTH_KEY),
      reinterpret_cast<const uint8_t *>(&source),
      offsetof(SystemPacket, authTag), digest);
  if (result != 0) return false;

  memcpy(&tag, digest, sizeof(tag));
  return true;
}

bool initializeEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  delay(100);

  // ESP-IDF/Arduino 코어 버전에 관계없이 STA 채널을 확실히 고정합니다.
  esp_wifi_set_promiscuous(true);
  const esp_err_t channelResult =
      esp_wifi_set_channel(Config::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (channelResult != ESP_OK) {
    Serial.printf("[ESP-NOW] 채널 설정 실패: %d\n", channelResult);
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] 초기화 실패");
    return false;
  }
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = Config::WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  const esp_err_t peerResult = esp_now_add_peer(&peer);
  if (peerResult != ESP_OK && peerResult != ESP_ERR_ESPNOW_EXIST) {
    Serial.printf("[ESP-NOW] 방송 피어 등록 실패: %d\n", peerResult);
    return false;
  }

  Serial.print("[ESP-NOW] 천장부 STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[ESP-NOW] 채널 %u 방송 송신 준비 완료\n",
                Config::WIFI_CHANNEL);
  return true;
}

// -----------------------------------------------------------------------------
// 패킷 생성, 시험 명령, 상태 출력
// -----------------------------------------------------------------------------

SystemPacket lastPacket = {};
TestMode currentTestMode = TEST_AUTO;
uint32_t sequenceNumber = 0;
uint32_t lastSendMs = 0;
uint32_t lastStatusMs = 0;
bool statusPrintRequested = false;

const char *testModeName(TestMode mode) {
  switch (mode) {
    case TEST_AUTO: return "자동";
    case TEST_FORCE_ALARM: return "강제경보";
    case TEST_FORCE_NORMAL: return "강제정상";
  }
  return "오류";
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    switch (command) {
      case 'a':
      case 'A':
        currentTestMode = TEST_AUTO;
        Serial.println("[시험] 실제 센서 자동 모드");
        break;
      case '1':
        currentTestMode = TEST_FORCE_ALARM;
        Serial.println("[시험] 강제 경보 모드");
        break;
      case '0':
        currentTestMode = TEST_FORCE_NORMAL;
        Serial.println("[시험] 강제 정상 모드");
        break;
      case 's':
      case 'S':
        statusPrintRequested = true;
        break;
      default:
        // Arduino 시리얼 모니터가 붙이는 CR/LF 등은 무시합니다.
        break;
    }
  }
}

SystemPacket buildPacket(uint32_t now) {
  const C1001SharedSample c = snapshotC1001();
  const bool c1001Ok = c.configured && c.lastOkMs != 0 &&
                       now - c.lastOkMs <=
                           Config::C1001_SAMPLE_TIMEOUT_MS;
  const bool ldOk = ldHealthy(now);
  const bool ldPresence = ldOk && ldSample.state != 0;
  const bool derivedMovement = ldPresence && ldSample.derivedMovement;

  uint32_t noMovementMs = 0;
  if (ldOk && ldPresence && ldSample.lastActivityMs != 0) {
    noMovementMs = now - ldSample.lastActivityMs;
  }

  const bool cFall = c1001Ok && c.fall == 1;
  const bool cStill = c1001Ok && c.still == 1;
  const bool lowPosition =
      ldOk && cStill && (ldSample.state == 2 || ldSample.state == 3) &&
      ldSample.staticEnergy > 0 &&
      ldSample.staticDistance >= Config::LOW_POSITION_MIN_CM;

  uint8_t flags = 0;
  if (ldOk) flags |= FLAG_LD_OK;
  if (c1001Ok) flags |= FLAG_C1001_OK;
  if (ldPresence) flags |= FLAG_LD_PRESENCE;
  if (derivedMovement) flags |= FLAG_DERIVED_MOVEMENT;
  if (cFall) flags |= FLAG_C1001_FALL;
  if (cStill) flags |= FLAG_C1001_STILL;
  if (lowPosition) flags |= FLAG_LOW_POSITION;

  SystemPacket packet = {};
  packet.magic = PACKET_MAGIC;
  packet.version = PACKET_VERSION;
  packet.flags = flags;
  packet.sequence = ++sequenceNumber;
  packet.uptimeMs = now;
  packet.noMovementMs = noMovementMs;
  packet.cMovementRange = c.movementRange;
  packet.ldMovingDistance = ldSample.movingDistance;
  packet.ldStaticDistance = ldSample.staticDistance;
  packet.ldDetectionDistance = ldSample.detectionDistance;
  packet.cPresence = c.presence;
  packet.cMovement = c.movement;
  packet.cFall = c.fall;
  packet.cStill = c.still;
  packet.ldState = ldSample.state;
  packet.ldMovingEnergy = ldSample.movingEnergy;
  packet.ldStaticEnergy = ldSample.staticEnergy;
  packet.testMode = static_cast<uint8_t>(currentTestMode);
  packet.authTag = 0;
  return packet;
}

void sendPacket(uint32_t now) {
  if (now - lastSendMs < Config::SEND_INTERVAL_MS) return;
  lastSendMs = now;

  SystemPacket packet = buildPacket(now);
  // SystemPacket은 무선 형식을 고정하려고 packed로 선언되어 있으므로
  // packed 멤버를 uint64_t 참조로 직접 넘기지 않고 정렬된 지역 변수에서
  // 계산한 뒤 복사합니다.
  uint64_t authTag = 0;
  if (!computePacketAuthTag(packet, authTag)) {
    ++immediateSendFailureCount;
    Serial.println("[보안 오류] HMAC 생성 실패");
    return;
  }
  memcpy(&packet.authTag, &authTag, sizeof(authTag));

  lastPacket = packet;
  const esp_err_t result = esp_now_send(
      BROADCAST_MAC, reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet));
  if (result != ESP_OK) {
    ++immediateSendFailureCount;
    Serial.printf("[ESP-NOW] 송신 요청 실패: %d\n", result);
  }
}

void printStatus(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - lastStatusMs < Config::STATUS_INTERVAL_MS) return;
  lastStatusMs = now;

  const bool ldOk = (lastPacket.flags & FLAG_LD_OK) != 0;
  const bool cOk = (lastPacket.flags & FLAG_C1001_OK) != 0;

  Serial.println("------------------------------------------------------------");
  Serial.printf(
      "[C1001] 통신=%s 사람=%d 움직임=%d 범위=%d 낙상=%d 정지=%d\n",
      cOk ? "정상" : "대기/오류", lastPacket.cPresence,
      lastPacket.cMovement, lastPacket.cMovementRange, lastPacket.cFall,
      lastPacket.cStill);
  Serial.printf(
      "[LD2410C] 통신=%s 상태=%s(%u) 감지=%u cm "
      "이동=%u cm/E%u 정지=%u cm/E%u\n",
      ldOk ? "정상" : "오류", ldStateName(lastPacket.ldState),
      lastPacket.ldState, lastPacket.ldDetectionDistance,
      lastPacket.ldMovingDistance, lastPacket.ldMovingEnergy,
      lastPacket.ldStaticDistance, lastPacket.ldStaticEnergy);
  Serial.printf(
      "[판정] 재실=%u 움직임=%u 낮은자세=%u 무움직임=%lu ms 시험=%s\n",
      (lastPacket.flags & FLAG_LD_PRESENCE) != 0,
      (lastPacket.flags & FLAG_DERIVED_MOVEMENT) != 0,
      (lastPacket.flags & FLAG_LOW_POSITION) != 0,
      static_cast<unsigned long>(lastPacket.noMovementMs),
      testModeName(static_cast<TestMode>(lastPacket.testMode)));
  Serial.printf(
      "[송신] seq=%lu 콜백성공=%lu 콜백실패=%lu 요청실패=%lu\n",
      static_cast<unsigned long>(lastPacket.sequence),
      static_cast<unsigned long>(sendSuccessCount),
      static_cast<unsigned long>(sendFailureCount),
      static_cast<unsigned long>(immediateSendFailureCount));
}

void setup() {
  Serial.begin(Config::USB_BAUD);
  delay(800);

  Serial.println();
  Serial.printf("===== WEFLASH 천장부 최종 코드 v%s =====\n",
                Config::FIRMWARE_VERSION);
  Serial.println("명령: A=자동, 1=강제경보, 0=강제정상, S=현재값");

  c1001Serial.begin(Config::C1001_BAUD, SERIAL_8N1,
                    Config::C1001_RX_PIN, Config::C1001_TX_PIN);
  ld2410Serial.begin(Config::LD2410_BAUD, SERIAL_8N1,
                     Config::LD2410_RX_PIN, Config::LD2410_TX_PIN);

  if (!startC1001Task()) {
    Serial.println("[성능 제한] C1001 태스크 시작 실패 - LD 전용으로 계속 동작합니다.");
  }

  if (!initializeEspNow()) {
    Serial.println("[치명적 오류] ESP-NOW 시작 실패 - 3초 후 재부팅");
    delay(3000);
    ESP.restart();
  }

  Serial.println("[READY] LD2410C 수집 및 ESP-NOW 방송 송신 시작");
  Serial.println("[안내] C1001은 별도 태스크에서 초기화되며 약 30~60초 걸릴 수 있습니다.");
}

void loop() {
  readLd2410();
  handleSerialCommands();

  const uint32_t now = millis();
  sendPacket(now);

  if (statusPrintRequested) {
    statusPrintRequested = false;
    printStatus(true);
  } else {
    printStatus(false);
  }

  delay(2);
}
