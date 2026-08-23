#include <Arduino.h>
#include "DFRobot_HumanDetection.h"

// ==================== 핀 설정 ====================
// C1001 TX -> GPIO4, C1001 RX -> GPIO5
constexpr int C1001_RX_PIN = 4;
constexpr int C1001_TX_PIN = 5;

// LD2410C TX -> GPIO16, LD2410C RX -> GPIO17
constexpr int LD_RX_PIN = 16;
constexpr int LD_TX_PIN = 17;

constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t C1001_BAUD = 115200;
constexpr uint32_t LD_BAUD = 256000;

// 센서 감지면부터 바닥까지 실제 높이(cm)로 수정
constexpr uint16_t INSTALL_HEIGHT_CM = 240;

constexpr uint8_t FALL_SENSITIVITY = 3;
// 실기에서 저장이 확인된 값. 2초 명령은 거부되고 60초 기본값이 유지됐다.
constexpr uint8_t FALL_CONFIRM_DELAY_SEC = 5;
constexpr uint8_t SETTING_RETRIES = 3;
constexpr uint16_t STILL_DWELL_SEC = 15;
constexpr uint32_t PRINT_INTERVAL_MS = 1000;

// C1001은 UART1, LD2410C는 UART2 사용
DFRobot_HumanDetection c1001(&Serial1);
HardwareSerial ldSerial(2);

// ==================== LD2410C 값 ====================
uint8_t ldBuffer[64];
uint8_t ldIndex = 0;

uint8_t ldState = 0;
uint16_t ldMovingDistance = 0;
uint8_t ldMovingEnergy = 0;
uint16_t ldStaticDistance = 0;
uint8_t ldStaticEnergy = 0;
uint16_t ldDetectionDistance = 0;

uint32_t lastLdFrameMs = 0;
uint32_t lastPrintMs = 0;
int previousFallState = -1;

void waitForC1001() {
  while (c1001.begin() != 0) {
    Serial.println("[C1001] 연결 실패");
    delay(1000);
  }

  Serial.println("[C1001] 연결 성공");
}

void setupC1001() {
  while (c1001.configWorkMode(c1001.eFallingMode) != 0) {
    Serial.println("[C1001] 낙상 모드 설정 실패");
    delay(1000);
  }

  c1001.configLEDLight(c1001.eFALLLed, 1);
  c1001.configLEDLight(c1001.eHPLed, 1);
  c1001.dmInstallHeight(INSTALL_HEIGHT_CM);
  // 반환값이 없는 설정 API라서 3회 전송하고 재부팅 후 저장값을 확인한다.
  for (uint8_t attempt = 0; attempt < SETTING_RETRIES; ++attempt) {
    c1001.dmFallTime(FALL_CONFIRM_DELAY_SEC);
    delay(200);
  }
  c1001.dmUnmannedTime(1);
  c1001.dmFallConfig(c1001.eResidenceTime, STILL_DWELL_SEC);
  c1001.dmFallConfig(c1001.eFallSensitivityC, FALL_SENSITIVITY);

  c1001.sensorRet();
  delay(2000);
  waitForC1001();

  uint32_t savedFallDelay = 0;
  for (uint8_t attempt = 0; attempt < SETTING_RETRIES; ++attempt) {
    savedFallDelay = c1001.getFallTime();
    if (savedFallDelay == FALL_CONFIRM_DELAY_SEC) break;
    delay(200);
  }
  if (savedFallDelay == FALL_CONFIRM_DELAY_SEC) {
    Serial.printf("[C1001] 낙상 지연 저장 확인: %lu초\n",
                  static_cast<unsigned long>(savedFallDelay));
  } else {
    Serial.printf("[C1001] 낙상 지연 저장 실패: %lu/%u초\n",
                  static_cast<unsigned long>(savedFallDelay),
                  FALL_CONFIRM_DELAY_SEC);
  }
}

void readLD2410C() {
  while (ldSerial.available()) {
    const uint8_t value = ldSerial.read();

    // 기본 데이터 프레임 헤더: F4 F3 F2 F1
    if (ldIndex == 0 && value != 0xF4) continue;

    if (ldIndex == 1 && value != 0xF3) {
      ldIndex = 0;
      continue;
    }

    if (ldIndex == 2 && value != 0xF2) {
      ldIndex = 0;
      continue;
    }

    if (ldIndex == 3 && value != 0xF1) {
      ldIndex = 0;
      continue;
    }

    ldBuffer[ldIndex++] = value;

    // LD2410C 기본 모드 프레임은 총 23바이트
    if (ldIndex == 23) {
      const bool validFooter =
          ldBuffer[19] == 0xF8 &&
          ldBuffer[20] == 0xF7 &&
          ldBuffer[21] == 0xF6 &&
          ldBuffer[22] == 0xF5;

      if (validFooter) {
        ldState = ldBuffer[8];
        ldMovingDistance = ldBuffer[9] | (ldBuffer[10] << 8);
        ldMovingEnergy = ldBuffer[11];
        ldStaticDistance = ldBuffer[12] | (ldBuffer[13] << 8);
        ldStaticEnergy = ldBuffer[14];
        ldDetectionDistance = ldBuffer[15] | (ldBuffer[16] << 8);
        lastLdFrameMs = millis();
      }

      ldIndex = 0;
    }

    if (ldIndex >= sizeof(ldBuffer)) {
      ldIndex = 0;
    }
  }
}

const char* ldStateText(uint8_t state) {
  switch (state) {
    case 0:
      return "사람 없음";
    case 1:
      return "움직임";
    case 2:
      return "정지";
    case 3:
      return "움직임+정지";
    default:
      return "알 수 없음";
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(1500);

  Serial1.begin(
      C1001_BAUD,
      SERIAL_8N1,
      C1001_RX_PIN,
      C1001_TX_PIN
  );

  waitForC1001();
  setupC1001();

  ldSerial.begin(
      LD_BAUD,
      SERIAL_8N1,
      LD_RX_PIN,
      LD_TX_PIN
  );

  Serial.println();
  Serial.println("=== C1001 + LD2410C 통합 테스트 시작 ===");
  Serial.printf("C1001: TX->GPIO%d, RX->GPIO%d\n",
                C1001_RX_PIN, C1001_TX_PIN);
  Serial.printf("LD2410C: TX->GPIO%d, RX->GPIO%d\n",
                LD_RX_PIN, LD_TX_PIN);
  Serial.println("C1001 움직임값: 0~100 강도값(거리가 아님)");
  Serial.println();
}

void loop() {
  // LD2410C 데이터는 지연 없이 계속 읽어야 한다.
  readLD2410C();

  if (millis() - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }

  lastPrintMs = millis();

  const int cPresence = c1001.smHumanData(c1001.eHumanPresence);
  const int cMovement = c1001.smHumanData(c1001.eHumanMovement);
  const int cMovementRange = c1001.smHumanData(c1001.eHumanMovingRange);
  // 패치 라이브러리로 실제 UART 성공 여부와 정상적인 '낙상 없음(0)'을 구분한다.
  uint16_t checkedFallState = 0;
  const int fallState =
      c1001.getFallStateChecked(checkedFallState) == 0 &&
              checkedFallState <= 1
          ? static_cast<int>(checkedFallState)
          : -1;
  const int stillState = c1001.getFallData(c1001.estaticResidencyState);

  // C1001 통신 중 들어온 LD 데이터를 다시 비운다.
  readLD2410C();

  Serial.println("----------------------------------------");
  Serial.printf(
      "[C1001] 사람=%d 움직임=%d 움직임값=%d 낙상=%d 정지=%d\n",
      cPresence,
      cMovement,
      cMovementRange,
      fallState,
      stillState
  );
  const bool ldConnected =
      lastLdFrameMs != 0 && millis() - lastLdFrameMs < 2000;

  if (!ldConnected) {
    Serial.println("[LD2410C] 통신 없음");
  } else {
    Serial.printf(
        "[LD2410C] 상태=%s(%u) 감지거리=%u cm\n",
        ldStateText(ldState),
        ldState,
        ldDetectionDistance
    );

    Serial.printf(
        "          움직임=%u cm/에너지 %u, 정지=%u cm/에너지 %u\n",
        ldMovingDistance,
        ldMovingEnergy,
        ldStaticDistance,
        ldStaticEnergy
    );
  }

  if (fallState == 1 && previousFallState != 1) {
    Serial.println(">>> 낙상 감지 <<<");
  } else if (fallState == 0 && previousFallState == 1) {
    Serial.println(">>> 낙상 상태 해제 <<<");
  }

  previousFallState = fallState;
}
