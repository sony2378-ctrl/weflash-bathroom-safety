#include <Arduino.h>
#include "DFRobot_HumanDetection.h"

// ------------------------------------------------------------
// C1001 단독 낙상 판정 시험용 (ESP32-S3)
//
// 배선
//   C1001 VIN -> 5V
//   C1001 GND -> GND
//   C1001 TX  -> ESP32-S3 GPIO4  (ESP RX)
//   C1001 RX  -> ESP32-S3 GPIO5  (ESP TX)
//
// 센서는 시험 구역 바로 위에서 감지면이 바닥을 향하도록 고정한다.
// ------------------------------------------------------------

constexpr int C1001_RX_PIN = 4;
constexpr int C1001_TX_PIN = 5;
constexpr uint32_t C1001_BAUD = 115200;
constexpr uint32_t USB_BAUD = 115200;

// 중요: 센서 감지면부터 바닥까지 실제로 잰 높이(cm)로 바꿀 것.
constexpr uint16_t INSTALL_HEIGHT_CM = 240;

// 실기 확인값: 민감도는 최대로 두고 낙상 확정 지연은 5초를 사용한다.
// 이 센서는 2초 명령을 거부하고 60초 기본값을 유지했으므로 2초로 낮추지 않는다.
constexpr uint8_t FALL_SENSITIVITY = 3;       // 0~3, 3이 가장 민감
constexpr uint8_t FALL_CONFIRM_DELAY_SEC = 5;
constexpr uint8_t SETTING_RETRIES = 3;
constexpr uint8_t NO_PERSON_DELAY_SEC = 1;
constexpr uint16_t STILL_DWELL_SEC = 15;      // 실신/정지 상태 참고용
constexpr uint32_t SAMPLE_INTERVAL_MS = 1000;

DFRobot_HumanDetection c1001(&Serial1);

int previousFallState = -1;
int previousStillState = -1;

void waitForSensor() {
  Serial.println("[INIT] Waiting for C1001...");
  while (c1001.begin() != 0) {
    Serial.println("[ERROR] C1001 no response - check 5V/GND and TX/RX crossover");
    delay(1000);
  }
  Serial.println("[OK] C1001 communication");
}

void setFallMode() {
  Serial.println("[INIT] Switching to fall-detection mode...");
  while (c1001.configWorkMode(c1001.eFallingMode) != 0) {
    Serial.println("[ERROR] Mode change failed - retrying");
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
  c1001.dmUnmannedTime(NO_PERSON_DELAY_SEC);
  c1001.dmFallConfig(c1001.eResidenceTime, STILL_DWELL_SEC);
  c1001.dmFallConfig(c1001.eFallSensitivityC, FALL_SENSITIVITY);

  // DFRobot 지침상 설정 저장 뒤 센서 리셋이 필요하다.
  c1001.sensorRet();
  delay(2000);

  // 리셋 뒤 통신이 다시 살아났는지 확인한다.
  waitForSensor();

  uint32_t savedFallDelay = 0;
  for (uint8_t attempt = 0; attempt < SETTING_RETRIES; ++attempt) {
    savedFallDelay = c1001.getFallTime();
    if (savedFallDelay == FALL_CONFIRM_DELAY_SEC) break;
    delay(200);
  }
  if (savedFallDelay != FALL_CONFIRM_DELAY_SEC) {
    Serial.printf("[ERROR] Fall delay not saved: %lu/%u sec\n",
                  static_cast<unsigned long>(savedFallDelay),
                  FALL_CONFIRM_DELAY_SEC);
    Serial.println("[ERROR] Power-cycle the sensor and run this test again.");
  } else {
    Serial.printf("[OK] Fall delay saved: %lu sec\n",
                  static_cast<unsigned long>(savedFallDelay));
  }
}

void printSettings() {
  Serial.println();
  Serial.println("========== C1001 FALL TEST ==========");
  Serial.printf("Mode             : %d (1 = fall mode)\n", c1001.getWorkMode());
  Serial.printf("Install height   : %d cm\n", c1001.dmGetInstallHeight());
  Serial.printf("Fall delay       : %d sec\n", c1001.getFallTime());
  Serial.printf("No-person delay  : %d sec\n", c1001.getUnmannedTime());
  Serial.printf("Still dwell      : %d sec\n", c1001.getStaticResidencyTime());
  Serial.printf("Fall sensitivity : %d (0~3)\n",
                c1001.getFallData(c1001.eFallSensitivity));
  Serial.println("-------------------------------------");
  Serial.println("PRESENCE: 0=no person, 1=person");
  Serial.println("MOVEMENT: 0=none, 1=still, 2=active");
  Serial.println("RANGE: 0~100 movement-strength parameter (not centimeters)");
  Serial.println("FALL/STILL: 0=no, 1=yes");
  Serial.println("=====================================");
  Serial.println();
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(1000);

  // begin(baud, format, RX, TX): C1001 TX와 ESP RX를 서로 교차 연결한다.
  Serial1.begin(C1001_BAUD, SERIAL_8N1, C1001_RX_PIN, C1001_TX_PIN);

  waitForSensor();
  setFallMode();
  printSettings();
}

void loop() {
  const int presence = c1001.smHumanData(c1001.eHumanPresence);
  const int movement = c1001.smHumanData(c1001.eHumanMovement);
  const int movementRange = c1001.smHumanData(c1001.eHumanMovingRange);
  // 패치 라이브러리로 실제 UART 성공 여부와 정상적인 '낙상 없음(0)'을 구분한다.
  uint16_t checkedFallState = 0;
  const int fallState =
      c1001.getFallStateChecked(checkedFallState) == 0 &&
              checkedFallState <= 1
          ? static_cast<int>(checkedFallState)
          : -1;
  const int stillState = c1001.getFallData(c1001.estaticResidencyState);

  Serial.printf("PRESENCE=%d  MOVEMENT=%d  RANGE=%d  FALL=%d  STILL=%d\n",
                presence, movement, movementRange, fallState, stillState);

  if (fallState == 1 && previousFallState != 1) {
    Serial.println(">>> FALL DETECTED: C1001 fall state changed to 1 <<<");
  } else if (fallState == 0 && previousFallState == 1) {
    Serial.println(">>> FALL CLEARED: C1001 fall state returned to 0 <<<");
  }

  if (stillState == 1 && previousStillState != 1) {
    Serial.println(">>> STILL DWELL DETECTED: no meaningful movement for the set time <<<");
  } else if (stillState == 0 && previousStillState == 1) {
    Serial.println(">>> STILL DWELL CLEARED <<<");
  }

  if ((presence < 0 || presence > 1) ||
      (movement < 0 || movement > 2) ||
      (fallState < 0 || fallState > 1) ||
      (stillState < 0 || stillState > 1)) {
    Serial.println("[WARN] Invalid response - inspect UART wiring or sensor power");
  }

  previousFallState = fallState;
  previousStillState = stillState;
  delay(SAMPLE_INTERVAL_MS);
}
