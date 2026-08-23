#include <Arduino.h>

// 외부 제어부 하드웨어 최종 시험
//
// 문센서 : GPIO4 ↔ 접점 ↔ GND
// SOS    : GPIO5 ↔ NO/COM 접점 ↔ GND
// 릴레이 : IN1 → GPIO17, VCC → 5V, GND → GND
// BF395  : 12V+ → COM, NO → BF395+, BF395- → 12V-

constexpr uint8_t DOOR_PIN = 4;
constexpr uint8_t SOS_PIN = 5;
constexpr uint8_t RELAY_PIN = 17;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t DEBOUNCE_MS = 50;

// 릴레이 CH1 트리거 점퍼를 H에 둔 시험 설정
// 점퍼를 L에 두었다면 HIGH와 LOW를 서로 바꾼다.
constexpr uint8_t RELAY_ON_LEVEL = HIGH;
constexpr uint8_t RELAY_OFF_LEVEL = LOW;

struct DebouncedInput {
  uint8_t pin;
  int rawState;
  int stableState;
  uint32_t changedAt;
};

DebouncedInput door = {DOOR_PIN, HIGH, HIGH, 0};
DebouncedInput sos = {SOS_PIN, HIGH, HIGH, 0};

bool alarmLatched = false;

void beginInput(DebouncedInput& input) {
  pinMode(input.pin, INPUT_PULLUP);
  delay(10);
  input.rawState = digitalRead(input.pin);
  input.stableState = input.rawState;
  input.changedAt = millis();
}

bool updateInput(DebouncedInput& input) {
  const int newRawState = digitalRead(input.pin);

  if (newRawState != input.rawState) {
    input.rawState = newRawState;
    input.changedAt = millis();
  }

  if (millis() - input.changedAt >= DEBOUNCE_MS &&
      input.stableState != input.rawState) {
    input.stableState = input.rawState;
    return true;
  }

  return false;
}

void setAlarm(bool enabled) {
  alarmLatched = enabled;
  digitalWrite(
      RELAY_PIN,
      enabled ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL
  );

  if (enabled) {
    Serial.println("========== 경보 ON / BF395 작동 ==========");
  } else {
    Serial.println("========== 경보 OFF ==========");
  }
}

void printDoorState() {
  if (door.stableState == LOW) {
    Serial.println("[문센서] 문 닫힘");
  } else {
    Serial.println("[문센서] 문 열림");
  }
}

void printStatus() {
  Serial.println("----------------------------------------");
  printDoorState();

  if (sos.stableState == LOW) {
    Serial.println("[SOS] 눌림");
  } else {
    Serial.println("[SOS] 대기");
  }

  Serial.print("[경보] ");
  Serial.println(alarmLatched ? "ON" : "OFF");
}

void handleSerialCommand() {
  while (Serial.available()) {
    const char command = Serial.read();

    switch (command) {
      case '1':
        setAlarm(true);
        break;

      case '0':
        setAlarm(false);
        break;

      case 't':
      case 'T':
        Serial.println("[시험] BF395 1초 작동");
        setAlarm(true);
        delay(1000);
        setAlarm(false);
        break;

      case 's':
      case 'S':
        printStatus();
        break;

      default:
        break;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  // 출력 모드로 바꾸기 전에 OFF 값을 먼저 기록해 부팅 순간 동작을 줄인다.
  digitalWrite(RELAY_PIN, RELAY_OFF_LEVEL);
  pinMode(RELAY_PIN, OUTPUT);

  beginInput(door);
  beginInput(sos);

  setAlarm(false);

  Serial.println();
  Serial.println("=== 외부 제어부 최종 테스트 ===");
  Serial.println("명령: 1=경보 ON, 0=경보 OFF, T=1초 시험, S=상태");
  printStatus();
}

void loop() {
  if (updateInput(door)) {
    printDoorState();
  }

  if (updateInput(sos)) {
    if (sos.stableState == LOW) {
      Serial.println("[SOS] 버튼 눌림");
      setAlarm(true);
    } else {
      Serial.println("[SOS] 버튼 해제 - 경보는 유지");
    }
  }

  handleSerialCommand();
}
