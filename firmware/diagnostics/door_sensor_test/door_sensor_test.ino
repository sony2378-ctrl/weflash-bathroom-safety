#define DOOR_SENSOR_PIN 4

const unsigned long DEBOUNCE_TIME = 50;

int lastRawState;
int stableState;

unsigned long lastChangeTime = 0;

void printDoorState(int state) {
  if (state == LOW) {
    Serial.println("문 닫힘");
  } else {
    Serial.println("문 열림");
  }
}

void setup() {
  Serial.begin(115200);

  // 문센서: GPIO4 ↔ 접점 ↔ GND
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);

  delay(100);

  lastRawState = digitalRead(DOOR_SENSOR_PIN);
  stableState = lastRawState;

  Serial.println("=== 문센서 테스트 ===");
  printDoorState(stableState);
}

void loop() {
  int rawState = digitalRead(DOOR_SENSOR_PIN);

  if (rawState != lastRawState) {
    lastRawState = rawState;
    lastChangeTime = millis();
  }

  if (millis() - lastChangeTime >= DEBOUNCE_TIME) {
    if (stableState != rawState) {
      stableState = rawState;
      printDoorState(stableState);
    }
  }
}
