// LD2410C UART 단독 테스트
// ESP32-S3
// LD TX -> GPIO16 (ESP RX)
// LD RX -> GPIO17 (ESP TX)

#define LD_RX 16
#define LD_TX 17

HardwareSerial LD(1);

void setup() {
  Serial.begin(115200);
  delay(1000);

  LD.begin(256000, SERIAL_8N1, LD_RX, LD_TX);

  Serial.println("=== LD2410C UART TEST ===");
  Serial.println("TX(sensor) -> GPIO16");
  Serial.println("RX(sensor) -> GPIO17");
}

void loop() {
  while (LD.available()) {
    uint8_t b = LD.read();

    if (b < 0x10) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
  }

  delay(10);
}
