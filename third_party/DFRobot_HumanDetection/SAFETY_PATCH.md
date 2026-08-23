# Safety patch

This package is based on DFRobot_HumanDetection and retains its original MIT
license. One backward-compatible method was added for this project:

`uint8_t getFallStateChecked(uint16_t &state)`

The upstream `getFallData(eFallState)` method returns `0` both when no fall is
detected and when the UART request fails. The added method returns the UART
communication result separately, allowing the ESP32 safety firmware to mark the
C1001 unhealthy instead of silently treating a failed read as "no fall".

Existing upstream methods and examples were not changed.
