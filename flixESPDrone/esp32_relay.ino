// ESP8266 relay: forward Qt commands to ESP32 over UART

#define ESP32_UART_BAUDRATE 115200
#define ESP32_RX_PIN 9
#define ESP32_TX_PIN 10

void setupESP32Relay() {
    Serial1.begin(ESP32_UART_BAUDRATE, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);
    print("ESP32 Relay UART initialized\n");
}

void sendCommandToESP32(const uint8_t *buf, int len) {
    if (Serial1) {
        Serial1.write(buf, len);
        print(">> Relay to ESP32: ");
        for (int i = 0; i < len; i++) {
            if (buf[i] >= 32 && buf[i] <= 126) {
                Serial.write(buf[i]);
            }
        }
        print("\n");
    }
}

void receiveFromESP32() {
    while (Serial1.available()) {
        uint8_t data = Serial1.read();
        Serial.write(data);
    }
}
