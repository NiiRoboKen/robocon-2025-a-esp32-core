#include <Arduino.h>

const int8_t SERIAL1_RX_PIN = 4;
const int8_t SERIAL1_TX_PIN = 5;
const int8_t SERIAL2_RX_PIN = 12;
const int8_t SERIAL2_TX_PIN = 13;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
}

void loop() {

}
