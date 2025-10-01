#include <Arduino.h>
#include "robot.hpp"

//並列処理をします
TaskHandle_t Task0;
TaskHandle_t Task1;

void Task0Function( void * pvParameters );
void Task1Function( void * pvParameters );

const int8_t SERIAL1_RX_PIN = 4;
const int8_t SERIAL1_TX_PIN = 5;
const int8_t SERIAL2_RX_PIN = 12;
const int8_t SERIAL2_TX_PIN = 13;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);  

  //コアの処理の決定
  xTaskCreatePinnedToCore(Task0Function, "Task0", 10000, NULL, 1, &Task0, 0);
  xTaskCreatePinnedToCore(Task1Function, "Task1", 10000, NULL, 1, &Task0, 1);
}

void Task0Function( void * pvParameters ) {}

void Task1Function( void * pvParameters ) {}

void loop() {}
