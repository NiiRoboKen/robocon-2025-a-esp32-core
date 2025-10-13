#include <Arduino.h>
#include "robot.hpp"
#include "bno055.hpp"
#include <ESP32Encoder.h>
#include <CAN.h>

//並列処理をします 
//===========================================

TaskHandle_t Task0;
TaskHandle_t Task1;

void Task0Function( void * pvParameters );
void Task1Function( void * pvParameters );

//UART
//===========================================

const int8_t SERIAL1_RX_PIN = 4;
const int8_t SERIAL1_TX_PIN = 5;
const int8_t SERIAL2_RX_PIN = 12;
const int8_t SERIAL2_TX_PIN = 13;

void uart2CommandCheck(uint8_t *uart2_data);

//自己位置推定 
//===========================================

void onTimer();

ESP32Encoder encoder1, encoder2;
const int ENCODER_PIN_A[2] = {35, 33};
const int ENCODER_PIN_B[2] = {34, 32};

//Robot
//===========================================

Robot robot;

//===========================================


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);  

   CAN.setPins(26, 27); //rx, tx
  // start the CAN bus at 250 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Wire.begin();
  // NDOFモードに設定
  setMode(0x0C);

  delay(100);

  // 最初の角度を基準にする
  readEuler();
  heading_offset = heading;  // 現在の角度を保存
  Serial.print("基準角度を設定: ");
  Serial.println(heading_offset);

  //エンコーダー読み取り設定
  encoder1.attachHalfQuad(ENCODER_PIN_A[0], ENCODER_PIN_B[0]);
  encoder1.setCount(0);
  encoder2.attachHalfQuad(ENCODER_PIN_A[1], ENCODER_PIN_B[1]);
  encoder2.setCount(0);

  //コアの処理の決定
  xTaskCreatePinnedToCore(Task0Function, "Task0", 10000, NULL, 1, &Task0, 0);
  xTaskCreatePinnedToCore(Task1Function, "Task1", 10000, NULL, 1, &Task1, 1);
}

//UART1 / UART2 / CAN
void Task0Function( void * pvParameters ) {
  uint8_t uart1_data[256], uart2_data[256];
  uint8_t uart1_len, uart2_len;

  for(;;) {
    //from raspi
    if(Sbtp1::receiveSBTP1(uart1_data,&uart1_len)) {
      Serial.print("Received 1: ");
      for (int i = 0; i < uart1_len; i++) {
        Serial.printf("%02X ", uart1_data[i]);
      }
      Serial.println();
      robot.uart1CommandHandle(uart1_data);
    }
    // from ps4
    if(Sbtp2::receiveSBTP2(uart2_data,&uart2_len)) {
      Serial.print("Received 2: ");
      for (int i = 0; i < uart2_len; i++) {
        Serial.printf("%02X ", uart2_data[i]);
      }
      Serial.println();
     
      robot.uart2CommandHandle(uart2_data);
    }
    
    delay(2);
  }
}

//I2C
void Task1Function( void * pvParameters ) {
  unsigned long prev_time = millis();
  for(;;) {
    if(millis() - prev_time >= 10) {
      prev_time = millis();
      //onTimer();
    }
  }
}

void loop() {}


//自己位置推定
void onTimer() {
  static double x = 0, y = 0;
  //各エンコーダーの折線速度[cm/s]
  //3.0[cm] * エンコーダーの割合 * (2π / dt[s])
  double s1 = 3.0 * (double)encoder1.getCount() / 4096.0 * 2 * PI * 100;
  double s2 = 3.0 * (double)encoder2.getCount() / 4096.0 * 2 * PI * -100;  

  //車体の速度成分[cm/s]
  //取り付け角45度の回転行列適応
  double vx = (s1 - s2) / sqrt(2);
  double vy = (s1 + s2) / sqrt(2);

  encoder1.setCount(0);
  encoder2.setCount(0);

  readEuler();

  // 基準を引いて相対角度に変換
  float relative_heading = heading - heading_offset;

  // -180°～+180° に正規化
  if (relative_heading > 180) relative_heading -= 360;
  if (relative_heading < -180) relative_heading += 360;

  //Serial.print("Heading: "); Serial.print(relative_heading); Serial.println(" °");

  float theta = (float)-relative_heading / 180 * PI;

  //積分[cm](車体角度の回転行列適応)
  x += vx * 0.01 * cos(theta) - vy * 0.01 * sin(theta);
  y += vx * 0.01 * sin(theta) + vy * 0.01 * cos(theta);

  Serial.print("X = ");
  Serial.print(x);
  Serial.print(" Y = ");
  Serial.print(y);
  Serial.print(" Theta = ");
  Serial.print(-relative_heading); 
  Serial.println();
  
}

