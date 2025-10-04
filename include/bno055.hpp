#pragma once 
#include <Wire.h>

#define BNO055_ADDRESS 0x28  // ADR=GNDの場合
int16_t heading_raw, roll_raw, pitch_raw;
float heading, roll, pitch;
float heading_offset = 0.0;  // 基準値

void write8(uint8_t reg, uint8_t value);

// ====== 関数 ======
void setMode(uint8_t mode) {
  write8(0x3D, 0x00); // CONFIGMODE
  delay(20);
  write8(0x3D, mode); // NDOF or IMU
  delay(20);
}

void write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readEuler() {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(0x1A); // Heading LSB
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDRESS, 6);

  if (Wire.available() == 6) {
    uint8_t hL = Wire.read();
    uint8_t hH = Wire.read();
    uint8_t rL = Wire.read();
    uint8_t rH = Wire.read();
    uint8_t pL = Wire.read();
    uint8_t pH = Wire.read();

    heading_raw = (int16_t)((hH << 8) | hL);
    roll_raw    = (int16_t)((rH << 8) | rL);
    pitch_raw   = (int16_t)((pH << 8) | pL);

    heading = heading_raw / 16.0;
    roll    = roll_raw / 16.0;
    pitch   = pitch_raw / 16.0;
  }
}
