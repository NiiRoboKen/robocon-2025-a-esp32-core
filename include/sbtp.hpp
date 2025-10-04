#pragma once 

#include <Arduino.h>

/*
・ UART1 を ラズパイ との通信にする
・ UART2 を ESP32-PS4 との通信にする
*/

const uint8_t SOF = 0x55;
const uint8_t EOF_ = 0xAA;
const uint8_t ESC = 0x5A;

class Sbtp1 {
    public:   
    static bool receiveSBTP1(uint8_t *buffer, uint8_t *len) {
        static enum { WAIT_SOF, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC, WAIT_EOF } state = WAIT_SOF;
        static uint8_t dataLen = 0, index = 0;
        static uint8_t tempBuf[256];
        static bool escFlag = false;

        while (Serial1.available()) {
            uint8_t b = Serial1.read();

            switch (state) {
            case WAIT_SOF:
                if (b == SOF) {
                state = WAIT_LEN;
                }
                break;

            case WAIT_LEN:
                dataLen = b;
                index = 0;
                state = WAIT_PAYLOAD;
                break;

            case WAIT_PAYLOAD:
                if (escFlag) {
                tempBuf[index++] = b ^ 0x42;  // 復元
                escFlag = false;
                } else if (b == ESC) {
                escFlag = true;  // 次のバイトを復元対象にする
                } else {
                tempBuf[index++] = b;
                }
                if (index >= dataLen) {
                state = WAIT_CRC;
                }
                break;

            case WAIT_CRC: {
                uint8_t crc = crc8(tempBuf, dataLen);
                if (crc == b) {
                state = WAIT_EOF;
                } else {
                Serial.println("CRC error!");
                state = WAIT_SOF;
                }
                break;
            }

            case WAIT_EOF:
                if (b == EOF_) {
                memcpy(buffer, tempBuf, dataLen);
                *len = dataLen;
                state = WAIT_SOF;          
                return true;  //受信成功
                } else {
                Serial.println("EOF error!");
                }
                state = WAIT_SOF;
                break;
            }
        }
        return false; // まだフレーム未完成
    }

    protected:
    void sendData(uint8_t data[], uint8_t len) {
        // CRC計算
        uint8_t crc = this->crc8(data, len);

        Serial1.write(SOF);         // SOF
        Serial1.write(len);       // Data Length

        // Payload + エスケープ処理
        for (uint8_t i = 0; i < len; i++) {
            uint8_t d = data[i];
            if (d == 0x55 || d == 0xAA || d == 0x5A) {
            Serial1.write(0x5A);
            Serial1.write(d ^ 0x42);
            } else {
            Serial1.write(d);
            }
        }

        Serial1.write(crc);          // CRC
        Serial1.write(EOF_);         // EOF
    }

    static uint8_t crc8(uint8_t *data, uint8_t len) {
        const uint8_t CRC8_GENERATE_POLYNOMIAL = 0xD5;
        const uint8_t CRC8_INITIAL_VALUE = 0xFF;
        const uint8_t CRC8_FINAL_XOR = 0xFF;

        uint8_t crc = CRC8_INITIAL_VALUE;

        for (uint8_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (crc << 1) ^ CRC8_GENERATE_POLYNOMIAL;
            }
            else {
                crc <<= 1;
            }
            }
        }
        return crc ^ CRC8_FINAL_XOR;
    }
};

class Sbtp2 {
    public:
    static bool receiveSBTP2(uint8_t *buffer, uint8_t *len) {
        static enum { WAIT_SOF, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC, WAIT_EOF } state = WAIT_SOF;
        static uint8_t dataLen = 0, index = 0;
        static uint8_t tempBuf[256];
        static bool escFlag = false;
        

        while (Serial2.available()) {
            uint8_t b = Serial2.read();

            switch (state) {
            case WAIT_SOF:
                if (b == SOF) {
                state = WAIT_LEN;
                }
                break;

            case WAIT_LEN:
                dataLen = b;
                index = 0;
                state = WAIT_PAYLOAD;
                break;

            case WAIT_PAYLOAD:
                if (escFlag) {
                tempBuf[index++] = b ^ 0x42;  // 復元
                escFlag = false;
                } else if (b == ESC) {
                escFlag = true;  // 次のバイトを復元対象にする
                } else {
                tempBuf[index++] = b;
                }
                if (index >= dataLen) {
                state = WAIT_CRC;
                }
                break;

            case WAIT_CRC: {
                uint8_t crc = crc8(tempBuf, dataLen);
                if (crc == b) {
                state = WAIT_EOF;
                } else {
                Serial.println("CRC error!");
                state = WAIT_SOF;
                }
                break;
            }

            case WAIT_EOF:
                if (b == EOF_) {
                memcpy(buffer, tempBuf, dataLen);
                *len = dataLen;
                state = WAIT_SOF;
                return true;  //受信成功
                } else {
                Serial.println("EOF error!");
                }
                state = WAIT_SOF;
                break;
            }
        }
        return false; // まだフレーム未完成
    }

    static void sendData(uint8_t data[], uint8_t len) {
    // CRC計算
        uint8_t crc = crc8(data, len);

        Serial2.write(SOF);         // SOF
        Serial2.write(len);       // Data Length

        // Payload + エスケープ処理
        for (uint8_t i = 0; i < len; i++) {
            uint8_t d = data[i];
            if (d == 0x55 || d == 0xAA || d == 0x5A) {
            Serial2.write(0x5A);
            Serial2.write(d ^ 0x42);
            } else {
            Serial2.write(d);
            }
        }

        Serial2.write(crc);          // CRC
        Serial2.write(EOF_);         // EOF
    }

    private:
    static uint8_t crc8(uint8_t *data, uint8_t len) {
        const uint8_t CRC8_GENERATE_POLYNOMIAL = 0xD5;
        const uint8_t CRC8_INITIAL_VALUE = 0xFF;
        const uint8_t CRC8_FINAL_XOR = 0xFF;

        uint8_t crc = CRC8_INITIAL_VALUE;

        for (uint8_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (crc << 1) ^ CRC8_GENERATE_POLYNOMIAL;
            }
            else {
                crc <<= 1;
            }
            }
        }
        return crc ^ CRC8_FINAL_XOR;
    }
};
