#pragma once

/*　メモ

・stopとresetは全てのstmに送るようになってるからoverrideする
・extented id | command | 送る側のid | 受け取る側のid |
//-----------------//
ID
MY_ID   0x00
ALL_ID  0xFF

STR_ID_ALL  0x0F
STR_ID_1    0x01
STR_ID_2    0x02
STR_ID_3    0x03
STR_ID_4    0x04

ELEVATOR_ID 0x10

RIGHT_ELEVATOR_ID   0x11
LEFT_ELEVATOR_ID    0x12
FRONT_ELEVATOR      0x13

RIGHT_ARM   0x31
LEFT_ARM    0x32
FRONT_ARM   0x33

//-----------------//
command
STOP    0x0000
RESET   0x0001
CONTROL 0x0002
PING    0x0003
PONG    0x0005

RETURN_STR_STATE        0x0006
RETURN_ENCODER_VALUE    0x0007

*/

#include <Arduino.h>
#include <CAN.h>

//親クラス
class CanSender {
    public:
    void send(uint32_t id, uint8_t data[], uint8_t data_size){
        CAN.beginExtendedPacket(id);
        for(int i = 0; i < data_size; i++) {
            CAN.write(data[i]);
        }
        CAN.endPacket();
        return;
    }
    void send(uint32_t id){
        CAN.beginExtendedPacket(id);
        CAN.endPacket();
        return;
    }
    virtual void stop() {
        uint32_t id = set_id(this->STOP, this->MY_ID, this->ALL_ID);
        send(id);
        return;
    }
    virtual void reset() {
        uint32_t id = set_id(this->RESET, this->MY_ID, this->ALL_ID);
        send(id);
        return;
    }
    virtual void ping() {
        uint32_t id = set_id(this->PING, this->MY_ID, this->ALL_ID);
        send(id);
        return;
    }
    protected:
    uint32_t set_id(uint32_t command, uint32_t my_id, uint32_t send_id) {
        uint32_t id = (command << 16) + (my_id << 8) + (send_id);
        return id;
    }
    const uint8_t MY_ID = 0x00;
    const uint8_t ALL_ID = 0xFF;
    
    const uint32_t STOP = 0x0000;
    const uint32_t RESET = 0x0001;
    const uint32_t CONTROL = 0x0002;
    const uint32_t PING = 0x0003;
};

//独立ステアリング
class CanSndividualSteering: public CanSender {
    public:
    void reset() override {
        for(int i = 0; i < 4; i++) {
            uint32_t id = this->set_id(this->RESET, this->MY_ID, this->STR_ID[i]);
            uint8_t data[1] = {8};
            this->send(id, data, 1);
            delay(2);
        }
    }
    void stop() override {
        for(int i = 0; i < 4; i++) {
            uint32_t id = this->set_id(this->STOP, this->MY_ID, this->STR_ID[i]);
            this->send(id);
            delay(2);
        }
    }
    void rotate_duty(uint8_t duty, int16_t degree) {
        if(duty < 0) return;
        uint8_t send_degree = 0;
        uint8_t dir = 0;
        if(degree > 0) {
            send_degree = (uint8_t)degree;
        }
        else {
            send_degree = (uint8_t)(180 + degree);
            dir = 1;
        }
        uint8_t data[5] = {0x01, send_degree, dir, duty, this->is_on};
        for(int i = 0; i < 4; i++) {
            uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->STR_ID[i]);
            this->send(id, data, 5);
            delay(10);
        }        
    }
    void unit_rotate_duty(uint8_t duty, int16_t degree, uint8_t number) {
        if(duty < 0) return;
        uint8_t send_degree = 0;
        uint8_t dir = 0;
        if(degree > 0) {
            send_degree = (uint8_t)degree;
        }
        else {
            send_degree = (uint8_t)(180 + degree);
            dir = 1;
        }
        uint8_t data[5] = {0x01, send_degree, dir, duty, this->is_on};

        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, number);
        this->send(id, data, 5);
        delay(4);
            
    }
    void rotate_robot(bool is_cw, uint8_t duty) {
        uint8_t degree[4];
        uint8_t dir[4];
        if(is_cw) {
            degree[0] = 180 - 135;
            degree[1] = 135;
            degree[2] = 180 - 45;
            degree[3] = 45;
            dir[0] = 1;
            dir[1] = 0;
            dir[2] = 1;
            dir[3] = 0;
        }
        else {
            degree[0] = 45;
            degree[1] = 180 - 45;
            degree[2] = 135;
            degree[3] = 180 - 135;
            dir[0] = 0;
            dir[1] = 1;
            dir[2] = 0;
            dir[3] = 1;
        }
        for(int i = 0; i < 4; i++) {
            uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->STR_ID[i]);
            uint8_t data[5] = {0x01, degree[i], dir[i], duty, this->is_on};
            this->send(id, data, 5);
            delay(10);
        }
    }
    void set_state(uint8_t state) {
        if(state == 0) {
            this->is_on = state;
        }
        else {
            this->is_on = 1;
        }
    }

    private:
    uint8_t is_on = 0;
    const uint8_t STR_ID_ALL = 0x0F;
    const uint8_t STR_ID_1 = 0x01;
    const uint8_t STR_ID_2 = 0x02;
    const uint8_t STR_ID_3 = 0x03;
    const uint8_t STR_ID_4 = 0x04;
    const uint8_t STR_ID[4] = {STR_ID_1 ,STR_ID_2, STR_ID_3, STR_ID_4};
};

//昇降機構
class Elevator: public CanSender {
    public:
    void stop() override {
        uint32_t id = set_id(this->STOP, this->MY_ID, this->ELEVATOR_ID);
        this->send(id);
        return;
    }
    void reset() override {
        uint32_t id = set_id(this->RESET, this->MY_ID, this->ELEVATOR_ID);
        this->send(id);
        return;
    }
    void ping() override {
        uint32_t id = set_id(this->PING, this->MY_ID, this->ELEVATOR_ID);
        this->send(id);
        return;
    }
    void move(uint8_t duty, uint8_t is_up) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->ELEVATOR_ID);
        uint8_t data[2] = {duty, is_up};
        this->send(id, data, 2);
        delay(2);
    }
    void moveRight(uint8_t duty, uint8_t is_up) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->ELEVATOR_RIGHT_ID);
        uint8_t data[2] = {duty, is_up};
        this->send(id, data, 2);
        delay(2);
    }    
    void moveLeft(uint8_t duty, uint8_t is_up) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->ELEVATOR_LEFT_ID);
        uint8_t data[2] = {duty, is_up};
        this->send(id, data, 2);
        delay(2);
    }
    private:
    const uint8_t ELEVATOR_ID = 0x10;
    const uint8_t ELEVATOR_RIGHT_ID = 0x11;
    const uint8_t ELEVATOR_LEFT_ID = 0x12;

    const uint8_t UP = 0x01;
    const uint8_t DOWN = 0x00;
};

//右アーム昇降
class CanRightSideArmElevator: public CanSender {
    public:
    void stop() override {
        uint32_t id = this->set_id(this->STOP, this->MY_ID, this->RIGHT_ELEVATOR_ID);
        this->send(id);
        delay(5);
    }
    void reset() override {
        uint32_t id = this->set_id(this->RESET, this->MY_ID, this->RIGHT_ELEVATOR_ID);
        this->send(id);
        delay(5);
    }
    void move(uint8_t duty, uint8_t is_up) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->RIGHT_ELEVATOR_ID);
        uint8_t data[2] = {duty, is_up};
        this->send(id, data, 2);
        delay(5);
    }
    void move(bool is_up){
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->RIGHT_ELEVATOR_ID);
        uint8_t data[1];
        if(is_up) {
            data[0] = UP;
        }
        else{
            data[0] = DOWN;
        }
        this->send(id, data, 1);
    }

    private:
    const uint8_t RIGHT_ELEVATOR_ID = 0x41;

    const uint8_t UP = 0x01;
    const uint8_t DOWN = 0x00;
};

//左アーム昇降
class CanLeftSideArmElevator: public CanSender {
    public:
    void stop() override {
        uint8_t id = this->set_id(this->STOP, this->MY_ID, this->LEFT_ELEVATOR_ID);
        this->send(id);
        delay(5);
    }
    void reset() override {
        uint8_t id = this->set_id(this->RESET, this->MY_ID, this->LEFT_ELEVATOR_ID);
        this->send(id);
        delay(5);
    }
    void move(uint8_t duty, uint8_t is_up) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->LEFT_ELEVATOR_ID);
        uint8_t data[2] = {duty, is_up};
        this->send(id, data, 2);
        delay(5);
    }
    private:
    const uint8_t LEFT_ELEVATOR_ID = 0x42;
    const uint8_t UP = 0x01;
    const uint8_t DOWN = 0x00;
};

//右アーム動作
class CanRightSideArm : public CanSender {
    public:
    void stop() override {
        uint32_t id = this->set_id(this->STOP, this->MY_ID, this->RIGHT_ARM);
        this->send(id);
        delay(5);
    }
    void reset() override {
        uint32_t id = this->set_id(this->RESET, this->MY_ID, this->RIGHT_ARM);
        this->send(id);
        delay(5);        
    }
    void move(uint8_t is_open, uint8_t is_open_move, uint8_t is_fold, uint8_t is_fold_move){
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->RIGHT_ARM);
        uint8_t data[4] = { is_open, is_open_move, is_fold, is_fold_move};
        this->send(id, data, 4);
        delay(5);
    }
    private:
    const uint8_t RIGHT_ARM = 0x31;
};

//左アーム動作
class CanLeftSideArm : public CanSender {
    public:
    void stop() override {
        uint32_t id = this->set_id(this->STOP, this->MY_ID, this->RIGHT_ARM);
        this->send(id);
        delay(5);
    }
    void reset() override {
        uint32_t id = this->set_id(this->RESET, this->MY_ID, this->RIGHT_ARM);
        this->send(id);
        delay(5);        
    }
    void move(uint8_t is_open, uint8_t is_open_move, uint8_t is_fold, uint8_t is_fold_move){
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->RIGHT_ARM);
        uint8_t data[4] = { is_open, is_open_move, is_fold, is_fold_move};
        this->send(id, data, 4);
        delay(5);
    }
    private:
    const uint8_t RIGHT_ARM = 0x32;
};

//共有アーム
class LinkArm: public CanSender {
    public:
    LinkArm(uint8_t id) {
        this->arm_id = id;
    }
    void stop() override {
        uint32_t id = this->set_id(this->STOP, this->MY_ID, this->arm_id);
        this->send(id);
        delay(3);
    }    
    void reset() override {
        uint32_t id = this->set_id(this->RESET, this->MY_ID, this->arm_id);
        this->send(id);
        delay(3);        
    }
    void start() {
        uint32_t id = this->set_id(this->RESET, this->MY_ID, this->arm_id);
        uint8_t data[1] = {0x5A};
        this->send(id, data, 2);
        delay(3);    
    }
    void setPoseNumber(uint8_t number) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->arm_id);
        uint8_t data[2] = {0x00, number};
        this->send(id, data, 2);
        delay(3);
    }
    void setPoseAngle(int theta1, int theta2) {
        uint8_t deg1, deg2, isNegative1, isNegative2;
        isNegative1 = (theta1 > 0) ? 1 : 0;
        isNegative2 = (theta2 > 0) ? 1 : 0;
        deg1 = (uint8_t)abs(theta1);
        deg2 = (uint8_t)abs(theta2);
        
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->arm_id);
        uint8_t data[5] = {0x01, deg1, isNegative1, deg2, isNegative2};
        this->send(id, data, 5);
        delay(3);
    }
    void setPoseDuty(uint8_t duty1, uint8_t dir1, uint8_t duty2, uint8_t dir2) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->arm_id);
        uint8_t data[5] = {0x02, duty1, dir1, duty2, dir2};
        this->send(id, data, 5);
        delay(3);
    }
    private:
    uint8_t arm_id;
};

//吸引
class Suction: public CanSender {
    public:
    void stop() override {
        uint32_t id = this->set_id(this->STOP, this->MY_ID, this->RIGHT_ID);
        this->send(id);
        delay(3);
        id = this->set_id(this->STOP, this->MY_ID, this->LEFT_ID);
        this->send(id);
        delay(3);
    }
    void reset() override {
        uint32_t id = this->set_id(this->RESET, this->MY_ID, this->RIGHT_ID);
        this->send(id);
        delay(3);
        id = this->set_id(this->RESET, this->MY_ID, this->LEFT_ID);
        this->send(id);
        delay(3);
    }
    void move(uint8_t is_on) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->RIGHT_ID);
        uint8_t data[1] = {is_on};
        this->send(id, data, 1);
        delay(3);
        id = this->set_id(this->CONTROL, this->MY_ID, this->LEFT_ID);
        this->send(id, data, 1);
        delay(3);
    }
    void moveRight(uint8_t is_on) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->RIGHT_ID);
        uint8_t data[1] = {is_on};
        this->send(id, data, 1);
        delay(3);
    }
    void moveLeft(uint8_t is_on) {
        uint32_t id = this->set_id(this->CONTROL, this->MY_ID, this->LEFT_ID);
        uint8_t data[1] = {is_on};
        this->send(id, data, 1);
        delay(3);
    }
    private:
    const uint8_t RIGHT_ID = 0x61;
    const uint8_t LEFT_ID  = 0x62;
};
