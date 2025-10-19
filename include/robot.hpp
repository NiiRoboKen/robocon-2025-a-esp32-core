#pragma once

#include <Arduino.h>
#include "can_sender.hpp"
#include "pid.hpp"
#include "sbtp.hpp"

//独立ステアリング
class SwerveDrive : public Sbtp1 {
    public:
    SwerveDrive() {
        p->x = 0;
        p->y = 0;
        p->deg100 = 0;
    }
    void reset() {
        can->reset();
    }
    void stop() {
        can->stop();
        Serial.println("str");
    }
    //現在の座標をセット
    void setCoordinate(int32_t x, int32_t y, int32_t deg) {
        p->x = (int32_t)(x * 10);
        p->y = (int32_t)(y * 10);
        p->deg100 = (int32_t)(deg * 100);
    }
    //UARTできた目標値を代入
    void setPidTarget(int32_t x, int32_t y, int32_t deg100){
        pid->x->setTarget(x);
        pid->y->setTarget(y);
        pid->theta->setTarget(deg100);
    }
    //PID制御
    void pidControl(bool isPid) {
        if(isPid) {
            can->set_state(1);     
        }          
        else {
            can->set_state(0);
            this->setCoordinate(0,0,0);
            this->setPidTarget(0,0,0);
            return;
        }

        pid->x->update((double)p->x, 10);
        pid->y->update((double)p->y, 10);
        pid->theta->update((double)p->deg100/100, 10);

        double u_x = pid->x->getOutput();
        double u_y = pid->y->getOutput();
        double u_omega = pid->theta->getOutput();

        double theta_rad = p->deg100 / 100 * PI / 180;

        //独ステの位置ベクトルを考慮して各成分を計算
        for(int i = 0; i < 4; i++) {
            double sp_x = P_str[i][0] * cos(theta_rad) - P_str[i][1] * sin(theta_rad);
            double sp_y = P_str[i][0] * sin(theta_rad) + P_str[i][1] * cos(theta_rad);

            double vx = u_x - u_omega * sp_y;
            double vy = u_y + u_omega * sp_x;

            int str_duty = sqrt(vx*vx + vy*vy);
            int str_degree = atan2(vy, vx) * 180 / PI;

            if(str_duty > 50) str_duty = 50;
            can->unit_rotate_duty(str_duty, str_degree, i);
        }
    }

    //ESP-PS4から受信したらこれを優先で実行
    void ps4Control(int duty, int16_t degree, uint8_t state) {
        pidControl(false);
        can->set_state(state);
        can->rotate_duty(duty, degree);
    }
    void ps4ControlRotate(uint8_t is_cw, int8_t duty, uint8_t state) {
        pidControl(false);
        can->set_state(state);
        uint8_t cw = is_cw ? 1 : 0;
        can->rotate_robot(cw, duty);
    }

    //タブレットに送信
    void sendTablet(){
        uint8_t data[14];
        this->serializeCommand(data);
        this->sendData(data,13);
    }

    // int32_t → 4バイト（BE順）
    void int32ToBytesBE(int32_t value, uint8_t *bytes) {
        bytes[0] = (value >> 24) & 0xFF;
        bytes[1] = (value >> 16) & 0xFF;
        bytes[2] = (value >> 8)  & 0xFF;
        bytes[3] = value & 0xFF;
    }

    // 4バイト（BE順）→ int32_t
    int32_t bytesToInt32BE(const uint8_t *bytes) {
        return  ( (int32_t)bytes[0] << 24 ) |
                ( (int32_t)bytes[1] << 16 ) |
                ( (int32_t)bytes[2] << 8  ) |
                ( (int32_t)bytes[3] );
    }

    // 現在地の構造体 → バイト配列
    void serializeCommand(uint8_t *buffer) {
        buffer[0] = this->UART_COMMAND;
        this->int32ToBytesBE(p->x, buffer + 1);
        this->int32ToBytesBE(p->y, buffer + 5);
        this->int32ToBytesBE(p->deg100, buffer + 9);
    }

    // バイト配列 → 現在地の構造体
    void deserializeCommand(const uint8_t *buffer) {
        p->x =  ( (int32_t)buffer[1] << 24 ) |
                ( (int32_t)buffer[2] << 16 ) |
                ( (int32_t)buffer[3] << 8  ) |
                ( (int32_t)buffer[4] );
        p->y =  ( (int32_t)buffer[5] << 24 ) |
                ( (int32_t)buffer[6] << 16 ) |
                ( (int32_t)buffer[7] << 8  ) |
                ( (int32_t)buffer[8] );       
        p->deg100 =  ( (int32_t)buffer[9] << 24 ) |
                ( (int32_t)buffer[10] << 16 ) |
                ( (int32_t)buffer[11] << 8  ) |
                ( (int32_t)buffer[12] );        
    }

    private:
    // mm mm deg * 100
    struct Coordinate {
        int32_t x;
        int32_t y;
        int32_t deg100;
    };
    struct CoordinatePid {
        Pid *x = new Pid(1,1,1);
        Pid *y = new Pid(1,1,1);
        Pid *theta = new Pid(1,1,1);
    };

    Coordinate *p = new Coordinate;
    CoordinatePid *pid = new CoordinatePid;
    const uint8_t UART_COMMAND = 0x10;
    
    const double P_str[4][2] = {{0.547, -0.837},{-0.547, -0.837},{0.547, 0.837},{-0.547, 0.837}};
    CanSndividualSteering *can = new CanSndividualSteering;
};

//ロジャー
class Roger : public Sbtp1{
    public:
    void reset() {
        can->reset();
    }
    void stop() {
        can->stop();
        Serial.println("roger");
    }
    void sendCan(uint8_t duty,uint8_t is_up) {
        can->move(duty, is_up);
    }
    void sendCanRight(uint8_t duty,uint8_t is_up) {
        can->moveRight(duty, is_up);
    }
    void sendCanLeft(uint8_t duty,uint8_t is_up) {
        can->moveLeft(duty, is_up);
    }

    private:
    Elevator *can = new Elevator();
};

//右アーム(昇降含む)
class RightArm : public Sbtp1 {
    public:
    RightArm() {
        this->is_open = 0;
        this->is_open_move = 0;
        this->is_fold = 0;
        this->is_fold_move = 0;

        this->duty = 0;
        this->is_up = 0;
    }
    void reset() {
        can_ope->reset();
        can_lift->reset();
    }
    void stop() {
        can_ope->stop();
        can_lift->stop();
        Serial.println("right arm");
    }
    //傾きのパラメータをセット
    void setInclination(uint8_t is_open_, uint8_t is_move_) {
        this->is_open = is_open_;
        this->is_open_move = is_move_;
    }
    //掴むパラメータをセット
    void setHand(uint8_t is_fold_, uint8_t is_move_) {
        this->is_fold = is_fold_;
        this->is_fold_move = is_move_;
    }
    //CAN送信
    void sendCanArm() {
        can_ope->move(this->is_open, this->is_open_move, this->is_fold, this->is_fold_move);
    }
    void setLiftParameters(uint8_t duty_, uint8_t is_up_) {
        this->duty = duty_;
        this->is_up = is_up_;
    }
    void sendCanLift() {
        can_lift->move(this->duty, this->is_up);
    }
    private:
    CanRightSideArm *can_ope = new CanRightSideArm;
    CanRightSideArmElevator *can_lift = new CanRightSideArmElevator;
    uint8_t is_open, is_open_move, is_fold, is_fold_move;
    uint8_t duty, is_up;
};

//左アーム(昇降含む)
class LeftArm : public Sbtp1 {
    public:
    LeftArm() {
        this->is_open = 0;
        this->is_open_move = 0;
        this->is_fold = 0;
        this->is_fold_move = 0;

        this->duty = 0;
        this->is_up = 0;
    }
    void reset() {
        can_ope->reset();
        can_lift->reset();
    }
    void stop() {
        can_ope->stop();
        can_lift->stop();
        Serial.println("left aem");
    }
    //傾きのパラメータをセット
    void setInclination(uint8_t is_open_, uint8_t is_move_) {
        this->is_open = is_open_;
        this->is_open_move = is_move_;
    }
    //掴むパラメータをセット
    void setHand(uint8_t is_fold_, uint8_t is_move_) {
        this->is_fold = is_fold_;
        this->is_fold_move = is_move_;
    }
    //CAN送信
    void sendCanArm() {
        can_ope->move(this->is_open, this->is_open_move, this->is_fold, this->is_fold_move);
    }
    void setLiftParameters(uint8_t duty_, uint8_t is_up_) {
        this->duty = duty_;
        this->is_up = is_up_;
    }
    void sendCanLift() {
        can_lift->move(this->duty, this->is_up);
    }
    private:
    CanLeftSideArm *can_ope = new CanLeftSideArm;
    CanLeftSideArmElevator *can_lift = new CanLeftSideArmElevator;
    uint8_t is_open, is_open_move, is_fold, is_fold_move;
    uint8_t duty, is_up;
};

//共有アーム
class FrontArm: public Sbtp1 {
    public:
    void reset() {
        right_can->reset();
        left_can->reset();
        suc->reset();
    }
    void stop() {
        right_can->stop();
        left_can->stop();
        suc->stop();
        Serial.println("frot arm");
    }
    void stopRight() {
        right_can->stop();
    }
    void stopLeft() {
        left_can->stop();
    }
    void start() {
        right_can->start();
        left_can->start();
    }
    void startRight() {
        right_can->start();
    }
    void startLeft() {
        left_can->start();
    }
    //各ボックスの番号で制御
    void setNumber(uint8_t number) {
        right_can->setPoseNumber(number);
        left_can->setPoseNumber(number);
    }
    void setNumberRight(uint8_t number) {
        right_can->setPoseNumber(number);
    }
    void setNumberLeft(uint8_t number) {
        left_can->setPoseNumber(number);
    }
    //角度で制御(degree)
    void setAngle(int theta1, int theta2) {
        right_can->setPoseAngle(theta1, theta2);
        left_can->setPoseAngle(theta1, theta2);
    }
    void setAngleRight(int theta1, int theta2) {
        right_can->setPoseAngle(theta1, theta2);
    }
    void setAngleLeft(int theta1, int theta2) {
        left_can->setPoseAngle(theta1, theta2);
    }
    //dutyで制御
    void setDuty(uint8_t duty1, uint8_t dir1, uint8_t duty2, uint8_t dir2) {
        right_can->setPoseDuty(duty1, dir1, duty2, dir2);
        left_can->setPoseDuty(duty1, dir1, duty2, dir2);
    }
    void setDutyRight(uint8_t duty1, uint8_t dir1, uint8_t duty2, uint8_t dir2) {
        right_can->setPoseDuty(duty1, dir1, duty2, dir2);
    }
    void setDutyLeft(uint8_t duty1, uint8_t dir1, uint8_t duty2, uint8_t dir2) {
        left_can->setPoseDuty(duty1, dir1, duty2, dir2);
    }

    void suction(uint8_t is_on) {
        suc->move(is_on);
    }
    void suctionRight(uint8_t is_on) {
        suc->moveRight(is_on);
    }
    void suctionLeft(uint8_t is_on) {
        suc->moveLeft(is_on);
    }

    private:
    LinkArm* right_can = new LinkArm(0x51);
    LinkArm* left_can  = new LinkArm(0x52);
    Suction* suc   = new Suction();
};

//ロボット全体
class Robot {
    public:
    void init() {
        str->reset();
    }
    void pong() {
        uint8_t send_data[1] = {0x03};
        sbtp->sendData(send_data, 1);
    }
    void stop() {
        str->ps4Control(0,90,0);
        roger->sendCan(0,0);
        right_arm->setInclination(0, 0);
        right_arm->setHand(0,0);
        right_arm->sendCanArm();
        left_arm->setInclination(0, 0);
        left_arm->setHand(0,0);
        left_arm->sendCanArm();
        front_arm->setDutyRight(0,0,0,0);
    }
    void uart1CommandHandle(uint8_t *data){
        switch (data[0])
        {
            case 0x01:
                this->pong();
                break;
            case 0x0A:
                this->stop();
                break;

            //sideアーム展開(All)
            case 0x30:
                right_arm->setInclination(1, 1);
                right_arm->setHand(0,0);
                right_arm->sendCanArm();
                left_arm->setInclination(1, 1);
                left_arm->setHand(0,0);
                left_arm->sendCanArm();
                break;
            //sideアーム展開(Right)
            case 0x31:
                right_arm->setInclination(1, 1);
                right_arm->setHand(0,0);
                right_arm->sendCanArm();
                break;
            //sideアーム展開(Left)
            case 0x32:
                left_arm->setInclination(1, 1);
                left_arm->setHand(0,0);
                left_arm->sendCanArm();
                break;

            //アーム開く(最大)
            case 0x33:
                right_arm->setInclination(0, 0);
                right_arm->setHand(0,1);
                right_arm->sendCanArm();
                left_arm->setInclination(0, 0);
                left_arm->setHand(0,1);
                left_arm->sendCanArm();
                break;

            //sideアーム収納(All)
            case 0x34:
                right_arm->setInclination(0, 1);
                right_arm->setHand(0,0);
                right_arm->sendCanArm();
                left_arm->setInclination(0, 1);
                left_arm->setHand(0,0);
                left_arm->sendCanArm();
                break;
            //sideアーム収納(Right)
            case 0x35:
                right_arm->setInclination(0, 1);
                right_arm->setHand(0,0);
                right_arm->sendCanArm();
                break;
            //sideアーム収納(Left)
            case 0x36:
                left_arm->setInclination(0, 1);
                left_arm->setHand(0,0);
                left_arm->sendCanArm();
                break;

            //共有アームストップ(All)
            case 0x50:
                //front_arm->stop();
                front_arm->setDutyRight(0,0,0,0);
                break;
            //共有アームストップ(Right)
            case 0x51:
                //front_arm->stopRight();
                front_arm->setDutyRight(0,0,0,0);
                break;
            //共有アームストップ(Left)
            case 0x52:
                //front_arm->stopLeft();
                front_arm->setDutyLeft(0,0,0,0);
                break;

            //共有アーム天井上げる
            case 0x53:
                front_arm->setAngle(45,45);
                break;
                
            //共有アーム回収モード
            case 0x54:
                front_arm->setAngle(-45,-45);
                break;
            //共有アーム畳む 上(All)
            case 0x55:
                front_arm->setAngle(170,0);
                break;
            //共有アーム畳む 前(All)
            case 0x56:
                front_arm->setAngle(-65,0);
                break;
            //共有アーム畳む 前(Right)
            case 0x57:
                front_arm->setAngleRight(-65,0);
                break;
            //共有アーム畳む 前(Left)
            case 0x58:
                front_arm->setAngleLeft(-65,0);
                break;

            //共有アームスタート(All)
            case 0x5A:
                front_arm->start();
                break;
            //共有アームスタート(Right)
            case 0x5B:
                front_arm->startRight();
                break;
            //共有アームスタート(Left)
            case 0x5C:
                front_arm->startLeft();
                break;
            
            //吸引(All)
            case 0x60:
                front_arm->suction(data[1]);
                break;
            case 0x61:
                front_arm->suction(data[1]);
                break;
            case 0x62:
                front_arm->suction(data[1]);
                break;

            default:
                break;
        }
    }
    void uart2CommandHandle(uint8_t *data){
        switch (data[0])
        {
            //独ステ
            case 0x10:
                str->reset();
                break;
            case 0x11:
                str->ps4Control(data[1], (int16_t)(data[2] << 8 | data[3]), data[4]);
                break;
            case 0x12:
                str->ps4ControlRotate(data[1], data[2], data[3]);
                break;
            //ロジャー
            case 0x20:
                roger->sendCan(data[1],data[2]);
                break;
            case 0x21:
                roger->sendCanRight(data[1],data[2]);
                break;
            case 0x22:
                roger->sendCanLeft(data[1], data[2]);
                break;
            //右アーム動作
            case 0x31:
                right_arm->setInclination(data[1], data[2]);
                right_arm->setHand(data[3], data[4]);
                right_arm->sendCanArm();
                break;
            //左アーム動作
            case 0x32:
                left_arm->setInclination(data[1], data[2]);
                left_arm->setHand(data[3], data[4]);
                left_arm->sendCanArm();
                break;  
            //右アーム昇降
            case 0x41:
                right_arm->setLiftParameters(data[1],data[2]);
                right_arm->sendCanLift();
                break;         
            //左アーム昇降
            case 0x42:
                left_arm->setLiftParameters(data[1],data[2]);
                left_arm->sendCanLift();
                break;        
            //共有アーム
            case 0x51:
                front_arm->setDutyRight(data[1],data[2],data[3],data[4]);
                break;
            case 0x52:
                front_arm->setDutyLeft(data[1],data[2],data[3],data[4]);
                break;
            //吸引
            case 0x60:
                front_arm->suction(data[1]);
                break;       
            case 0x61:
                front_arm->suctionRight(data[1]);
                break;
            case 0x62:
                front_arm->suctionLeft(data[1]);
                break;        
            default:
                break;
        }
    }
    private:
    Sbtp1       *sbtp      = new Sbtp1();
    SwerveDrive *str       = new SwerveDrive();
    Roger       *roger     = new Roger();
    RightArm    *right_arm = new RightArm();
    LeftArm     *left_arm  = new LeftArm();
    FrontArm    *front_arm = new FrontArm();
};