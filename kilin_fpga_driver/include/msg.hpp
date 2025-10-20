#ifndef MSG_H_
#define MSG_H_

#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include "mode.hpp"

typedef struct Motor{
    std::uint8_t enable_;
    double kp_;
    double ki_;
    double kd_;
    double kt_;
    double torque_ff_;
    double input_voltage_;
}Motor;

typedef struct Servo{
    int id_;
    int position_;
}Servo;

typedef struct TXData{
    float position_;
    float torque_; 
    float KP_;
    float KI_;
    float KD_;
    float KT_;
}TXData;

typedef struct RXData{
    float position_;   
    float velocity_;   
    float torque_;          
    int calibrate_finish_; 
    int mode_state_;   
    Mode mode_;    
}RXData;

class Module{
public:
    std::vector<TXData> txdata;
};

#endif 

// ok with RSBL
