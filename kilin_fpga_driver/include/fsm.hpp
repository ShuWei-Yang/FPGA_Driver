#ifndef __FSM_H
#define __FSM_H

#include <math.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <Eigen/Dense>
#include <iterator> 

#include "leg_module.hpp"
#include "mode.hpp"
#include "Motor.pb.h"
#include "Power.pb.h"

#pragma once

enum class Scenario{
    ROBOT,
    SINGLE_MODULE
};

class ModeFsm{
public:
    ModeFsm(std::vector<LegModule> *legs_, 
        std::vector<LegModule> *servos_,
        std::vector<bool> *pb_state_, 
        double *pb_v);
    ModeFsm(){}
    Mode workingMode_;
    Mode prev_workingMode_;

    Scenario scenario_;

    std::vector<LegModule>* legs_;
    std::vector<LegModule>* servos_;
    std::vector<bool>* pb_state_;

    bool hall_calibrated;
    int hall_calibrate_status;
    int impedance_status;

    int measure_offset = 0;
    double dt_ = 0.001; 
    double cal_vel_ = 0.25;
    double cal_tol_ = 0.05;
    double cal_dir_[2];
    double cal_command[2];
    double* powerboard_voltage;

    void runFsm(motor_msg::MotorStateStamped &motor_fb_msg, const motor_msg::MotorCmdStamped &motor_cmd_msg);
    bool switchMode(Mode next_mode);
    void publishMsg(motor_msg::MotorStateStamped &motor_fb_msg);
};


#endif

// ok