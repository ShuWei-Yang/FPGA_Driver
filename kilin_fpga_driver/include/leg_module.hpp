#ifndef __LEGMODULE_H
#define __LEGMODULE_H

#include <iostream>
#include <vector>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <iomanip>

#include "msg.hpp"
#include "fpga_handler.hpp"

class LegModule
{
public:
    LegModule(std::string _label, 
    YAML::Node _config, 
    NiFpga_Status _status, 
    NiFpga_Session _fpga_session);
    LegModule(){}

    std::string label_;
    YAML::Node config_;
    std::vector<Motor> motors_list_;

    ModuleIO io_;
    std::string RS485_port_;
    bool enable_;

    std::vector<Servo> servos_list_;

    void load_config();
};

#endif


// ok with RSBL