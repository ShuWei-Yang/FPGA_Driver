#ifndef __LEGMODULE_H
#define __LEGMODULE_H

#include <iostream>
#include <vector>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <array>
#include <cstddef>

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
    bool enable_{false};

    std::vector<Servo> servos_list_;
    int RS485_CMD_us_{200};
    int RS485_IDLE_us_{50};
    int RS485_READ_us_{150};

    void load_config();
    //TXData& tx(std::size_t idx) { return txdata_buffer_.at(idx); }
    //const TXData& tx(std::size_t idx) const { return txdata_buffer_.at(idx); }
    //RXData& rx(std::size_t idx) { return rxdata_buffer_.at(idx); }
    //const RXData& rx(std::size_t idx) const { return rxdata_buffer_.at(idx); }
//private:
    TXData txdata_buffer_[6];
    RXData rxdata_buffer_[6];
};

#endif


// ok with RSBL