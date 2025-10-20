#ifndef __FPGAHANDLER_H
#define __FPGAHANDLER_H

#include "NiFpga.h"
#include "NiFpga_RS485_v2.h"
#include "color.hpp"
#include "msg.hpp"
#include "packet.h"

#include <unistd.h>
#include <iostream>
#include <functional>
#include <signal.h>
#include <dlfcn.h>
#include <vector>
#include <ncurses.h>
#include <curses.h>
#include <iostream>
#include <bitset>
#include <string>
#include <cstdint>
#undef OK

#pragma once

class ModuleIO
{
public:
    ModuleIO(NiFpga_Status  status_,
             NiFpga_Session fpga_session_,
             std::string    RS485_port_,
             std::vector<Motor>* motors_list_,
             std::vector<Servo>* servos_list_);

    ModuleIO(){};

    NiFpga_Status status_;
    NiFpga_Session fpga_session_;
    std::vector<Motor>* motors_list_;
    std::vector<Servo>* servos_list_;

    NiFpga_FPGA_POWER_RS485_v2_ControlU32 DCMotor;
    NiFpga_FPGA_POWER_RS485_v2_ControlU32 RSBLServo;
    
    // tx buffer
    NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8 DataTx_;
    NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8Size DataTx_size_; 

    // rx buffer
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8 DataRx_; 
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size DataRx_size_;

    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8 DataRxBus1_; 
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8 DataRxBus2_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8 DataRxBus3_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size DataRxBus1_size_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size DataRxBus2_size_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size DataRxBus3_size_;

    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool Rxfinish_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool RxfinishBus1_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool RxfinishBus2_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool RxfinishBus3_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool ChecksumOK_;

    NiFpga_FPGA_POWER_RS485_v2_ControlU32 CMDTimeus_;
    NiFpga_FPGA_POWER_RS485_v2_ControlU32 IdleTimeus_;
    NiFpga_FPGA_POWER_RS485_v2_ControlU32 ReadTimeus_;

    void write_DCMotor_(std::uint32_t cfg);
    void write_RSBL_(std::uint32_t cfg);

    void write_tx_(const uint8_t* tx_arr1);
    void read_rx_main_(uint8_t* rx_arr1);
    void read_rx_bus1_(uint8_t* rx_arr1);
    void read_rx_bus2_(uint8_t* rx_arr1);
    void read_rx_bus3_(uint8_t* rx_arr1);

    void write_CMDTime_us_(std::uint32_t us);
    void write_IdleTime_us_(std::uint32_t us);
    void write_ReadTime_us_(std::uint32_t us);

    NiFpga_Bool read_RXfinish_();
    NiFpga_Bool read_RXfinishBus1_();
    NiFpga_Bool read_RXfinishBus2_();
    NiFpga_Bool read_RXfinishBus3_();
    NiFpga_Bool read_ChecksumOK_();

    void RS485_setup(int cmd_us, int idle_us, int read_us);

    void RS485_send_dc_command(TXData txdata);
    void RS485_receive_dc_feedback(RXData* rxdata_main);
    void RS485_encode_dc(uint8_t (&txmsg)[8], TXData txdata);
    void RS485_decode_dc(uint8_t (&rxmsg_main)[8], RXData *rxdata_main);

    void RS485_send_servo_command(TXData txdata_servo);
    void RS485_receive_servo_feedback(RXData *rxdata_bus1, RXData *rxdata_bus2, RXData *rxdata_bus3);
    void RS485_encode_servo(uint8_t (&txmsg_servo)[8], TXData txdata_servo);
    void RS485_decode_servo(uint8_t (&rxmsg_bus1)[8], uint8_t(&rxmsg_bus2)[8], uint8_t(&rxmsg_bus3)[8], RXData* rxdata_bus1, RXData* rxdata_bus2, RXData* rxdata_bus3);

    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);

private:
    uint32_t w_pb_digital_{0};
    uint32_t w_pb_signal_{0};
    uint32_t w_pb_power_{0};
};

class FpgaHandler
{
public:
    FpgaHandler();
    ~FpgaHandler();

    NiFpga_Session session_;
    NiFpga_Status status_;

    NiFpga_FPGA_POWER_RS485_v2_ControlBool w_pb_digital_;
    NiFpga_FPGA_POWER_RS485_v2_ControlBool w_pb_signal_;
    NiFpga_FPGA_POWER_RS485_v2_ControlBool w_pb_power_;

    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16 r_powerboard_data_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16Size size_powerboard_data_;

    void write_powerboard_(std::vector<bool> *powerboard_state_);
    void read_powerboard_data_();

    double powerboard_Ifactor[7];
    double powerboard_Ioffset[7];
    double powerboard_Vfactor[7];
    double powerboard_Voffset[7];

    double powerboard_I_list_[7] = {0, 0, 0, 0, 0, 0, 0};
    double powerboard_V_list_[7] = {0, 0, 0, 0, 0, 0, 0};
};

#endif

// ok with RSBL