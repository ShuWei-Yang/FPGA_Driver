#ifndef __FPGAHANDLER_H
#define __FPGAHANDLER_H

#include "NiFpga.h"
#include "NiFpga_RS485_v2.h"
#include "color.hpp"
#include "msg.hpp"

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
#undef OK

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

    NiFpga_FPGA_POWER_RS485_v2_ControlU32_DCMotor;
    NiFpga_FPGA_POWER_RS485_v2_ControlU32_RSBLServo;
    
    // tx buffer
    NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8_DataTx;
    NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8Size_DataTx; 

    // rx buffer
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRX; 
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRX;

    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus1; 
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus2;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus3;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRXBus1;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRXBus2;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRXBus3;

    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinish;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus1;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus2;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus3;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_ChecksumOK;

    void write_DCMotor_(NiFpga_Bool value);
    void write_RSBL_(NiFpga_Bool value);

    void write_tx_(const uint8_t* tx_arr12);
    void read_rx_main_(uint8_t* rx_arr64);
    void read_rx_bus1_(uint8_t* rx_arr16);
    void read_rx_bus2_(uint8_t* rx_arr16);
    void read_rx_bus3_(uint8_t* rx_arr16);

    NiFpga_Bool read_RXfinish_();
    NiFpga_Bool read_RXfinishBus1_();
    NiFpga_Bool read_RXfinishBus2_();
    NiFpga_Bool read_RXfinishBus3_();
    NiFpga_Bool read_ChecksumOK_();

    void RS485_setup(int cmd_us, int idle_us, int read_us);
    // void RS485_set_mode(Mode mode);

    void RS485_send_dc_command(RS485_txdata txdata);
    void RS485_receive_dc_feedback(RS485_rxdata* rxdata_main);
    void RS485_encode_dc(uint8_t (&txmsg)[8], RS485_txdata txdata);
    void RS485_decode_dc(uint8_t (&rxmsg_main)[8], RS485_rxdata* rxdata_main);

    void RS485_send_servo_command(RS485_txdata txdata_servo);
    void RS485_receive_servo_feedback(RS485_rxdata* rxdata_bus1, RS485_rxdata* rxdata_bus2, RS485_rxdata* rxdata_bus3);
    void RS485_encode_servo(uint8_t (&txmsg_servo)[8], RS485_txdata txdata_servo);
    void RS485_decode_servo(uint8_t (&rxmsg_bus1)[8], (&rxmsg_bus2)[8], (&rxmsg_bus3)[8], RS485_rxdata* rxdata_bus1, RS485_rxdata* rxdata_bus2, RS485_rxdata* rxdata_bus3);
};

class FpgaHandler
{
public:
    FpgaHandler();
    ~FpgaHandler();

    NiFpga_Session session_;
    NiFpga_Status status_;

    NiFpga_FPGA_POWER_RS485_v2_ControlBool_Digital;
    NiFpga_FPGA_POWER_RS485_v2_ControlBool_Signal;
    NiFpga_FPGA_POWER_RS485_v2_ControlBool_Power;

    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16 r_powerboard_data_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16Size size_powerboard_data_;

    void write_powerboard_(std::vector<bool> *powerboard_state_);
    void read_powerboard_data_();

    double powerboard_Ifactor[12];
    double powerboard_Ioffset[12];
    double powerboard_Vfactor[12];
    double powerboard_Voffset[12];

    double powerboard_I_list_[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double powerboard_V_list_[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

#endif

// ok with RSBL