#include "fpga_handler.hpp"
#include <cstring>

ModuleIO::ModuleIO(NiFpga_Status _status,
                   NiFpga_Session _fpga_session,
                   std::string RS485_port_,
                   std::vector<Motor>* motors_list_,
                   std::vector<Servo>* servos_list_){
    status_       = _status;
    fpga_session_ = _fpga_session;
    motors_list_  = motors_list;
    servos_list_ = servo_list;

    DCMotor_ = NiFpga_FPGA_POWER_RS485_v2_ControlU32_DCMotor;
    RSBLServo_ = NiFpga_FPGA_POWER_RS485_v2_ControlU32_RSBLServo;

    DataTx_ = NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8_DataTx;
    DataTx_size_ = NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8Size_DataTx; 

    DataRX_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRX;
    DataRX_Bus1_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus1;
    DataRX_Bus2_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus2;
    DataRX_Bus3_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus3;
    DataRx_size_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRX;   
    DataRx_bus_size_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRXBus1; 

    RXfinish_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinish;
    RXfinishBus1_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus1;
    RXfinishBus2_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus2;
    RXfinishBus3_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus3;
    ChecksumOK_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_ChecksumOK;

    CMDTimeus_  = NiFpga_FPGA_POWER_RS485_v2_ControlU32_CMDTimeus;
    IdleTimeus_ = NiFpga_FPGA_POWER_RS485_v2_ControlU32_IdleTimeus;
    ReadTimeus_ = NiFpga_FPGA_POWER_RS485_v2_ControlU32_ReadTimeus;

}

void ModuleIO::write_DCMotor_(uint32_t cfg){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, DCMotor_, cfg));
}

void ModuleIO::write_RSBL_(uint32_t cfg){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, RSBLServo_, cfg));
}

void ModuleIO::write_CMDTime_us_(uint32_t us){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, CMDTimeus_, us));
}

void ModuleIO::write_IdleTime_us_(uint32_t us){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, IdleTimeus_, us));
}

void ModuleIO::write_ReadTime_us_(uint32_t us){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, ReadTimeus_, us));
}

void ModuleIO::write_tx_(const uint8_t* tx_arr12){
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayU8(fpga_session_, DataTx_, tx_arr12, DataTx_size_));
}

void ModuleIO::read_rx_main_(uint8_t* rx_arr64){
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, DataRX_, rx_arr64, DataRx_size_));
}

void ModuleIO::read_rx_bus1_(uint8_t* rx_arr16){
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, DataRX_Bus1_, rx_arr16, DataRx_bus_size_));
}

void ModuleIO::read_rx_bus2_(uint8_t* rx_arr16){
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, DataRX_Bus2_, rx_arr16, DataRx_bus_size_));
}

void ModuleIO::read_rx_bus3_(uint8_t* rx_arr16){
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, DataRX_Bus3_, rx_arr16, DataRx_bus_size_));
}

NiFpga_Bool ModuleIO::read_RXfinish_(){
    NiFpga_Bool main = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, RXfinish_, &main));
    return main;
}
NiFpga_Bool ModuleIO::read_RXfinishBus1_(){
    NiFpga_Bool bus1 = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, RXfinishBus1_, &bus1));
    return bus1;
}
NiFpga_Bool ModuleIO::read_RXfinishBus2_(){
    NiFpga_Bool bus2 = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, RXfinishBus2_, &bus2));
    return bus2;
}
NiFpga_Bool ModuleIO::read_RXfinishBus3_(){
    NiFpga_Bool bus3 = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, RXfinishBus3_, &bus3));
    return bus3;
}
NiFpga_Bool ModuleIO::read_ChecksumOK_(){
    NiFpga_Bool check = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, ChecksumOK_, &check));
    return check;
}

void ModuleIO::RS485_setup(int cmd_us, int idle_us, int read_us){
    write_CMDTime_us_(cmd_us);
    write_IdleTime_us_(idle_us);
    write_ReadTime_us_(read_us);
}

// void ModuleIO::RS485_set_mode(Mode mode)

void ModuleIO::RS485_send_dc_command(RS485_txdata txdata){
    uint8_t txmsg[8];
    RS485_txdata txdata_biased;
    txdata_biased.position_ = txdata.position_;
    txdata_biased.torque_ = txdata.torque_;
    txdata_biased.KP_ = txdata.KP_;
    txdata_biased.KI_ = txdata.KI_;
    txdata_biased.KD_ = txdata.KD_;
    RS485_encode_dc(txmsg, txdata_biased);
    write_tx_(txmsg);
    write_DCMotor_(1);
}

void ModuleIO::RS485_receive_dc_feedback(RS485_rxdata* rxdata_main){
    uint8_t rxmsg_main[8];
    read_rx_main_(rxmsg_main);
    RS485_decode_dc(rxmsg_main, rxdata_main);
  }

void ModuleIO::RS485_encode_dc(uint8_t (&txmsg)[8], RS485_txdata txdata){
    int pos_int, torque_int, KP_int, KI_int, KD_int;
    pos_int = float_to_uint(-txdata.position_, P_CMD_MIN, P_CMD_MAX, 16);
    KP_int = float_to_uint(txdata.KP_, KP_MIN, KP_MAX, 12);
    KI_int = float_to_uint(txdata.KI_, KI_MIN, KI_MAX, 12);
    KD_int = float_to_uint(txdata.KD_, KD_MIN, KD_MAX, 12);
    torque_int = float_to_uint(txdata.torque_, T_MIN, T_MAX, 12);

    txmsg[0] = pos_int >> 8;
    txmsg[1] = pos_int & 0xFF;
    txmsg[2] = KP_int >> 4;
    txmsg[3] = ((KP_int & 0x0F) << 4) | (KI_int >> 8);
    txmsg[4] = KI_int & 0xFF;
    txmsg[5] = KD_int >> 4;
    txmsg[6] = ((KD_int & 0x0F) << 4) | (torque_int >> 8);
    txmsg[7] = torque_int & 0xFF;
}

void ModuleIO::RS485_decode_dc(uint8_t (&rxmsg_main)[8], RS485_rxdata* rxdata_main){
    int pos_raw, vel_raw, torque_raw, cal_raw, ver_raw, mode_raw;
    pos_raw = ((int)(rxmsg_main[0]) << 8) | rxmsg_main[1];
    vel_raw = ((int)(rxmsg_main[2]) << 8) | rxmsg_main[3];
    torque_raw = ((int)(rxmsg_main[4]) << 8) | rxmsg_main[5];
    cal_raw = ((int)(rxmsg_main[6] & 0x0F));
    mode_raw = ((int)(rxmsg_main[7]& 0x0F));

    rxdata_main->position_ = -uint_to_float(pos_raw, P_FB_MIN, P_FB_MAX, 16);
    rxdata_main->velocity_ = uint_to_float(vel_raw, V_MIN, V_MAX, 16);
    rxdata_main->torque_ = uint_to_float(torque_raw, T_MIN, T_MAX, 12);
    rxdata_main->calibrate_finish_ = cal_raw;
    rxdata_main->mode_state_ = mode_raw;

    if (mode_raw == _SET_ZERO)rxdata_main->mode_ = Mode::SET_ZERO;
    else if (mode_raw == _MOTOR_MODE)rxdata_main->mode_ = Mode::MOTOR;
    else if (mode_raw == _HALL_CALIBRATE)rxdata_main->mode_ = Mode::HALL_CALIBRATE;
    else if (mode_raw == _REST_MODE)rxdata_main->mode_ = Mode::REST;
}

void ModuleIO::RS485_send_servo_command(RS485_txdata txdata_servo){
    uint8_t txmsg_servo[8];
    RS485_encode_servo(txmsg_servo, txdata_servo);
    write_tx_(txmsg_servo);
    write_RSBL_(1);
}

void ModuleIO::RS485_receive_servo_feedback(RS485_rxdata* rxdata_bus1, RS485_rxdata* rxdata_bus2, RS485_rxdata* rxdata_bus3){
    uint8_t rxmsg_bus1[8];
    uint8_t rxmsg_bus2[8];
    uint8_t rxmsg_bus3[8];
    read_rx_bus1_(rxmsg_bus1);
    read_rx_bus2_(rxmsg_bus2);
    read_rx_bus3_(rxmsg_bus3);
    RS485_decode_servo(rxmsg_bus1, rxdata_bus1);
    RS485_decode_servo(rxmsg_bus2, rxdata_bus2);
    RS485_decode_servo(rxmsg_bus3, rxdata_bus3);
}

void ModuleIO::RS485_encode_servo(uint8_t (&txmsg_servo)[8], RS485_txdata txdata_servo){
    txmsg_servo[0];
    txmsg_servo[1];
    txmsg_servo[2];
    txmsg_servo[3];
    txmsg_servo[4];
    txmsg_servo[5];
    txmsg_servo[6];
}

void ModuleIO::RS485_decode_servo(uint8_t (&rxmsg_bus1)[8], uint8_t (&rxmsg_bus2)[8], uint8_t (&rxmsg_bus3)[8], RS485_rxdata* rxdata_bus1, RS485_rxdata* rxdata_bus2, RS485_rxdata* rxdata_bus3){
    rxmsg_bus1[0];
    rxmsg_bus1[1];
    rxmsg_bus1[2];
    rxmsg_bus1[3];
    rxmsg_bus1[4];
    rxmsg_bus1[5];
    rxmsg_bus1[6];
    rxmsg_bus1[7];

    rxmsg_bus2[0];
    rxmsg_bus2[1];
    rxmsg_bus2[2];
    rxmsg_bus2[3];
    rxmsg_bus2[4];
    rxmsg_bus2[5];
    rxmsg_bus2[6];
    rxmsg_bus2[7];

    rxmsg_bus3[0];
    rxmsg_bus3[1];
    rxmsg_bus3[2];
    rxmsg_bus3[3];
    rxmsg_bus3[4];
    rxmsg_bus3[5];
    rxmsg_bus3[6];
    rxmsg_bus3[7];
}

int ModuleIO::float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float ModuleIO::uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

FpgaHandler::~FpgaHandler(){
    NiFpga_MergeStatus(&status_, NiFpga_Close(session_, 0));
    important_message("[FPGA Handler] Session Closed");

    NiFpga_MergeStatus(&status_, NiFpga_Finalize());
    important_message("[FPGA Handler] FPGA Finalized");
}

void FpgaHandler::write_powerboard_(std::vector<bool>* powerboard_state_){
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_, powerboard_state_->at(0)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_,  powerboard_state_->at(1)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_,   powerboard_state_->at(2)));
}

void FpgaHandler::read_powerboard_data_()
{
    uint16_t rx_arr[24];
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU16(session_, NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16_Data, rx_arr, NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16Size_Data));

    for (int i = 0; i < 24; i++)
    {
        if (i % 2 == 0)powerboard_I_list_[i / 2] = rx_arr[i] * powerboard_Ifactor[i / 2] + powerboard_Ioffset[i / 2];
        if (i % 2 == 1)powerboard_V_list_[(i - 1) / 2] = rx_arr[i] * powerboard_Vfactor[(i - 1) / 2] + powerboard_Voffset[(i - 1) / 2];
    }
}

// ok with RSBL
