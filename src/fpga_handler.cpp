#include <fpga_handler.hpp>

ModuleIO::ModuleIO(NiFpga_Status _status, NiFpga_Session _fpga_session, std::string CAN_port_, std::vector<Motor> *motors_list){

    status_ = _status;
    fpga_session_ = _fpga_session;
    motors_list_ = motors_list;

    CAN_timeout_us_ = 1000;
    if (CAN_port_ == "MOD1CAN0")
    {
        r_CAN_id1_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID1;
        r_CAN_id2_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID2;

        r_CAN_id1_FC_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID1FC;
        r_CAN_id2_FC_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID2FC;

        r_port_select_ = NiFpga_FPGA_RS485_v1_2_ControlArrayBool_Mod1CAN0Select;
        r_port_select_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayBoolSize_Mod1CAN0Select;

        r_tx_buf_id1_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN0ID1TX;
        r_tx_buf_id2_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN0ID2TX;
        r_tx_buf_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_Mod1CAN0ID1TX;

        r_rx_buf_id1_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN0ID1RX;
        r_rx_buf_id2_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN0ID2RX;
        r_rx_buf_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_Mod1CAN0ID1RX;

        r_CAN_transmit_ = NiFpga_FPGA_RS485_v1_2_ControlBool_MOD1CAN0Transmit;
        r_CAN_complete_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0Complete;
        r_CAN_success_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0success;
        r_CAN_complete_counter_ = NiFpga_FPGA_RS485_v1_2_IndicatorI16_Mod1CAN0CompleteCounter;

        r_tx_timeout_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0TXTimeout;
        r_rx_timeout_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0RXTimeout;

        r_timeout_us_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0RXTimeoutus;
    }
    else if (CAN_port_ == "MOD1CAN1")
    {
        r_CAN_id1_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID1;
        r_CAN_id2_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID2;

        r_CAN_id1_FC_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID1FC;
        r_CAN_id2_FC_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID2FC;

        r_port_select_ = NiFpga_FPGA_RS485_v1_2_ControlArrayBool_Mod1CAN1Select;
        r_port_select_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayBoolSize_Mod1CAN1Select;

        r_tx_buf_id1_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN1ID1TX;
        r_tx_buf_id2_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN1ID2TX;
        r_tx_buf_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_Mod1CAN1ID1TX;

        r_rx_buf_id1_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN1ID1RX;
        r_rx_buf_id2_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN1ID2RX;
        r_rx_buf_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_Mod1CAN1ID1RX;

        r_CAN_transmit_ = NiFpga_FPGA_RS485_v1_2_ControlBool_MOD1CAN1Transmit;
        r_CAN_complete_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1Complete;
        r_CAN_success_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1success;
        r_CAN_complete_counter_ = NiFpga_FPGA_RS485_v1_2_IndicatorI16_Mod1CAN1CompleteCounter;

        r_tx_timeout_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1TXTimeout;
        r_rx_timeout_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1RXTimeout;

        r_timeout_us_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1RXTimeoutus;
    }
    else
    {
        std::cout << "[ERROR] CAN_PORT CONFIG ERROR !" << std::endl;
    }
}


// Write FPGA status
void ModuleIO::write_CAN_id_(uint32_t id1, uint32_t id2)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id1_, id1));
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id2_, id2));
}

void ModuleIO::write_CAN_id_fc_(uint32_t id1_fc, uint32_t id2_fc)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id1_FC_, id1_fc));
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id2_FC_, id2_fc));
}

void ModuleIO::read_CAN_id_fc_(uint32_t *fc1, uint32_t *fc2)
{
    NiFpga_MergeStatus(&status_, NiFpga_ReadU32(fpga_session_, r_CAN_id1_FC_, fc1));
    NiFpga_MergeStatus(&status_, NiFpga_ReadU32(fpga_session_, r_CAN_id2_FC_, fc2));
}

void ModuleIO::write_port_select_(const NiFpga_Bool *array)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayBool(fpga_session_, r_port_select_, array, r_port_select_size_));
}
void ModuleIO::write_tx_data_(const uint8_t *tx_arr1, const uint8_t *tx_arr2)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id1_, tx_arr1, r_tx_buf_size_));
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id2_, tx_arr2, r_tx_buf_size_));
}
void ModuleIO::write_CAN_transmit_(NiFpga_Bool value)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(fpga_session_, r_CAN_transmit_, value));
}
void ModuleIO::write_timeout_us_(uint32_t value)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_timeout_us_, value));
}

// Read FPGA status

void ModuleIO::read_rx_data_(uint8_t *rx_arr1, uint8_t *rx_arr2)
{
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_id1_, rx_arr1, r_rx_buf_size_));
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_id2_, rx_arr2, r_rx_buf_size_));
}

NiFpga_Bool ModuleIO::read_CAN_complete_()
{
    NiFpga_Bool complete = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_CAN_complete_, &complete));
    return complete;
}

NiFpga_Bool ModuleIO::read_CAN_success_()
{
    NiFpga_Bool success = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_CAN_success_, &success));
    return success;
}

int16_t ModuleIO::read_CAN_complete_counter_()
{
    int16_t count = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadI16(fpga_session_, r_CAN_complete_counter_, &count));
    return count;
}

NiFpga_Bool ModuleIO::read_tx_timeout_()
{
    NiFpga_Bool id1_timeout = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_tx_timeout_, &id1_timeout));
    return id1_timeout;
}


NiFpga_Bool ModuleIO::read_rx_timeout_()
{
    NiFpga_Bool id1_timeout = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_rx_timeout_, &id1_timeout));
    return id1_timeout;
}

int main(int argc, char* argv[])
{
    //dummy
    return 0;
}