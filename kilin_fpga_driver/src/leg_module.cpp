#include <leg_module.hpp>

LegModule::LegModule(std::string _label, 
    YAML::Node _config, 
    NiFpga_Status _status, 
    NiFpga_Session _fpga_session){
    label_  = _label;
    config_ = _config;
    enable_ = false;

    load_config();
    enable_ = false;
    for (int i = 0; i < 6; i++){
        txdata_buffer_[i].position_ = 0;
        txdata_buffer_[i].torque_ = 0;
        txdata_buffer_[i].KP_ = motors_list_[i].kp_;
        txdata_buffer_[i].KI_ = motors_list_[i].ki_;
        txdata_buffer_[i].KD_ = motors_list_[i].kd_;
        txdata_buffer_[i].KT_ = motors_list_[i].kt_;

        rxdata_buffer_[i].mode_ = Mode::REST;
        rxdata_buffer_[i].mode_state_ = _REST_MODE;
        rxdata_buffer_[i].position_ = 0;
        rxdata_buffer_[i].torque_ = 0;
        rxdata_buffer_[i].velocity_ = 0;
        rxdata_buffer_[i].calibrate_finish_ = 0;
    }

    io_ = ModuleIO(_status, _fpga_session, RS485_port_, &motors_list_, &servos_list_);
    
    //RS485_CMD_us_  = config_[label_]["RS485_CMD_us"].as<int>(200);
    //RS485_IDLE_us_ = config_[label_]["RS485_IDLE_us"].as<int>(50);
    //RS485_READ_us_ = config_[label_]["RS485_READ_us"].as<int>(150);
    //io_.RS485_setup(RS485_CMD_us_, RS485_IDLE_us_, RS485_READ_us_);
}

void LegModule::load_config()
{
    Motor motorL1;
    Motor motorL2;
    Motor motorL3;
    Motor motorR1;
    Motor motorR2;
    Motor motorR3;
//    Servo servo_bus1_l;
    // load configuration from yaml file
    std::cout << "[ " << label_ << " Configuration ]" << std::endl;
    enable_ = config_["Motor"][label_]["Enable"].as<int>();
    //motor.fw_version_ = config_[label_]["Motor"]["FW_Version"].as<int>(0);
    //motor.calibration_bias = config_[label_]["Motor"]["Calibration_Bias"].as<double>(0.0);
    motorL1.kp_ = config_["Motor"]["L1"]["KP"].as<double>();
    motorL1.ki_ = config_["Motor"]["L1"]["KI"].as<double>();
    motorL1.kd_ = config_["Motor"]["L1"]["KD"].as<double>();
    motorL1.kt_ = config_["Motor"]["L1"]["KT"].as<double>();
    motorL1.torque_ff_ = config_["Motor"]["L1"]["Torque_Feedforward"].as<double>();
    motorL1.input_voltage_ = config_["Motor"]["L1"]["input_voltage"].as<int>();

    motorL2.kp_ = config_["Motor"]["L2"]["KP"].as<double>();
    motorL2.ki_ = config_["Motor"]["L2"]["KI"].as<double>();
    motorL2.kd_ = config_["Motor"]["L2"]["KD"].as<double>();
    motorL2.kt_ = config_["Motor"]["L2"]["KT"].as<double>();
    motorL2.torque_ff_ = config_["Motor"]["L2"]["Torque_Feedforward"].as<double>();
    motorL2.input_voltage_ = config_["Motor"]["L2"]["input_voltage"].as<int>();

    motorL3.kp_ = config_["Motor"]["L3"]["KP"].as<double>();
    motorL3.ki_ = config_["Motor"]["L3"]["KI"].as<double>();
    motorL3.kd_ = config_["Motor"]["L3"]["KD"].as<double>();
    motorL3.kt_ = config_["Motor"]["L3"]["KT"].as<double>();
    motorL3.torque_ff_ = config_["Motor"]["L3"]["Torque_Feedforward"].as<double>();
    motorL3.input_voltage_ = config_["Motor"]["L3"]["input_voltage"].as<int>();

    motorR1.kp_ = config_["Motor"]["R1"]["KP"].as<double>();
    motorR1.ki_ = config_["Motor"]["R1"]["KI"].as<double>();
    motorR1.kd_ = config_["Motor"]["R1"]["KD"].as<double>();
    motorR1.kt_ = config_["Motor"]["R1"]["KT"].as<double>();
    motorR1.torque_ff_ = config_["Motor"]["R1"]["Torque_Feedforward"].as<double>();
    motorR1.input_voltage_ = config_["Motor"]["R1"]["input_voltage"].as<int>();

    motorR2.kp_ = config_["Motor"]["R2"]["KP"].as<double>();
    motorR2.ki_ = config_["Motor"]["R2"]["KI"].as<double>();
    motorR2.kd_ = config_["Motor"]["R2"]["KD"].as<double>();
    motorR2.kt_ = config_["Motor"]["R2"]["KT"].as<double>();
    motorR2.torque_ff_ = config_["Motor"]["R2"]["Torque_Feedforward"].as<double>();
    motorR2.input_voltage_ = config_["Motor"]["R2"]["input_voltage"].as<int>();

    motorR3.kp_ = config_["Motor"]["R3"]["KP"].as<double>();
    motorR3.ki_ = config_["Motor"]["R3"]["KI"].as<double>();
    motorR3.kd_ = config_["Motor"]["R3"]["KD"].as<double>();
    motorR3.kt_ = config_["Motor"]["R3"]["KT"].as<double>();
    motorR3.torque_ff_ = config_["Motor"]["R3"]["Torque_Feedforward"].as<double>();
    motorR3.input_voltage_ = config_["Motor"]["R3"]["input_voltage"].as<int>();

    motors_list_.push_back(motorL1);
    motors_list_.push_back(motorL2);
    motors_list_.push_back(motorL3);
    motors_list_.push_back(motorR1);
    motors_list_.push_back(motorR2);
    motors_list_.push_back(motorR3);

//    servo_bus1_l.id_ = config_[label_]["Servo"]["Bus 1"].as<int>(1);
//    servo_bus1_l.position_ = config_[label_]["Servo"]["InitPos_L"].as<int>(0);
//    servo_r.id_ = config_[label_]["Servo"]["ID_R"].as<int>(2);
//    servo_r.position_ = config_[label_]["Servo"]["InitPos_R"].as<int>(0);

//    servos_list_.push_back(servo_l);
//    servos_list_.push_back(servo_r);

    std::cout << "MotorL1: " << std::endl;
    //std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motorL1.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motorL1.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motorL1.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motorL1.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motorL1.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Input_Voltage: " << std::setw(13) << motorL1.input_voltage_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

    std::cout << "MotorL2: " << std::endl;
    //std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motorL2.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motorL2.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motorL2.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motorL2.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motorL2.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Input_Voltage: " << std::setw(13) << motorL2.input_voltage_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

    std::cout << "MotorL3: " << std::endl;
    //std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motorL3.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motorL3.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motorL3.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motorL3.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motorL3.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Input_Voltage: " << std::setw(13) << motorL3.input_voltage_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

    std::cout << "MotorR1: " << std::endl;
    //std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motorR1.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motorR1.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motorR1.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motorR1.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motorR1.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Input_Voltage: " << std::setw(13) << motorR1.input_voltage_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

    std::cout << "MotorR2: " << std::endl;
    //std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motorR2.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motorR2.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motorR2.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motorR2.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motorR2.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Input_Voltage: " << std::setw(13) << motorR2.input_voltage_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

    std::cout << "MotorR3: " << std::endl;
    //std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motorR3.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motorR3.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motorR3.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motorR3.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motorR3.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Input_Voltage: " << std::setw(13) << motorR3.input_voltage_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

//    std::cout << "Servo_L: " << std::endl;
//    std::cout << std::setw(14) << "  Servo_ID: " << std::setw(13) << servo_l.id_ << std::endl;
//    std::cout << std::setw(14) << "  Servo_Position: " << std::setw(13) << servo_l.position_ << std::endl;
//    std::cout << "Servo_R: " << std::endl;
//    std::cout << std::setw(14) << "  Servo_ID: " << std::setw(13) << servo_r.id_ << std::endl;
//    std::cout << std::setw(14) << "  Servo_Position: " << std::setw(13) << servo_r.position_ << std::endl;
//    std::cout << std::setw(14) << "---------------------------" << std::endl;
}


// ok with RSBL