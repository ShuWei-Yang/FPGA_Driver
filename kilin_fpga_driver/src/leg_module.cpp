#include <leg_module.hpp>

LegModule::LegModule(std::string _label, 
    YAML::Node _config, 
    NiFpga_Status _status, 
    NiFpga_Session _fpga_session)
{
    label_  = _label;
    config_ = _config;
    enable_ = false;

    load_config();

    for(size_t i = 0; i < 5; ++i){
        txdata_buffer_[i].position_ = 0;
        txdata_buffer_[i].torque_ = 0;
        txdata_buffer_[i].KP_ = motors_list_[0].kp_;
        txdata_buffer_[i].KI_ = motors_list_[0].ki_;
        txdata_buffer_[i].KD_ = motors_list_[0].kd_;
        txdata_buffer_[i].KT_ = motors_list_[0].kt_;

        rxdata_buffer_[i].mode_ = Mode::REST;
        rxdata_buffer_[i].mode_state_ = _REST_MODE;
        rxdata_buffer_[i].position_ = 0;
        rxdata_buffer_[i].torque_ = 0;
        rxdata_buffer_[i].velocity_ = 0;
        rxdata_buffer_[i].calibrate_finish_ = 0;
    }

    io_ = ModuleIO(_status, _fpga_session, RS485_port_, &motors_list_);
    io_.RS485_setup(
          yaml_i(config_[label_], "RS485_CMD_us",  200),
          yaml_i(config_[label_], "RS485_IDLE_us", 200),
          yaml_i(config_[label_], "RS485_READ_us", 600)
    );
}

void LegModule::load_config()
{
    Motor motor;
    RSBL servo_l;
    RSBL servo_r;
    // load configuration from yaml file
    std::cout << "[ " << label_ << " Configuration ]" << std::endl;
    enable_ = config_[label_]["Enable"].as<int>();

    motor.fw_version_ = config_[label_]["Motor"]["FW_Version"].as<int>();
    motor.kp_ = config_[label_]["Motor"]["KP"].as<double>();
    motor.ki_ = config_[label_]["Motor"]["KI"].as<double>();
    motor.kd_ = config_[label_]["Motor"]["KD"].as<double>();
    motor.kt_ = config_[label_]["Motor"]["KT"].as<double>();
    motor.torque_ff_ = config_[label_]["Motor"]["Torque_Feedfoward"].as<double>();
    motor.input_voltage_ = config_[label_]["Motor"]["Input_Voltage"].as<int>();

    Motor_bias = config_[label_]["Motor"]["Calibration_Bias"].as<double>();
    motor.calibration_bias = Motor_bias;

    motors_list_.push_back(motor);

    servo_l.id_ = config_[label_]["Servo"]["ID"].as<int>();
    servo_l.position_ = config_[label_]["Servo"]["Position"].as<int>();
    servo_r.id_ = config_[label_]["Servo"]["ID"].as<int>();
    servo_r.position_ = config_[label_]["Servo"]["Position"].as<int>();
    servos_list_.push_back(servo_l);
    servos_list_.push_back(servo_r);

    std::cout << "Motor: " << std::endl;
    std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motor.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motor.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motor.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motor.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motor.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Input_Voltage: " << std::setw(13) << motor.input_voltage_ << std::endl;
    std::cout << std::setw(14) << "  Bias: " << std::setw(13) << Motor_bias << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

    std::cout << "Servo_L: " << std::endl;
    std::cout << std::setw(14) << "  Servo_ID: " << std::setw(13) << servo_l.id_ << std::endl;
    std::cout << std::setw(14) << "  Servo_Position: " << std::setw(13) << servo_l.position_ << std::endl;
    std::cout << "Servo_R: " << std::endl;
    std::cout << std::setw(14) << "  Servo_ID: " << std::setw(13) << servo_r.id_ << std::endl;
    std::cout << std::setw(14) << "  Servo_Position: " << std::setw(13) << servo_r.position_ << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;
}


// ok with RSBL