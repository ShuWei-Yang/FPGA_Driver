#include "fpga_server.hpp"
#include <curses.h>
#include <iostream>
#include <csignal>

volatile int motor_message_updated = 0;
volatile int fpga_message_updated  = 0;  // power

std::mutex mutex_;

motor_msg::MotorCmdStamped motor_cmd_data;
void motor_data_cb(motor_msg::MotorCmdStamped motor_msg){
    mutex_.lock();
    motor_message_updated = true;
    motor_cmd_data = motor_msg;
    mutex_.unlock();
}

power_msg::PowerCmdStamped power_cmd_data;
void power_data_cb(power_msg::PowerCmdStamped power_msg){
    mutex_.lock();
    fpga_message_updated = true;
    power_cmd_data = power_msg;
    mutex_.unlock();

}

volatile sig_atomic_t sys_stop = 0;
void inthand(int signum){
    sys_stop = 1;
}

bool is_sys_stop(){
    return sys_stop;
}

Kilin::Kilin(){
    //std::vector<LegModule> legs_;
    /* initialize powerboard state (Digital/Signal/Power) */
    digital_switch_ = false;
    signal_switch_ = false;
    power_switch_ = false;

    /* initialize robot state */
    fsm_.hall_calibrated = false;

    powerboard_state_.push_back(digital_switch_); 
    powerboard_state_.push_back(signal_switch_); 
    powerboard_state_.push_back(power_switch_); 

    ModeFsm fsm(&motors_list_, nullptr, &powerboard_state_, fpga_.powerboard_V_list_);
    fsm_ = fsm;
    
    load_config_();
    // legs_.resize(6); 
    console_.init(&fpga_, &motors_list_[0], nullptr, &powerboard_state_, &fsm_, &main_mtx_);
}

void Kilin::load_config_(){ 
    yaml_node_ = YAML::LoadFile(CONFIG_PATH);

    fsm_.dt_ = yaml_node_["MainLoop_period_us"].as<int>() * 0.000001;  // sec
    fsm_.measure_offset = yaml_node_["Measure_offset"].as<int>();
    fsm_.cal_vel_ = yaml_node_["Hall_calibration_vel"].as<double>();
    fsm_.cal_tol_ = yaml_node_["Hall_calibration_tol"].as<double>();

    if (yaml_node_["Scenario"].as<std::string>().compare("SingleModule") == 0)fsm_.scenario_ = Scenario::SINGLE_MODULE;
    else fsm_.scenario_ = Scenario::ROBOT;

    /* initialize leg modules */
    modules_num_ = yaml_node_["Number_of_modules"].as<int>();

    for (int i = 0; i < modules_num_; i++){
        std::string label = yaml_node_["Modules_list"][i].as<std::string>();
        LegModule module(label, yaml_node_, fpga_.status_, fpga_.session_);
        motors_list_.push_back(module);
    }

    YAML::Node Factors = yaml_node_["Powerboard_Scaling_Factor"];
    int idx = 0;
    std::cout << "PowerBoard Scaling Factor" << std::endl;
    for (auto f : Factors){
        fpga_.powerboard_Ifactor[idx] = f["Current_Factor"].as<double>();
        fpga_.powerboard_Ioffset[idx] = f["Current_Offset"].as<double>();
        fpga_.powerboard_Vfactor[idx] = f["Voltage_Factor"].as<double>();
        fpga_.powerboard_Voffset[idx] = f["Voltage_Offset"].as<double>();
        std::cout   << "Index " << idx
                    << " Current Factor: " << fpga_.powerboard_Ifactor[idx]
                    << ", Current Offset: " << fpga_.powerboard_Ioffset[idx]
                    << std::endl
                    << " Voltage Factor: " << fpga_.powerboard_Vfactor[idx]
                    << ", Voltage Offset: " << fpga_.powerboard_Voffset[idx]
                    << std::endl;
        idx++;
    }
}

void Kilin::mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                         core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                         core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                         core::Publisher<motor_msg::MotorStateStamped>& state_pub_){
    fpga_.write_powerboard_(&powerboard_state_);
    fpga_.read_powerboard_data_();

    core::spinOnce();
    mutex_.lock();
    power_msg::PowerStateStamped power_fb_msg;
    motor_msg::MotorStateStamped motor_fb_msg;
    fsm_.runFsm(motor_fb_msg, motor_cmd_data); 
    motor_message_updated = 0;
    HALL_CALIBRATED_ = fsm_.hall_calibrated;
    mutex_.unlock();

    powerboardPack(power_fb_msg);

    mutex_.lock();
    if(power_cmd_data.clean()){
            HALL_CALIBRATED_ = false;
    }

    if (fpga_message_updated){
        powerboard_state_.at(0) = power_cmd_data.digital();
        powerboard_state_.at(1) = power_cmd_data.signal();
        powerboard_state_.at(2) = power_cmd_data.power();

        if (power_cmd_data.robot_mode() == (int)Mode::MOTOR && fsm_.workingMode_ != Mode::MOTOR)fsm_.switchMode(Mode::MOTOR);
        else if (power_cmd_data.robot_mode() == (int)Mode::HALL_CALIBRATE && fsm_.workingMode_ != Mode::HALL_CALIBRATE && fsm_.workingMode_ != Mode::MOTOR)fsm_.switchMode(Mode::HALL_CALIBRATE);
        else if (power_cmd_data.robot_mode() == (int)Mode::SET_ZERO && fsm_.workingMode_ != Mode::SET_ZERO)fsm_.switchMode(Mode::SET_ZERO);
        else if (power_cmd_data.robot_mode() == (int)Mode::CONFIG && fsm_.workingMode_ != Mode::CONFIG)fsm_.switchMode(Mode::CONFIG);
        else if (power_cmd_data.robot_mode() == (int)Mode::REST && fsm_.workingMode_ != Mode::REST)fsm_.switchMode(Mode::REST);
        fpga_message_updated = 0;
    }
    mutex_.unlock();
    state_pub_.publish(motor_fb_msg);
    state_pb_pub_.publish(power_fb_msg);
}

void Kilin::powerboardPack(power_msg::PowerStateStamped& power_dashboard_reply){

    mutex_.lock();
    
    power_dashboard_reply.set_digital(powerboard_state_.at(0));
    power_dashboard_reply.set_signal(powerboard_state_.at(1));
    power_dashboard_reply.set_power(powerboard_state_.at(2));

    if (fsm_.hall_calibrated == true) power_dashboard_reply.set_clean(true);
    else power_dashboard_reply.set_clean(false);

    if (fsm_.workingMode_ == Mode::REST) power_dashboard_reply.set_robot_mode(power_msg::REST_MODE);
    else if (fsm_.workingMode_ == Mode::HALL_CALIBRATE) power_dashboard_reply.set_robot_mode(power_msg::HALL_CALIBRATE);
    else if (fsm_.workingMode_ == Mode::MOTOR) power_dashboard_reply.set_robot_mode(power_msg::MOTOR_MODE);
    else if (fsm_.workingMode_ == Mode::SET_ZERO) power_dashboard_reply.set_robot_mode(power_msg::SET_ZERO);

    power_dashboard_reply.set_v_0(fpga_.powerboard_V_list_[0]);
    power_dashboard_reply.set_i_0(fpga_.powerboard_I_list_[0]);

    power_dashboard_reply.set_v_1(fpga_.powerboard_V_list_[1]);
    power_dashboard_reply.set_i_1(fpga_.powerboard_I_list_[1]);

    power_dashboard_reply.set_v_2(fpga_.powerboard_V_list_[2]);
    power_dashboard_reply.set_i_2(fpga_.powerboard_I_list_[2]);

    power_dashboard_reply.set_v_3(fpga_.powerboard_V_list_[3]);
    power_dashboard_reply.set_i_3(fpga_.powerboard_I_list_[3]);

    power_dashboard_reply.set_v_4(fpga_.powerboard_V_list_[4]);
    power_dashboard_reply.set_i_4(fpga_.powerboard_I_list_[4]);

    power_dashboard_reply.set_v_5(fpga_.powerboard_V_list_[5]);
    power_dashboard_reply.set_i_5(fpga_.powerboard_I_list_[5]);

    power_dashboard_reply.set_v_6(fpga_.powerboard_V_list_[6]);
    power_dashboard_reply.set_i_6(fpga_.powerboard_I_list_[6]);

    mutex_.unlock();
}

int main(int argc, char* argv[]){
    signal(SIGINT, inthand);
    important_message("[FPGA Server] : Launched");
    Kilin kilin;

    /* gRPC topics */
    core::NodeHandler nh;
    core::Publisher<power_msg::PowerStateStamped>& power_pub = nh.advertise<power_msg::PowerStateStamped>("power/state");
    core::Subscriber<power_msg::PowerCmdStamped>& power_sub = nh.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_data_cb);

    core::Publisher<motor_msg::MotorStateStamped>& motor_pub = nh.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Subscriber<motor_msg::MotorCmdStamped>& motor_sub = nh.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_data_cb);
    kilin.mainLoop_(power_sub, power_pub, motor_sub, motor_pub);

    if (NiFpga_IsError(kilin.fpga_.status_))
    std::cout << red << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << kilin.fpga_.status_ << reset << std::endl;
    else{
        endwin();
        important_message("[FPGA Server] : Exit Safely");
    }
    return 0;
}
