#include <fsm.hpp>

ModeFsm::ModeFsm(std::vector<LegModule>* legs_,
                 std::vector<LegModule>* servos_,
                 std::vector<bool>* pb_state,
                 double* pb_v){
    workingMode_ = Mode::REST;
    prev_workingMode_= Mode::REST;
    legs_list_ = legs_;
    servos_list_ = servos_;
    pb_state_ = pb_state;
    powerboard_voltage= pb_v;

    hall_calibrated = false;
    hall_calibrate_status = 0;
}

void ModeFsm::runFsm(motor_msg::MotorStateStamped& motor_fb_msg,
                     const motor_msg::MotorCmdStamped& motor_cmd_msg){
    switch (workingMode_){
        case Mode::REST:{
            if (pb_state_->at(2) == true){
                publishMsg(motor_fb_msg);
                for (auto& mod : *legs_list_){
                    int index = 0;
                    if (mod.enable_){
                        mod.txdata_buffer_[0].position_ = 0;
                        mod.txdata_buffer_[0].torque_ = 0;
                        mod.txdata_buffer_[0].KP_ = 0;
                        mod.txdata_buffer_[0].KI_ = 0;
                        mod.txdata_buffer_[0].KD_ = 0;
                    }
                }
            }
        }
        break;

        case Mode::SET_ZERO:{
            if (pb_state_->at(2) == true){
                if (legs_list_->at(0).enable_){
                    legs_list_->at(0).io_.Motor_bias = 0;
                }
            }
            publishMsg(motor_fb_msg);
            for (auto& mod : *legs_list_){
                if (mod.enable_){
                    mod.txdata_buffer_[0].position_ = P_CMD_MAX;
                    mod.txdata_buffer_[0].torque_ = 0;
                    mod.txdata_buffer_[0].KP_ = 0;
                    mod.txdata_buffer_[0].KI_ = 0;
                    mod.txdata_buffer_[0].KD_ = 0;
                }
            }
        }
        break;

        case Mode::HALL_CALIBRATE:{
            int legs_enabled = 0;
            if (legs_list_->at(0).enable_){
                legs_list_->at(0).txdata_buffer_[0].position_ = 0;
                legs_list_->at(0).txdata_buffer_[0].torque_ = 0;
                legs_list_->at(0).txdata_buffer_[0].KP_ = 0;
                legs_list_->at(0).txdata_buffer_[0].KI_ = 0;
                legs_list_->at(0).txdata_buffer_[0].KD_ = 0;
            }
            switch(hall_calibrate_status){
                case -1:{
                    switchMode(Mode::REST);
                }
                break;

                case 0:{
                    int cal_cnt = 0
                    if (legs_list_->at(0).enable_ 
                    && legs_list_->at(0).rxdata_buffer_[0].calibrate_finish_ == 2)cal_cnt++;
                    if (cal_cnt == legs_enabled && measure_offset == 0) hall_calibrate_status++;
                    else if (cal_cnt == legs_enabled && measure_offset == 1) hall_calibrate_status = -1;
                }
                break;

                case 1:{
                    if (modules_list_->at(0).enable_){
                        legs_list_->at(0).io_.Motor_bias = legs_list_->at(0).Motor_bias;
                        cal_command[0][0] = - modules_list_->at(0).Motor_bias;
                        legs_list_->at(0).txdata_buffer_[0].position_ = - modules_list_->at(0).Motor_bias;
                        cal_dir_[0][0] = 1;
                    }
                    hall_calibrate_status++;
                }
                break;
                
                case 2:{
                    int finish_cnt = 0;
                    double errj = 0;
                    errj = cal_command[0][0];
                    if (fabs(errj) < cal_tol_){
                        modules_list_->at(0).txdata_buffer_[0].position_ = 0;
                        modules_list_->at(0).txdata_buffer_[0].torque_ = 0;
                        modules_list_->at(0).txdata_buffer_[0].KP_ = 0;
                        modules_list_->at(0).txdata_buffer_[0].KI_ = 0;
                        modules_list_->at(0).txdata_buffer_[0].KD_ = 0;
                        finish_cnt++;
                   }
                    else{
                        cal_command[0][0] += cal_dir_[0][0] * cal_vel_ * dt_;
                        modules_list_->at(0).txdata_buffer_[0].position_ = cal_command[i][0];
                        modules_list_->at(0).txdata_buffer_[0].torque_ = 0;
                        modules_list_->at(0).txdata_buffer_[0].KP_ = 50;
                        modules_list_->at(0).txdata_buffer_[0].KI_ = 0;
                        modules_list_->at(0).txdata_buffer_[0].KD_ = 1.5;        
                    }
                    if (finish_cnt == 2 * legs_enabled) hall_calibrate_status++;
                }
                break;

                case 3:{
                    hall_calibrated = true;
                    hall_calibrate_status = 0;
                    switchMode(Mode::MOTOR);
                }
                break;
            }

        }
        break;

        case Mode::MOTOR:{
            publishMsg(motor_fb_msg);
            int index = 0;
            for (auto& mod : *legs_list_){
                mod.txdata_buffer_[0].position_ = motor_cmd_msg.position();
                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.torque();
                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.kp();
                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.ki();
                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.kd();
            
            }
            break;
            index++;
        }
        break;

        case Mode::CONFIG:{
            // for debug
        }
        break;
    }
}
bool ModeFsm::switchMode(Mode next_mode){
    prev_workingMode_ = workingMode_;
    workingMode_ = next_mode;
    return true;
}   

void ModeFsm::publishMsg (motor_msg::MotorStateStamped& motor_fb_msg){
    int index = 0;
    const auto& rx = legs_list_->at(0).rxdata_buffer_[0];
    const auto& tx = legs_list_->at(0).txdata_buffer_[0];
    motor_fb_msg.set_position(rx.position_);
    motor_fb_msg.set_velocity(rx.velocity_);
    motor_fb_msg.set_torque(rx.torque_ * tx.KT_);
}

// ok