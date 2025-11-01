#include <fsm.hpp>
#include <vector>
#include <iterator>


ModeFsm::ModeFsm(std::vector<LegModule>* _legs,
        std::vector<LegModule>* _servos,
        std::vector<bool>* _pb_state,
        double *pb_v){
    workingMode_ = Mode::REST;
    prev_workingMode_= Mode::REST;
    legs_ = _legs;
    servos_ = _servos;
    pb_state_ = _pb_state;
    powerboard_voltage= pb_v;

    hall_calibrated = false;
    hall_calibrate_status = 0;
}

void ModeFsm::runFsm(motor_msg::MotorStateStamped& motor_fb_msg,
                     const motor_msg::MotorCmdStamped& motor_cmd_msg){
    // auto& mod = legs_->at(0);
    switch (workingMode_){
        case Mode::REST:{
            publishMsg(motor_fb_msg);
            if (pb_state_->at(2) == true){
                for (auto& mod : *legs_){
                    if (mod.enable_){
                        for (int i = 0; i < 6; i++){
                            mod.txdata_buffer_[i].position_ = 0;
                            mod.txdata_buffer_[i].torque_ = 0;
                            mod.txdata_buffer_[i].KP_ = 0;
                            mod.txdata_buffer_[i].KI_ = 0;
                            mod.txdata_buffer_[i].KD_ = 0;
                        }
                    }
                }
            }
        }
        break;

        case Mode::SET_ZERO:{
            publishMsg(motor_fb_msg);
            if (pb_state_->at(2) == true){
                for (auto& mod : *legs_){
                    if (mod.enable_){
                        for (int i = 0; i < 6; i++){
                            mod.txdata_buffer_[i].position_ = P_CMD_MAX;
                            mod.txdata_buffer_[i].torque_ = 0;
                            mod.txdata_buffer_[i].KP_ = 0;
                            mod.txdata_buffer_[i].KI_ = 0;
                            mod.txdata_buffer_[i].KD_ = 0;
                        }
                    }
                }
            }
            
        }
        break;

        case Mode::HALL_CALIBRATE:{
            int legs_enabled = 0;
            for (auto& mod : *legs_){
                if (mod.enable_){
                    for (int i = 0; i < 6; i++){
                        mod.txdata_buffer_[i].position_ = 0;
                        mod.txdata_buffer_[i].torque_ = 0;
                        mod.txdata_buffer_[i].KP_ = 0;
                        mod.txdata_buffer_[i].KI_ = 0;
                        mod.txdata_buffer_[i].KD_ = 0;
                        legs_enabled++;
                    }
                }
            }
            
            switch(hall_calibrate_status){
                case -1:{
                    switchMode(Mode::REST);
                }
                break;

                case 0:{
                    int cal_cnt = 0;
                    for (auto& mod : *legs_) {
                        if (mod.enable_){
                            for (int i = 0; i < 6; i++){
                                if (mod.rxdata_buffer_[i].calibrate_finish_ == 2) cal_cnt++;
                            }
                        }
                        if (cal_cnt == legs_enabled && measure_offset == 0) hall_calibrate_status++;
                        else if (cal_cnt == legs_enabled && measure_offset == 1) hall_calibrate_status = -1;
                    }
                }
                break;

                case 1:{
                 //   if (modules_list_->at(0).enable_){
                   //     legs_->at(0).io_.Motor_bias = legs_->at(0).Motor_bias;
                     //   cal_command[0] = - modules_list_->at(0).Motor_bias;
                       // legs_->at(0).txdata_buffer_[0].position_ = - modules_list_->at(0).Motor_bias;
                       // cal_dir_[0] = 1;
                    hall_calibrate_status++;
                }
                break;
                
                case 2:{
                    int finish_cnt = 0;
                    for (auto& mod : *legs_) {
                        if (mod.enable_){
                            for (int i = 0; i < 6; i++){
                                double errj = 0;
                                errj = cal_command[2];
                                if (fabs(errj) < cal_tol_){
                                    for (i = 0; i < 6; i++){
                                        mod.txdata_buffer_[i].position_ = 0;
                                        mod.txdata_buffer_[i].torque_ = 0;
                                        mod.txdata_buffer_[i].KP_ = 0;
                                        mod.txdata_buffer_[i].KI_ = 0;
                                        mod.txdata_buffer_[i].KD_ = 0;
                                    }
                                }
                            }
                        }
                    if (finish_cnt == legs_enabled) hall_calibrate_status++;
                    }
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

        case Mode::MOTOR: {
            // 先把目前 feedback publish 出去
            publishMsg(motor_fb_msg);

            // 逐條腿處理命令
            // legs_->at(0) 對 L1, 1→L2, 2→L3, 3→R1, 4→R2, 5→R3
            const int max_legs = 6;
            const int n = (legs_->size() < static_cast<size_t>(max_legs))
                        ? static_cast<int>(legs_->size())
                        : max_legs;

            for (int leg_idx = 0; leg_idx < n; ++leg_idx)
            {
                // 找到這隻腿對應的 LegCmd (來自 MotorCmdStamped)
                const motor_msg::LegCmd* leg_cmd = nullptr;
                switch (leg_idx) {
                    case 0: leg_cmd = &motor_cmd_msg.l1(); break;
                    case 1: leg_cmd = &motor_cmd_msg.l2(); break;
                    case 2: leg_cmd = &motor_cmd_msg.l3(); break;
                    case 3: leg_cmd = &motor_cmd_msg.r1(); break;
                    case 4: leg_cmd = &motor_cmd_msg.r2(); break;
                    case 5: leg_cmd = &motor_cmd_msg.r3(); break;
                    default: break;
                }
                if (!leg_cmd) {
                    continue; // index 超出 L1~R3，就跳過
                }

                // 從該腿的 LegCmd 拿 DC 指令
                const auto& dc = leg_cmd->dc();

                // 寫到對應的 LegModule txdata_buffer_
                auto& mod = legs_->at(leg_idx);

                for (int i = 0; i < 6; i++) {
                    mod.txdata_buffer_[i].position_ = dc.position();
                    mod.txdata_buffer_[i].torque_   = dc.torque();
                    mod.txdata_buffer_[i].KP_       = dc.kp();
                    mod.txdata_buffer_[i].KI_       = dc.ki();
                    mod.txdata_buffer_[i].KD_       = dc.kd();
                }
            }

            break;
}


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

void ModeFsm::publishMsg(motor_msg::MotorStateStamped& motor_fb_msg)
{
    const auto& leg = legs_->at(0); 
    const auto& rx  = leg.rxdata_buffer_[0]; 
    const auto& tx  = leg.txdata_buffer_[0]; 

    auto* dc = motor_fb_msg.mutable_l1()->mutable_dc();
    dc->set_position(rx.position_);
    dc->set_velocity(rx.velocity_);
    dc->set_torque(rx.torque_ * tx.KT_);
}




// ok
