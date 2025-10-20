#ifndef FPGA_SERVER_HPP
#define FPGA_SERVER_HPP

#include "fpga_handler.hpp"
#include "leg_module.hpp"
#include "console.hpp"
#include "fsm.hpp"

#include <NodeHandler.h>
#include <sys/time.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <mutex>
#include <signal.h>

#ifndef CONFIG_PATH
#define CONFIG_PATH "/home/shuweiyang/kilin_sbRIO_ws/kilin_fpga_driver/config/config.yaml"
#endif

void inthand(int signum);
bool is_sys_stop();

class Kilin{
    public:
        Kilin();
        FpgaHandler fpga_;
        YAML::Node yaml_node_;

        /* console */
        std::mutex main_mtx_;
        Console console_;

        /* powerboard state */
        std::vector<bool> powerboard_state_;
        bool digital_switch_;
        bool signal_switch_;
        bool power_switch_;

        /* robot state */
        std::vector<LegModule> motors_list_;
        std::vector<LegModule> servos_list_;
        ModeFsm fsm_;
        bool HALL_CALIBRATED_;

        void load_config_();
        int modules_num_;

        void mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
            core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
            core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
            core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

        void powerboardPack(power_msg::PowerStateStamped &power_fb_msg);

};

#endif

// ok