#include "fpga_server.hpp"


int main(int argc, char* argv[])
{
    signal(SIGINT, inthand);

    important_message("[FPGA Server] : Launched");
    
    Kilin kilin;
    
    /* gRPC Topic */
    core::NodeHandler nh;

    core::Publisher<power_msg::PowerStateStamped>& power_pub = nh.advertise<power_msg::PowerStateStamped>("power/state");
    core::Subscriber<power_msg::PowerCmdStamped>& power_sub = nh.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_data_cb);

    core::Publisher<motor_msg::MotorStateStamped>& motor_pub = nh.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Subscriber<motor_msg::MotorCmdStamped>& motor_sub = nh.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_data_cb);

    kilin.interruptHandler(power_sub, power_pub, motor_sub, motor_pub);

    if (NiFpga_IsError(kilin.fpga_.status_)) std::cout << red << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << kilin.fpga_.status_ << reset << std::endl;
    else
    {
        endwin();
        important_message("\n[FPGA Server] : Exit Safely");
    }
    return 0;
}

//TODO: IO test LED