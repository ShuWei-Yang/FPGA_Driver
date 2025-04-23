#ifndef FPGA_SERVER_HPP
#define FPGA_SERVER_HPP

#include "fpga_handler.hpp"

void inthand(int signum);
bool is_sys_stop();

#endif