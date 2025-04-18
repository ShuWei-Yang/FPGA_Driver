#ifndef __FPGAHANDLER_H
#define __FPGAHANDLER_H

#include "NiFpga.h"
#include "NiFpga_FPGA_RS485_v1_2.h"
// #include "can_packet.h"
// #include "color.hpp"
#include "msg.hpp"

#include <unistd.h>
#include <iostream>
#include <functional>
#include <signal.h>
#include <dlfcn.h>
#include <vector>
#include <ncurses.h>
#include <curses.h>
#include <iostream>
#include <bitset>
#include <string>
#undef OK

class ModuleIO{
    ModuleIO(NiFpga_Status status_, NiFpga_Session fpga_session_, std::string CAN_port_,
        std::vector<Motor> *motors_list);

    ModuleIO(){};

};











#endif
