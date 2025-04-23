#include "fpga_server.hpp"

volatile sig_atomic_t sys_stop = 0;
/* CAPTURE SYS STOP SIGNAL TO KILL PROCESS*/
void inthand(int signum)
{
    sys_stop = 1;
}

bool is_sys_stop()
{
    return sys_stop;
}