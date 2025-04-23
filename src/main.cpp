#include "fpga_server.hpp"


int main(int argc, char* argv[])
{
    signal(SIGINT, inthand);
    while (!is_sys_stop())
    {
        /* code */
    }
    
    return 0;
}