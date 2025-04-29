#include "fpga_server.hpp"


int main(int argc, char* argv[])
{
    signal(SIGINT, inthand);

    important_message("[FPGA Server] : Launched");
    
    while (!is_sys_stop())
    {
        /* code */
    }
    
    return 0;
}

//TODO: IO test LED