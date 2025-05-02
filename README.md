# kilin_sbRIO_ws

## Overview
This repository provides a driver for FPGA-based systems. It leverages the `yaml-cpp` and `Eigen` libraries for YAML configuration parsing and linear algebra computations, respectively. And this is build on sbrio and cooperate with gRPC.

---

# Structure of the kilin_sbRIO_ws workspace
```
kilin_sbRIO_ws/
├── install/
├── kilin_fpga_driver/
│   ├── build/
│   ├── cmake/
│   ├── config/
│   ├── fpga_bitfile/
│   ├── include/
│   ├── log/
│   └── src/
└── third_party/
    ├── eigen/
    ├── grpc_core/
    └── yaml-cpp/
```
## Prerequisites

Before building the project, ensure that the following dependencies are installed on your system:

### 1. **yaml-cpp**
```bash
cd kilin_sbRIO_ws/third_party/yaml-cpp/
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/kilin_sbRIO_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/kilin_sbRIO_ws/install
make -j16
sudo make install
```

### 2. **Eigen**
```bash
cd kilin_sbRIO_ws/third_party/eigen/
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/kilin_sbRIO_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/kilin_sbRIO_ws/install
make -j16
sudo make install
```

### 3. **grpc**
for grpc_core installation, please refer to the [grpc_core](https://github.com/hiho817/grpc_core.git)

## Change Address
Cause the path can not be find on sbRIO, path of the bitfile and log path are hard-coded in the code, you need to change the path in the following files:
1. /fpga_server.hpp (CONFIG_PATH and FPGA_PATH)
2. /config.yaml (log_path)
3. /NiFpga_FPGA_CANBus_4module_v3_steering.h (NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile)

## compiler
```bash
cd kilin/kilin_fpga_driver/
mkdir build && cd build
$ cmake .. -DCMAKE_PREFIX_PATH=$HOME/kilin_sbRIO_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/kilin_sbRIO_ws/install -DOPENSSL_ROOT_DIR=$HOME/kilin_sbRIO_ws/install/ssl
make -j16
```