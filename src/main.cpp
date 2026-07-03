// src/main.cpp
#include "vn100.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    std::string config_path = (argc > 1) ? argv[1] : "../config.cfg";
    vn100 imu;
    imu.load_config(config_path);
    
    if (!imu.enabled()) {
        std::cerr << "IMU is disabled in config." << std::endl;
        return 1;
    }

    try {
        imu.open();  // Uses loaded config defaults
        std::cout << "IMU opened successfully. Reading data..." << std::endl;

        while (true) {
            imu.loop();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        imu.close();
        return 1;
    }

    imu.close();
    return 0;
}