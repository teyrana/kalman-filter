// GPL v3 (c) 2020, Daniel Williams 

// standard library includes
#include <iostream>
#include <iterator>
using std::cerr;
using std::cout;
using std::endl;

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

// project includes
#include "driver.hpp"


void setup_logging() {

    // create color multi threaded logger
    auto console = spdlog::stdout_color_mt("console");    
    auto err_logger = spdlog::stderr_color_mt("stderr");    

    spdlog::set_level(spdlog::level::trace);
    // change log pattern
    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$] %v");
    
    // Compile time log levels
    // define SPDLOG_ACTIVE_LEVEL to desired level
    SPDLOG_INFO("Logging started");
}

int main()
{
    setup_logging();

    IMU::Driver imu("/dev/ttyUSB0", 115200);

    if( IMU::Driver::IDLE != imu.state()){
        SPDLOG_INFO("<< Failed to start up IMU. Exiting.");
        return EXIT_FAILURE;
    }

    SPDLOG_INFO( "==== Starting Streams: ==== ");
    if( EXIT_SUCCESS == imu.stream()){
        // imu.monitor();
    } else {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
