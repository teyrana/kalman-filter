// GPL v3 (c) 2020, Daniel Williams 

// standard library includes
#include <iostream>
#include <iterator>

// 3rd-Party Library Includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

// project includes
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

static size_t sequence_number = -1;

template<typename message_t>
void handle_stream_data( message_t& stream_message){
    ++sequence_number;

    const uint32_t millisecond_timestamp = stream_message.timestamp();
    const double second_timestamp = (static_cast<double>(millisecond_timestamp)/1000000);

    const double pitch = stream_message.read_float32( 0 );
    const double yaw = stream_message.read_float32( 4 );
    const double roll = stream_message.read_float32( 8 );

    SPDLOG_INFO("#### ({:4d}) @{:.3f}    [ pitch: {:+f}, yaw: {:+f}, roll(+): {:+f} ]", 
                    sequence_number, second_timestamp, pitch, yaw, roll );

    // Eigen::Vector3d euler_angles(0,0,0);
    // euler_angles[0] = stream_message.read_float32( 0 );
    // euler_angles[1] = stream_message.read_float32( 4 );
    // euler_angles[2] = stream_message.read_float32( 8 );

    // tbc...
}

int main()
{
    setup_logging();

    SPDLOG_INFO("## Setting up IMU Driver...");
    IMU::Driver imu("/dev/ttyUSB0", 115200);

    if( IMU::Driver::IDLE != imu.state()){
        SPDLOG_INFO("<< Failed to start up IMU. Exiting.");
        return EXIT_FAILURE;
    }

    SPDLOG_INFO( "==== Starting Streams: ==== ");
    if( EXIT_SUCCESS == imu.stream(500)){
        SPDLOG_INFO("==== Monitoring: ====");
        imu.monitor( handle_stream_data );
        return EXIT_SUCCESS;
    } else {
        SPDLOG_ERROR("!! Error setting up stream !!");
        ::sleep(1);
        imu.stop(true);
        return EXIT_FAILURE;
    }
}
