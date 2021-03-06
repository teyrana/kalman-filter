// GPL v3 (c) 2020, Daniel Williams 

// standard library includes
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>

// POSIX / Linux specific handlers:
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

// 3rd-Party Library Includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

// project includes
#include "driver.hpp"
// #include "integrator.hpp"


static std::unique_ptr<IMU::Driver> driver;

// stream handling variables
static size_t sequence_number = -1;
static double second_timestamp = NAN;

// return the filtered, tared orientation estimate in euler angle form
static Eigen::Vector3d filtered_euler_angles;
// gyro rate: counts per degrees/sec
static Eigen::Vector3d raw_gyro;
// accelerometer: counts per g
static Eigen::Vector3d raw_accel;
// magnetometer: count per gauss
static Eigen::Vector3d raw_magno;

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

template<typename message_t>
void handle_stream_data( message_t& stream_message){
    ++sequence_number;

    const uint32_t millisecond_timestamp = stream_message.timestamp();
    second_timestamp = (static_cast<double>(millisecond_timestamp)/1000000);

    {   // filtered data: 
        stream_message.read_vector3d(0, filtered_euler_angles );

        {   // DEBUG
            const double pitch = filtered_euler_angles[0];
            const double yaw = filtered_euler_angles[1];
            const double roll = filtered_euler_angles[2];
            SPDLOG_INFO("    (#{:4d}) @{:.3f}    [ pitch: {:+f}, yaw: {:+f}, roll(+): {:+f} ]", 
                        sequence_number, second_timestamp, pitch, yaw, roll );
        }   // DEBUG
    }

    {   // raw data: 36 bytes: Vector(float x3), Vector(float x3), Vector(float x3)
        stream_message.read_vector3d(12, raw_gyro);
        stream_message.read_vector3d(24, raw_accel);
        stream_message.read_vector3d(36, raw_magno);

        {   // DEBUG
            SPDLOG_INFO("               [ {:+f}, {:+f}, {:+f} ][ {:+f}, {:+f}, {:+f} ][ {:+f}, {:+f}, {:+f} ]", 
                        raw_gyro[0],raw_gyro[1],raw_gyro[2],
                        raw_accel[0],raw_accel[1],raw_accel[2],
                        raw_magno[0],raw_magno[1],raw_magno[2] );
        }   // DEBUG
    }
}

static void sig_handler(int _)
{
    (void)_;  // quiet "unused variable" warnings
    SPDLOG_WARN("Ctrl-C Detected.  Exiting.");
    IMU::Driver::streaming = false;
}

int main()
{
    setup_logging();

    // register signal handler (for example: ctrl-c)
    signal(SIGINT, sig_handler);

    SPDLOG_INFO("## Creating IMU Driver...");
    driver = std::make_unique<IMU::Driver>();

    SPDLOG_INFO("## Setting up IMU Driver...");
    if( 0 != driver->open("/dev/ttyUSB0", 115200)){
        SPDLOG_ERROR("<< Failed to connect IMU. Exiting.");
        return EXIT_FAILURE;
    } else {
        SPDLOG_ERROR(">> Connected to IMU.");
    }

    SPDLOG_INFO( "==== Starting Streams: ==== ");
    if( EXIT_SUCCESS == driver->stream(std::chrono::milliseconds(500)) ){
        SPDLOG_INFO("==== Monitoring: ====");
        driver->monitor( handle_stream_data );
        return EXIT_SUCCESS;
    } else {
        SPDLOG_ERROR("!! Error setting up stream !!");
        ::sleep(1);
        driver->stop();
        return EXIT_FAILURE;
    }
}
