// GPL v3 (c) 2020, Daniel Williams 

// standard library includes
#include <cstdio>
#include <cstring>
#include <iostream>

// Posix / Linux System Includes
#include <fcntl.h>
#include <unistd.h>

// 3rd-Party Library Includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

// project includes
#include "driver.hpp"

static std::unique_ptr<IMU::Driver> driver;

// ================================= Function Definitions ================================= 

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


#ifdef DEBUG
ssize_t get_euler_angles(){
    SPDLOG_INFO("## 1: Requesting Euler Angles: (Filtered, Tared) ");
    // 1 (0x01), Read tared orientation as euler angles
    constexpr IMU::Command<0,12> get_euler_angles_command(1);
    IMU::Response<get_euler_angles_command.response_payload_size> euler_angles_response;
    driver->request( get_euler_angles_command, euler_angles_response);
    if( ! euler_angles_response.success() ){
        SPDLOG_ERROR("    <<!!: Command Error... abort!");
        return -1;
    }

    SPDLOG_INFO("    >> Received Euler Angles ({} bytes).", euler_angles_response.size());

    Eigen::Vector3d euler_angles(0,0,0);
    euler_angles[0] = euler_angles_response.read_float32( 4 );
    euler_angles[1] = euler_angles_response.read_float32( 0 );
    euler_angles[2] = euler_angles_response.read_float32( 8 );

    SPDLOG_INFO("    >> Converted to Euler Angles:");
    SPDLOG_INFO("        >> angle[0]:  {:+f}", euler_angles[0] );
    SPDLOG_INFO("        >> angle[1]:  {:+f}", euler_angles[1] );
    SPDLOG_INFO("        >> angle[2]:  {:+f}", euler_angles[2] );
    
    return euler_angles_response.size();
}
#endif

#ifdef DEBUG
ssize_t get_quaternion(){
    ssize_t bytes_received = 0;

    SPDLOG_INFO("## 1: Requesting Quaternion: (Filtered, Tared)");
    // 0 (0x00), Read tared orientation as quaternion
    constexpr IMU::Command<0,16> get_quaternion_command(0);    
    IMU::Response<get_quaternion_command.response_payload_size> quaternion_response;
    driver->request( get_quaternion_command, quaternion_response);
    if( ! quaternion_response.success() ){
        SPDLOG_ERROR("    <<!!: Command Error... abort!");
        return -1;
    }

    SPDLOG_INFO("    >> Received Quaternion Data: ({} bytes)", quaternion_response.size());
    
    Eigen::Quaternionf orientation_raw(0,0,0,0);
    // confirmed that the return order is: [X,Y,Z,W]
    orientation_raw.x() = quaternion_response.read_float32(0);
    orientation_raw.y() = quaternion_response.read_float32(4);
    orientation_raw.z() = quaternion_response.read_float32(8);
    orientation_raw.w() = quaternion_response.read_float32(12);

    SPDLOG_INFO("    >> Received Quaternion:    (%+ld bytes).", bytes_received);

    Eigen::Quaterniond orientation_expanded( orientation_raw );
    SPDLOG_INFO("        :w: {:+f}", orientation_expanded.w() );
    SPDLOG_INFO("        :x: {:+f}", orientation_expanded.x() );
    SPDLOG_INFO("        :y: {:+f}", orientation_expanded.y() );
    SPDLOG_INFO("        :z: {:+f}", orientation_expanded.z() );

    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Matrix3d rotation = orientation_expanded.toRotationMatrix();
    SPDLOG_INFO("    >> Converted to Euler Angles:");
    SPDLOG_INFO("        [ {:+f}, {:+f}, {:+f} ]", rotation(0,0), rotation(0,1), rotation(0,2) );
    SPDLOG_INFO("        [ {:+f}, {:+f}, {:+f} ]", rotation(1,0), rotation(1,1), rotation(1,2) );
    SPDLOG_INFO("        [ {:+f}, {:+f}, {:+f} ]", rotation(2,0), rotation(2,1), rotation(2,2) );
    
    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Vector3d euler_angles = rotation.eulerAngles( 1, 0, 2);

    SPDLOG_INFO("    >> Converted to Euler Angles:");
    SPDLOG_INFO("        >> angle[0]:  {:+f}", euler_angles[0] );
    SPDLOG_INFO("        >> angle[1]:  {:+f}", euler_angles[1] );
    SPDLOG_INFO("        >> angle[2]:  {:+f}", euler_angles[2] );

    return bytes_received;
}
#endif

#ifdef DEBUG
ssize_t get_rotation_matrix(){

    SPDLOG_INFO("## 1: Requesting Rotation Matrix: (Filtered, Tared)");
    // 2 (0x02), Get tared orientation as rotation matrix
    constexpr IMU::Command<0,36> get_rotation_matrix_command(2);
    IMU::Response<get_rotation_matrix_command.response_payload_size> matrix_response;
    driver->request( get_rotation_matrix_command, matrix_response);
    if( ! matrix_response.success() ){
        SPDLOG_ERROR("    <<!!: Command Error... abort!");
        return -1;
    }

    SPDLOG_INFO("    >> Received Rotation Matrix ({} bytes):", matrix_response.size());

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    rotation(0,0) = matrix_response.read_float32(  0 );
    rotation(0,1) = matrix_response.read_float32(  4 );
    rotation(0,2) = matrix_response.read_float32(  8 );
    rotation(1,0) = matrix_response.read_float32( 12 );
    rotation(1,1) = matrix_response.read_float32( 16 );
    rotation(1,2) = matrix_response.read_float32( 20 );
    rotation(2,0) = matrix_response.read_float32( 24 );
    rotation(2,1) = matrix_response.read_float32( 28 );
    rotation(2,2) = matrix_response.read_float32( 32 );

    SPDLOG_INFO("    >> Received Rotation Matrix:");
    SPDLOG_INFO("        [ {:+f}, {:+f}, {:+f} ]", rotation(0,0), rotation(0,1), rotation(0,2) );
    SPDLOG_INFO("        [ {:+f}, {:+f}, {:+f} ]", rotation(1,0), rotation(1,1), rotation(1,2) );
    SPDLOG_INFO("        [ {:+f}, {:+f}, {:+f} ]", rotation(2,0), rotation(2,1), rotation(2,2) );

    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Vector3d euler_angles = rotation.eulerAngles( 1, 0, 2);

    SPDLOG_INFO("    >> Converted to Euler Angles:");
    SPDLOG_INFO("        >> angle[0]:  {:+f}", euler_angles[0] );
    SPDLOG_INFO("        >> angle[1]:  {:+f}", euler_angles[1] );
    SPDLOG_INFO("        >> angle[2]:  {:+f}", euler_angles[2] );

    return matrix_response.size();
}
#endif

// ================================= Main Function ================================= 

int main()
{
    setup_logging();

    SPDLOG_INFO("## Creating IMU Driver...");
    driver = std::make_unique<IMU::Driver>();

    SPDLOG_INFO("## Setting up IMU Driver...");
    if( 0 != driver->open("/dev/ttyUSB0", 115200)){
        SPDLOG_ERROR("<< Failed to connect IMU. Exiting.");
        return EXIT_FAILURE;
    } else {
        SPDLOG_ERROR(">> Connected to IMU.");
    }
    
    get_quaternion();

    get_rotation_matrix();

    get_euler_angles();

    return EXIT_SUCCESS;
}
