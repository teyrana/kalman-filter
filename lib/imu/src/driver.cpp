// GPL v3 (c) 2020, Daniel Williams 

// Standard Library Includes
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>

// Posix / Linux System Includes
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// 3rd-Party Library Includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

// 1st-Party. Program Includes
#include "driver.hpp"

using IMU::Driver;

Driver::Driver()
    : conn_("/dev/ttyS0", 9600)
    , stream_interval( 250 * 1000)    // 250 ms
    , stream_duration( 25000 * 1000)  // 2.5s
    , stream_delay( 1000 )            // minimum delay
    , stream_receive_message()
    , state_(ERROR)
{}

Driver::Driver( const std::string _path, uint32_t _baud)
    : conn_(_path, _baud)
    , stream_interval( 250 * 1000)    // 250 ms
    , stream_duration( 25000 * 1000)  // 2.5s
    , stream_delay( 1000 )            // minimum delay
    , stream_receive_message()
    , state_(ERROR)
{

    if( !conn_.is_open() ){
        state_ = ERROR;
        return;
    }
    state_ = STARTUP;
    
    if( 0 != configure() ){
        state_ = ERROR;
        return;
    }
    
    state_ = IDLE;
    return;    
}

Driver::~Driver(){
    if( STREAM == state_ ){
        const auto bytes_written = write( stop_stream_command );

        // ensure the command is sent, before we close the socket:
        usleep( bytes_written * 10 );  // nominal write speed: <10 usec/byte
    }
}

int Driver::configure(){
    if( ! conn_.is_open() ){
        return -1;
    }

    fprintf(stderr, ">> Configuring serial device...\n");

//     ssize_t bytes_written = 0;
//     { // set running average order:
//         // set: static running average mode
//         auto set_average_mode_command = Command<1>( 124, 0 );
//         bytes_written = write( set_average_mode_command );
//         usleep( bytes_written * 10 );

//     }{ // set axes order:
//         auto set_axis_directions_command = Command<1>( 116 );
// #ifdef REMAP_AXES
//         // Remap axes: from: "Natural Axes" (hardware default)
//         //               to: Conventional Axes (UUV Literature)
//         // enum for: right-up-forward => forward-right-down
//         set_axis_directions_command.data(0) = 0x03 | 0x02; 
// #else
//         // explicitly set to hardware defaults:
//         set_axis_directions_command.data(0) = 0;
// #endif 
//         set_axis_directions_command.pack();
//         auto bytes_written = write( set_axis_directions_command );
//         usleep( bytes_written * 10 );
//     }{
//         // // 16 (0x10) Set Euler angle decomposition order
//         // auto set_euler_order_command = Command<0>( 156 );
//         // set_euler_order_command.data(0) = ?
//         // auto bytes_written = write( set_axis_directions_command );
//         // usleep( bytes_written * 10 );
//     }

    fprintf(stderr, "<< Device Configured.\n");

    state_ = IDLE;
    return 0;
}

int Driver::monitor(){

    while( STREAM == state_ ){
        ssize_t bytes_read = receive( stream_receive_message );

        if (0 == bytes_read) {
            fprintf(stderr, "    !! Error while streaming!! \n");
            fprintf(stderr, "    !! [%d]: %s !!\n", errno, strerror(errno) );
            state_ = ERROR;
        }

        if( stream_receive_message.size() != static_cast<size_t>(bytes_read)){
            continue;
        } else { 
            // TODO: implement happy path here

        }
    }

    return -1;  // error return path
}

#ifdef DEBUG
ssize_t Driver::get_euler_angles(){

    std::cerr << "## 1: Requesting Euler Angles: (Filtered, Tared) " << std::endl;
    if( 0 > write( get_euler_angles_command ) ) {
        std::cerr << "    <<!!: Command Error... abort!" << std::endl;
        abort();
    }

    // to: receive-buffer:
    Serial::Message<0,12,0> receive_message;
    ssize_t bytes_received = 0;
    if( 0 > (bytes_received = receive( receive_message))){
        std::cerr << "    <<!!: Receive Error... abort!" << std::endl;
        abort();
    }

    fprintf(stdout, "    >> Received Euler Angles (%+ld bytes).\n", bytes_received);
    // for( size_t i = 0; i < receive_message.size(); ++i){
    //     if( 0 == i%8 ){
    //         fputc(' ', stderr);
    //     }
    //     fprintf(stderr, "%02X ", receive_message[i]);
    // }
    // fputc('\n', stderr);
     
    Eigen::Vector3d euler_angles(0,0,0);
    euler_angles[0] = receive_message.read_float32( 4 );
    euler_angles[1] = receive_message.read_float32( 0 );
    euler_angles[2] = receive_message.read_float32( 8 );

    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        >> angle[0]:  %+f\n", euler_angles[0] );
    fprintf( stdout, "        >> angle[1]:  %+f\n", euler_angles[1] );
    fprintf( stdout, "        >> angle[2]:  %+f\n", euler_angles[2] );
    
    return bytes_received;
}
#endif

#ifdef DEBUG
ssize_t Driver::get_quaternion(){

    std::cerr << "## 1: Requesting Quaternion: (Filtered, Tared)" << std::endl;
    if( 0 > write( get_quaternion_command ) ){
        std::cerr << "    <<!!: Command Error... abort!" << std::endl;
        abort();
    }

    // to: receive-buffer:
    Serial::Message<0,16,0> receive_message;
    ssize_t bytes_received = 0;
    if( 0 > (bytes_received = receive( receive_message))){
        std::cerr << "    <<!!: Receive Error... abort!" << std::endl;
        abort();
    }

    Eigen::Quaternionf orientation_raw(0,0,0,0);
    // confirmed that the return order is: [X,Y,Z,W]
    orientation_raw.x() = receive_message.read_float32(0);
    orientation_raw.y() = receive_message.read_float32(4);
    orientation_raw.z() = receive_message.read_float32(8);
    orientation_raw.w() = receive_message.read_float32(12);

    fprintf( stdout, "    >> Received Quaternion:    (%+ld bytes).\n", bytes_received);

    Eigen::Quaterniond orientation_expanded( orientation_raw );
    fprintf( stdout, "        :w: %+f\n", orientation_expanded.w() );
    fprintf( stdout, "        :x: %+f\n", orientation_expanded.x() );
    fprintf( stdout, "        :y: %+f\n", orientation_expanded.y() );
    fprintf( stdout, "        :z: %+f\n", orientation_expanded.z() );

    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Matrix3d rotation = orientation_expanded.toRotationMatrix();
    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(0,0), rotation(0,1), rotation(0,2) );
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(1,0), rotation(1,1), rotation(1,2) );
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(2,0), rotation(2,1), rotation(2,2) );
    
    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Vector3d euler_angles = rotation.eulerAngles( 1, 0, 2);

    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        >> angle[0]:  %+f\n", euler_angles[0] );
    fprintf( stdout, "        >> angle[1]:  %+f\n", euler_angles[1] );
    fprintf( stdout, "        >> angle[2]:  %+f\n", euler_angles[2] );

    return bytes_received;
}
#endif

#ifdef DEBUG
ssize_t Driver::get_rotation_matrix(){

    std::cerr << "## 1: Requesting Rotation Matrix: (Filtered, Tared) " << std::endl;
    if( 0 > write( get_rotation_matrix_command ) ){
        std::cerr << "    <<!!: Command Error... abort!" << std::endl;
        abort();
    }

    ssize_t bytes_received = 0;

    // to: receive-buffer:
    Serial::Message<0,36,0> receive_message;
    if( 0 > (bytes_received = receive( receive_message))){
        std::cerr << "    <<!!: Receive Error... abort!" << std::endl;
        abort();
    }
    fprintf(stdout, "    >> Received Rotation Matrix (%+ld bytes).\n", bytes_received);

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    rotation(0,0) = receive_message.read_float32(  0 );
    rotation(0,1) = receive_message.read_float32(  4 );
    rotation(0,2) = receive_message.read_float32(  8 );
    rotation(1,0) = receive_message.read_float32( 12 );
    rotation(1,1) = receive_message.read_float32( 16 );
    rotation(1,2) = receive_message.read_float32( 20 );
    rotation(2,0) = receive_message.read_float32( 24 );
    rotation(2,1) = receive_message.read_float32( 28 );
    rotation(2,2) = receive_message.read_float32( 32 );

    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(0,0), rotation(0,1), rotation(0,2) );
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(1,0), rotation(1,1), rotation(1,2) );
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(2,0), rotation(2,1), rotation(2,2) );

    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Vector3d euler_angles = rotation.eulerAngles( 1, 0, 2);

    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        >> angle[0]:  %+f\n", euler_angles[0] );
    fprintf( stdout, "        >> angle[1]:  %+f\n", euler_angles[1] );
    fprintf( stdout, "        >> angle[2]:  %+f\n", euler_angles[2] );

    return bytes_received;
}
#endif

template<typename message_t>
ssize_t Driver::receive( message_t& receive_message ){
    return conn_.receive( receive_message.data(), receive_message.size() );
}

int Driver::stream() {
    ssize_t bytes_written = 0;

    // (1) Set up the streaming slots to stream the desire commands:
    IMU::Command<8> slot_command(0x50);
    slot_command[2] = 1;
    memset( const_cast<uint8_t*>(slot_command.data() + 3), 0xFF, 7 );
    slot_command.pack();
    bytes_written = write( slot_command );
    fprintf(stdout, ">>>> Wrote Streaming Slots.\n");
    // slot_command.fprinth(stdout);
    usleep( bytes_written * 5000 );

    // (2) Set streaming timing:
    IMU::Command<12> timing_command(0x52);
    timing_command.write_uint32( stream_interval, 0 );
    timing_command.write_uint32( stream_duration, 4 );
    timing_command.write_uint32( stream_delay,    8 );
    timing_command.pack();

    bytes_written = write( timing_command );
    fprintf(stdout, ">>>> Wrote stream Timing.\n");
    usleep( bytes_written * 5000 );

    // // (3) Start streaming, using the current configuration:
    bytes_written = write( start_stream_command );
    fprintf(stdout, ">>>> Wrote Start-Streaming Command.\n");
    usleep( bytes_written * 5000 );

    return 0;
}

template<typename command_t>
ssize_t Driver::write( const command_t& command) {
    return conn_.write( command.data(), command.size() );
}