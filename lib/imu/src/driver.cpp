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

// 1st-Party. Program Includes
#include "driver.hpp"

using IMU::Driver;

volatile sig_atomic_t IMU::Driver::streaming = false;

Driver::Driver()
    : conn_("/dev/ttyS0", 9600)
    , log( *spdlog::get("console"))
    , stream_interval( 500 * 1000)    // 500 ms
    , stream_duration( 25000 * 1000)  // 2.5s
    , stream_delay( 1000 )            // minimum delay
    , state_(ERROR)
{}

Driver::Driver( const std::string _path, uint32_t _baud)
    : conn_(_path, _baud)
    , log( *spdlog::get("console"))
    , stream_interval( 500 * 1000)    // 500 ms
    // , stream_duration( 0xffffffff) // production
    , stream_duration( 30 * 1000000 ) // 30s -- development
    , stream_delay( 1000 )            // minimum delay
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
    state_ = SHUTDOWN;
    stop();
}

int Driver::configure(){
    if( ! conn_.is_open() ){
        return -1;
    }

    log.info(">> Configuring serial device...");

    {   // 221 (0xDD) Set response Header
        IMU::Command<4, 0> set_response_header_command( 221 );
        set_response_header_command[5] = IMU::Response<0>::flags();
        set_response_header_command.pack();
        log.debug("    >> Setting response header fields => 0x{:02X}", IMU::Response<0>::flags() );
        write( set_response_header_command );
    
        // IMU::Command<0, 4> get_response_header_command( 222 );
        // write( get_response_header_command );
        // log.debug("    >> Getting response header fields...");
    }

    {   // set: static running average mode (0x00)
        // // IMU::Command<1,0> set_average_mode_command( 124, 0 );
        // IMU::Command<1,0> set_average_mode_command( 124 );
        // if( 1 < verbosity_){
        //     fprintf(stderr, "    >> Changing Average Mode (=> %d) ...\n", set_average_mode_command[2]);
        // }
        // write( set_average_mode_command );
        
    }{ // 116 (x074) Set Axes Directions:
        IMU::Command<1,0> set_axis_directions_command( 116, axis_mapping );
        log.debug("    >> Mapping Axes => 0x{:02X} ...", set_axis_directions_command[2]);
        write( set_axis_directions_command );
        
    }{  // 96 (0x60)  Set Tare with current orientation
        constexpr IMU::Command<0,0> set_tare_command( 96 );
        log.debug("    >> Setting Tare position to current orientation ...");
        write( set_tare_command );
        
    }{  // 95 (0x5F) Set internal timestamp ( == 0 )(=> usec since startup)
        IMU::Command<4,0> zero_timestamp_command(95);
        zero_timestamp_command.pack();
        log.debug("    >> Zero Timestamp position == 0 ...");
        write( zero_timestamp_command );

    }{  // // 16 (0x10) Set Euler angle decomposition order
        // auto set_euler_order_command = Command<0>( 156 );
        // set_euler_order_command.data(0) = ?
        // auto bytes_written = write( set_axis_directions_command );
        // usleep( bytes_written * 10 );
    
    }

    log.info("<< Device Configured.");

    state_ = IDLE;
    return 0;
}

int Driver::monitor( void (*callback) (stream_message_t& buffer) ){
    stream_message_t receive_buffer;
    ssize_t bytes_read = -1;
    while( streaming ){
    
        bytes_read = receive( receive_buffer );

        if ( -2 == bytes_read ) {
            // ignore frame; error is already handled
            continue;
        }else if ( -1 == bytes_read ) {
            log.error("    !! Error while streaming: [{}]: {} !!", errno, strerror(errno) );
            // return (state_ = ERROR);
            continue;
        }else if( receive_buffer.size() != static_cast<size_t>(bytes_read)) {
            log.error("    !! Unexpected byte count !!");
            log.error("    !! Received: {} != {}", receive_buffer.size(), bytes_read );
            continue;
        // }else{
        //     log.debug("    << Received: {} bytes", bytes_read);
        }

        callback( receive_buffer );
    }
    log.info("    <<<< monitor loop finished.");
    state_ = IDLE;

    return state_;
}

void Driver::stop(){
    if( conn_.is_open() ){
        // 86 (0x56) Stop Streaming
        constexpr IMU::Command<0,0> stop_stream_command( 86 );
        write( stop_stream_command );
    
        conn_.flush();
    }
}

bool Driver::stream() {
    bool success = true;

    IMU::Command<8,0> slot_command( 80 );
    IMU::Command<8,0> set_slots_command( 80 );
    set_slots_command.write_bytes( command_slots.data(), 0, 8);
    set_slots_command.pack();
    success &= write( set_slots_command );
    log.info("    >>>> Wrote Streaming Slots.");

    IMU::Command<12,0> timing_command( 82 );
    timing_command.write_uint32( stream_interval, 0 );
    timing_command.write_uint32( stream_duration, 4 );
    timing_command.write_uint32( stream_delay,    8 );
    timing_command.pack();
    success &= write( timing_command );
    log.info("    >>>> Wrote stream Timing: {}, {}, {}", 
                stream_interval, stream_duration, stream_delay );
    

    constexpr IMU::Command<0,0> start_stream_command( 85 );
    success &= write( start_stream_command );
    log.info("    >>>> Wrote Start-Streaming Command.");

    if(success){
        state_ = STREAM;
        streaming = true;
        return EXIT_SUCCESS ;
    }else{
        state_ = ERROR;
        streaming = false;
        return EXIT_FAILURE;
    }
}

bool Driver::stream( uint32_t desired_stream_interval ) {
    if( 0 == desired_stream_interval ){
        return -1;
    }

    stream_interval = desired_stream_interval * 1000;
    stream_duration = 0xFFFFFFFF;
    stream_delay = 1000; // minimum delay

    return stream();
}

