// GPL v3 (c) 2020, Daniel Williams 

// Standard Library Includes
#include <chrono>
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
    : conn_()
    , log( *spdlog::get("console"))
    , receive_first_byte_timeout(std::chrono::milliseconds(250))
    , receive_last_byte_timeout(std::chrono::milliseconds(50))
    , stream_interval(std::chrono::milliseconds(500))
    // , stream_duration(std::chrono::microseconds::max()) // production
    , stream_duration(std::chrono::seconds(32))         // development
    , stream_delay( std::chrono::milliseconds(1) )  // minimum delay
    , state_(ERROR)
{}

int Driver::open( const std::string _path, uint32_t _baud){
    conn_.open( _path, _baud );

    if( !conn_.is_open() ){
        state_ = ERROR;
        return -1;
    }
    state_ = STARTUP;
    
    if( 0 != configure() ){
        state_ = ERROR;
        return -1;
    }
    
    state_ = IDLE;
    return 0;
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
        request( set_response_header_command );
    
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
        request( set_axis_directions_command );
        
    }{  // 96 (0x60)  Set Tare with current orientation
        constexpr IMU::Command<0,0> set_tare_command( 96 );
        log.debug("    >> Setting Tare position to current orientation ...");
        request( set_tare_command );
        
    }{  // 95 (0x5F) Set internal timestamp ( == 0 )(=> usec since startup)
        IMU::Command<4,0> zero_timestamp_command(95);
        zero_timestamp_command.pack();
        log.debug("    >> Zero Timestamp position == 0 ...");
        request( zero_timestamp_command );

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

        bytes_read = conn_.receive( receive_buffer.data(), receive_buffer.size(), receive_first_byte_timeout, receive_last_byte_timeout);

        if ( 0 > bytes_read ) {
            log.error("    !! Error while streaming: [{}]: {} !!", errno, strerror(errno) );
            // return (state_ = ERROR);
            continue;

        }else if( receive_buffer.provides_success() && (!receive_buffer.success()) ){
            log.warn("    XX command failed !! (value = {})", receive_buffer[0]); 
            // this resests byte flow, and it should pick up again at the frame boundary
            conn_.flush();
            
            state_ = ERROR;
            return state_;
            
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

    IMU::Command<8,0> set_slots_command( 80 );
    set_slots_command.write_bytes( command_slots.data(), 0, 8);
    set_slots_command.pack();
    success &= request( set_slots_command );
    log.info("    >>>> Wrote Streaming Slots.");

    IMU::Command<12,0> timing_command( 82 );
    const uint32_t raw_interval_usec = stream_interval.count();
    timing_command.write_uint32( raw_interval_usec, 0 );
    uint32_t raw_duration_usec = stream_duration.count();
    if( stream_duration == std::chrono::microseconds::max() ){
        raw_duration_usec = 0xFFFFFFFF;
    }
    timing_command.write_uint32( raw_duration_usec, 4 );
    const uint32_t raw_delay_usec = stream_delay.count();
    timing_command.write_uint32( raw_delay_usec,    8 );
    timing_command.pack();
    success &= request( timing_command );
    log.info("    >>>> Wrote stream Timing: {}, {}, {}", 
                stream_interval.count(), stream_duration.count(), stream_delay.count());
    
    // if we expect to receive a frame every 'stream_interval', then wait 
    // twice the expected interval for the first byte.
    // >> this is just a rule-of-thumb / heuristic <<
    receive_first_byte_timeout = 2*stream_interval;

    constexpr IMU::Command<0,0> start_stream_command( 85 );
    success &= request( start_stream_command );
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

bool Driver::stream( std::chrono::microseconds desired_stream_interval ) {
    stream_interval = desired_stream_interval;
    // stream_duration = 0xFFFFFFFF;
    return stream();
}

