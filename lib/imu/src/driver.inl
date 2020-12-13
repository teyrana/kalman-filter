// GPL v3 (c) 2020, Daniel Williams 

// Standard Library Includes
#include <cstdio>
#include <ctime>

// // Posix / Linux System Includes
#include <unistd.h>   // `usleep`

using IMU::Driver;

// template<typename response_t>
// ssize_t Driver::receive( response_t& res){
//     auto bytes_read = conn_.receive( res.data(), res.size() );

//     if( res.provides_success() && (!res.success()) ){
//         log.warn("    XX command failed !! (value = {})", res[0]);
        
//         // this resests byte flow, and it should pick up again at the frame boundary
//         conn_.flush();

//         return -2;
//     }

//     return bytes_read;
// }


template<typename command_t>
bool Driver::request( const command_t & cmd){
    IMU::Response<cmd.response_payload_size> res;
    return request(cmd, res);
}

template<typename command_t, typename response_t>
bool Driver::request( const command_t & cmd, response_t& res){

    // cmd.log( log, spdlog::level::trace, "    >> Sending command:");

    /* ssize_t bytes_written =*/ conn_.write( cmd.data(), cmd.size() );
    conn_.drain();
    
    if( 0 == res.size() ){
        log.error("        << No Response byts expected! ");
        return true;
    }

    const std::chrono::microseconds start_timeout( std::chrono::milliseconds(250) );
    const std::chrono::microseconds end_timeout( this->receive_last_byte_timeout );

    ssize_t bytes_received = conn_.receive( res.data(), res.size(), start_timeout, receive_last_byte_timeout );

    if(0 == bytes_received) {
        log.error("        << Zero-Length Response, but expected {} bytes! ", res.size());
        return false;
    } else if( res.provides_success() && (!res.success()) ){
        log.warn("        XX command request failed !!");
        return false;
    } else if( res.provides_command()){
        const uint8_t echo_command = res.command();
        if( (0xFF==echo_command) && (85!=cmd.command()) ){ 
            log.trace( "        !! Encountered streaming signal, on non stream-start command!!\n" );
            return false;
        }else if( 0xFF == echo_command ){
            // this is fine
            // log.trace( "        ## N/A Command: 0xFF\n" );
            return true;
        }else if( echo_command != cmd.command() ){ 
            log.info("        !? command mismatch: {:02X} != {:02X}", echo_command, cmd.command() );
            return false;
        }
    }

    return true;
}

template<typename command_t>
void IMU::Driver::write( const command_t & cmd){
    /* ssize_t bytes_written =*/ conn_.write( cmd.data(), cmd.size() );
    conn_.drain();
}
