// GPL v3 (c) 2020, Daniel Williams 

// Standard Library Includes
#include <cstdio>
#include <ctime>

// // Posix / Linux System Includes
#include <unistd.h>   // `usleep`

using IMU::Driver;

template<typename response_t>
ssize_t Driver::receive( response_t& res){
    auto bytes_read = conn_.receive( res.data(), res.size() );

    if( res.provides_success() && (!res.success()) ){
        log.warn("        XX 'command failed' flag: {}", res[0]);
        
        // this resests byte flow, and it should pick up again at the frame boundary
        conn_.flush();

        return -1;
    }

    return bytes_read;
}

template<typename command_t, typename response_t>
response_t& Driver::request( const command_t & cmd, response_t& res){

    // cmd.log( log, spdlog::level::trace, "    >> Sending command:");

    /* ssize_t bytes_written =*/ conn_.write( cmd.data(), cmd.size() );
    conn_.drain();

    ssize_t bytes_received = conn_.receive( res.data(), res.size() );

    if( 0 == bytes_received ){
        log.error("        << Zero-Length Response expected! ");
    }

    // res.log( log, spdlog::level::trace, "    << Response:");

    return res;
}

template<typename command_t>
bool IMU::Driver::write( const command_t & cmd){

    IMU::Response<cmd.response_payload_size> res;

    res = request( cmd, res );

    if( res.provides_success() && (!res.success()) ){
        log.warn("        XX 'command failed' flag.");
        return false;
    }

    if( res.provides_command()){
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
