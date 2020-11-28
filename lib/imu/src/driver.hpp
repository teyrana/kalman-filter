// GPL v3 (c) 2020, Daniel Williams 


#ifndef _IMU_INTERFACE_HPP_
#define _IMU_INTERFACE_HPP_

// Serial API
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

// 3rd-Party Library Includes
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "../../serial/src/connection.hpp"
#include "command.hpp"
#include "response.hpp"

namespace IMU {


class Driver {
public:
    static constexpr uint32_t CONNECTED_FLAG = 0x01;
    static constexpr uint32_t CONFIGURED_FLAG = 0x02;
    static constexpr uint32_t STREAMING_FLAG = 0x04;
    enum DriverState_t {
        ERROR = 0,
        // serial port is ready to use
        STARTUP = CONNECTED_FLAG,
        // the peer device is ready to use (i.e. configured)
        IDLE = CONNECTED_FLAG | CONFIGURED_FLAG, 
        STREAM = CONNECTED_FLAG | CONFIGURED_FLAG | STREAMING_FLAG
    };

    // Remap axes: from: right-up-forward / "Natural Axes" (hardware default)
    //               to: forward-right-down / Conventional Axes (UUV Literature)
    static constexpr uint8_t axis_order = 0x03;
    // flip z: forward-right-up => forward-right-down
    static constexpr uint8_t axis_flips = 0x00;
    static constexpr uint8_t axis_mapping = axis_flips | axis_order;

    // // (explicitly defaults set to 0 -- hardware defaults)
    // static constexpr uint8_t axis_mapping = 0;

public:
    Driver();
    Driver(std::string _path, uint32_t _baudrate);
    ~Driver();

    int configure();

    /// \brief the expected type of each streaming message;
    typedef IMU::Response<12> stream_message_t;
    
    /// \brief watch the incoming message stream, and invoke the callback with each increment of data 
    int monitor( void (*callback) (stream_message_t& buffer) );

    int inline state(){ return state_; }

    /// \param force send a stop command, regardless of driver state
    void stop(bool force=false);

    bool stream();


    /// \param desired_stream_interval interval, in msec
    bool stream( uint32_t desired_stream_interval );

public:    

    template<typename response_t>
    ssize_t receive( response_t& res);

    template<typename command_t, typename response_t>
    response_t& request( const command_t& cmd, response_t& res );

    template<typename command_t>
    bool write( const command_t& cmd);

private:  // properties
    Serial::Connection conn_;

    spdlog::logger& log;

    /// \brief output streaming data once every 'streaming_interval' microseconds
    uint32_t stream_interval;
    /// \brief output streaming for this many microseconds
    uint32_t stream_duration;
    /// \brief start streaming after this many microseconds
    uint32_t stream_delay;

    DriverState_t state_;

}; // class Interface
} // namespace IMU

// to include the template-function definitions
#include "driver.inl"

#endif // #ifndef _IMU_INTERFACE_HPP_
