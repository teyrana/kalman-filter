// GPL v3 (c) 2020, Daniel Williams 


#ifndef _IMU_INTERFACE_HPP_
#define _IMU_INTERFACE_HPP_

// standard library headers
#include <chrono>

// POSIX / Linux Headers
#include <signal.h>

// Serial API
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

// 3rd-Party Library Includes
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

// 1st-party Internal Headers
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
        STREAM = CONNECTED_FLAG | CONFIGURED_FLAG | STREAMING_FLAG,
        SHUTDOWN = CONFIGURED_FLAG
    };

    // Remap axes: from: right-up-forward / "Natural Axes" (hardware default)
    //               to: forward-right-down / Conventional Axes (UUV Literature)
    static constexpr uint8_t axis_order = 0x03;
    // flip z: forward-right-up => forward-right-down
    static constexpr uint8_t axis_flips = 0x00;
    static constexpr uint8_t axis_mapping = axis_flips | axis_order;

    // // (explicitly defaults set to 0 -- hardware defaults)
    // static constexpr uint8_t axis_mapping = 0;

    // there are 8 possible slots in the command, as implemented in hardware
    std::array<uint8_t, 8> command_slots = {
            1,  // Command 0x01: request the filtered, tared Euler Angles (returns 12 bytes)
            64, // Command 0x40: get all raw sensor data:   gyro x3, accel x3, compass x3 (returns 3x3x4==36 bytes)
                // note: the default (0xFF) is to stream nothing
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   

    /// \brief the expected type of each streaming message;
    typedef IMU::Response<48> stream_message_t;


public:
    Driver();
    ~Driver();

    int configure();

    /// \brief watch the incoming message stream, and invoke the callback with each increment of data 
    int monitor( void (*callback) (stream_message_t& buffer) );

    int open(std::string _path, uint32_t _baudrate);

    int inline state(){ return state_; }

    /// \brief stop the driver & connection, and shutdown
    void stop();


    template<typename command_t>
    bool request( const command_t& cmd );

    template<typename command_t, typename response_t>
    bool request( const command_t& cmd, response_t& res );

    /// \param desired_stream_interval interval, in msec
    bool stream();
    bool stream( std::chrono::microseconds desired_stream_interval );

private:    

    // /// \brief receive a response of a given type (i.e. size)
    // template<typename response_t>
    // ssize_t receive( response_t& res);

    template<typename command_t>
    void write( const command_t& cmd);

public:  // properties
    static volatile sig_atomic_t streaming;

private:  // properties
    Serial::Connection conn_;

    spdlog::logger& log;

    /// \brief milliseconds to wait for the first byte of traffic
    std::chrono::microseconds receive_first_byte_timeout;
    /// \brief milliseconds to wait for the last bytes (from the first byte) 
    std::chrono::microseconds receive_last_byte_timeout;

    /// \brief output streaming data once every 'streaming_interval' microseconds
    std::chrono::microseconds stream_interval;
    /// \brief output streaming for this many microseconds
    std::chrono::microseconds stream_duration;
    /// \brief start streaming after this many microseconds
    std::chrono::microseconds stream_delay;

    DriverState_t state_;

}; // class Interface
} // namespace IMU

// to include the template-function definitions
#include "driver.inl"

#endif // #ifndef _IMU_INTERFACE_HPP_
