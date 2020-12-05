// GPL v3 (c) 2020, Daniel Williams 


#ifndef _IMU_INTERFACE_HPP_
#define _IMU_INTERFACE_HPP_

// Serial API
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

// 3rd-Party Library Includes
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "command.hpp"
#include "../../serial/src/connection.hpp"

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

public:
    Driver();
    Driver(std::string _path, uint32_t _baudrate);
    ~Driver();

    int configure();

    int monitor();

    int inline state(){ return state_; }

    int stream();

    
#ifdef DEBUG
public:
    // mostly for debugging / development
    ssize_t get_euler_angles();
    ssize_t get_quaternion();
    ssize_t get_rotation_matrix();
#endif  // #ifdef DEBUG

private:  // properties
    Serial::Connection conn_;

    spdlog::logger& log;

    /// \brief output streaming data once every 'streaming_interval' microseconds
    uint32_t stream_interval;
    /// \brief output streaming for this many microseconds
    uint32_t stream_duration;
    /// \brief start streaming after this many microseconds
    uint32_t stream_delay;

    Serial::Message<0,12,0> stream_receive_message;

    DriverState_t state_;

private:  // functions

    template<typename message_t>
    ssize_t receive( message_t& receive_message);

    template<typename command_t>
    ssize_t write( const command_t & command);

}; // class Interface

} // namespace IMU

#endif // #ifndef _IMU_INTERFACE_HPP_
