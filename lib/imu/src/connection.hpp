// GPL v3 (c) 2020, Daniel Williams 


#ifndef _IMU_INTERFACE_HPP_
#define _IMU_INTERFACE_HPP_

// Serial API
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

// 3rd-Party Library Includes
#include <Eigen/Geometry>

#include "command.hpp"

namespace IMU {

class Connection {
public:
    static constexpr uint32_t CONNECTED_FLAG = 0x01;
    static constexpr uint32_t CONFIGURED_FLAG = 0x02;
    static constexpr uint32_t STREAMING_FLAG = 0x04;
    enum ConnectionState_t {
        ERROR = 0,
        STARTUP = CONNECTED_FLAG,
        IDLE = CONNECTED_FLAG | CONFIGURED_FLAG, 
        STREAM = CONNECTED_FLAG | CONFIGURED_FLAG | STREAMING_FLAG
    };

public:
    Connection();
    Connection(std::string _path, uint32_t _baudrate);
    ~Connection();

    int configure();

    int stream();

    int monitor();
    
#ifdef DEBUG
public:
    // mostly for debugging / development
    ssize_t get_euler_angles();
    ssize_t get_quaternion();
    ssize_t get_rotation_matrix();
#endif  // #ifdef DEBUG

private:
    uint32_t baud_rate_;
    int fd_;
    std::string path_;

    /// \brief output streaming data once every 'streaming_interval' microseconds
    uint32_t stream_interval;
    /// \brief output streaming for this many microseconds
    uint32_t stream_duration;
    /// \brief start streaming after this many microseconds
    uint32_t stream_delay;

    static constexpr ssize_t expected_receive_bytes = 12;
    std::array<uint8_t, expected_receive_bytes> stream_receive_buffer;
    
    ConnectionState_t state; 

#ifdef DEBUG
    // 0 (0x00), Read tared orientation as quaternion
    static constexpr Command<0> readSingleQuaternionCommand = Command<0>(0);
    
    // 1 (0x01), Read tared orientation as euler angles
    static constexpr Command<0> readSingleEulerAnglesCommand = Command<0>(1);

    // 2 (0x02), Get tared orientation as rotation matrix
    static constexpr Command<0> readSingleRotationMatrixCommand = Command<0>(2);
#endif  // #ifdef DEBUG

    // 156 (0x9c) Get Euler angle decomposition order
    static constexpr Command<0> getEulerAngleDecompositionCommand = Command<0>( 156 );

    static constexpr Command<0> start_stream_command = Command<0>( 0x55 );

    // 86 (0x56) Stop Streaming
    static constexpr Command<0> stop_stream_command = Command<0>( 0x56 );

private:

    /// \brief serial-to-host conversion, floah a binary, big-endian representation of a float into a C-typed float variable
    static float load_float( uint8_t* source, float& dest );

    /// \brief serial-to-host conversion, floah a binary, big-endian representation of a float into a C-typed float variable
    static double byte4_to_double( uint8_t* source ){ float ret; return static_cast<double>(load_float(source, ret)); } 

    static uint32_t write_int32( uint32_t source, uint8_t * dest);

    template<typename command_t>
    ssize_t write( command_t cmd){ return write( cmd.data(), cmd.size()); }
    ssize_t write( const uint8_t * command, ssize_t command_length );
    ssize_t receive( uint8_t* source, ssize_t len );

}; // class Interface

} // namespace IMU

#endif // #ifndef _IMU_INTERFACE_HPP_
