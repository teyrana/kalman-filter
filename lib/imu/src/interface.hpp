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

class Interface {
public:
    Interface();
    ~Interface();

    int configure();

    int stream();
    
#ifdef DEBUG
public:
    // mostly for debugging / development
    ssize_t get_euler_angles();
    ssize_t get_quaternion();
    ssize_t get_rotation_matrix();
#endif  // #ifdef DEBUG

private:
    int baud_rate;
    int dev;
    std::string path;

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

private:

    /// \brief serial-to-host conversion, floah a binary, big-endian representation of a float into a C-typed float variable
    float load_float( uint8_t* source, float& dest );

    /// \brief serial-to-host conversion, floah a binary, big-endian representation of a float into a C-typed float variable
    double byte4_to_double( uint8_t* source ){ float ret; return static_cast<double>(load_float(source, ret)); } 
    
    template<typename command_t>
    ssize_t command( command_t cmd){ return command( cmd.data(), cmd.size()); }

    ssize_t command( const uint8_t * command, ssize_t command_length );
    ssize_t receive( uint8_t* source, ssize_t len );

}; // class Interface

} // namespace IMU

#endif // #ifndef _IMU_INTERFACE_HPP_
