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
    
private:
    int baud_rate;
    int dev;
    std::string path;

    static constexpr Command<0> readSingleQuaternionCommand = Command<0>(0);

private:
    Eigen::Quaternionf& load_quaternion( uint8_t* source, Eigen::Quaternionf& dest );

    /// \brief serial-to-host conversion, floah a binary, big-endian representation of a float into a C-typed float variable
    float load_float( uint8_t* source );

}; // class Interface

} // namespace IMU

#endif // #ifndef _IMU_INTERFACE_HPP_
