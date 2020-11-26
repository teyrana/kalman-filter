// GPL v3 (c) 2020, Daniel Williams 

// standard library includes
#include <iostream>
#include <iterator>
using std::cerr;
using std::cout;
using std::endl;

// project includes
#include "driver.hpp"

// #include "integrator.hpp"

int main()
{
    std::cout << "## Setting up IMU Connection..." << std::endl;
    IMU::Driver imu("/dev/ttyUSB0", 115200);

    if( IMU::Driver::IDLE != imu.state()){
        return EXIT_FAILURE;
    }

    imu.get_quaternion();

    imu.get_rotation_matrix();

    imu.get_euler_angles();

    return EXIT_SUCCESS;
}
