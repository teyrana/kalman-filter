// GPL v3 (c) 2020, Daniel Williams 

// standard library includes
#include <iostream>
#include <iterator>
using std::cerr;
using std::cout;
using std::endl;

// project includes
#include "connection.hpp"

// #include "integrator.hpp"

int main()
{
    IMU::Connection imu;

    if(IMU::Connection::IDLE != imu.state()){
        return EXIT_FAILURE;
    }

    cerr << "==== Starting Streams: ==== " << endl;
    if( EXIT_SUCCESS == imu.stream()){
        // imu.monitor();
    } else {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
