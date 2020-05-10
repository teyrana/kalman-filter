// GPL v3 (c) 2020, Daniel Williams 

// standard library includes
#include <iostream>
#include <iterator>
using std::cerr;
using std::cout;
using std::endl;

// project includes
#include "interface.hpp"

// #include "integrator.hpp"

int main()
{
    IMU::Interface imu;

    if(EXIT_FAILURE == imu.configure()){
        return EXIT_FAILURE;
    }

    cerr << "==== Starting Streams: ==== " << endl;
    if( EXIT_SUCCESS == imu.stream()){
        return EXIT_FAILURE;
    }

    // imu_source_start();

    return EXIT_SUCCESS;
}