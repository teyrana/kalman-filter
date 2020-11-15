// GPL v3 (c) 2020, Daniel Williams 

// Standard Library Includes
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>

// Posix / Linux System Includes
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// 3rd-Party Library Includes
#include <Eigen/Geometry> 

// 1st-Party. Program Includes
#include "interface.hpp"

using IMU::Interface;

IMU::Interface::Interface()
    : baud_rate(B115200)
    , path("/dev/ttyUSB0")
{ 
    std::cout << "## Setting up IMU..." << std::endl;
}

Interface::~Interface(){
    close(dev);
}

int Interface::configure() {
    std::cerr << "....## Opening IMU serial port:" << std::endl;
    std::cout << "    path:  " << path << std::endl;
    std::cout << "    baud:  " << (baud_rate==B115200?"B115200":"??") << std::endl;

    //open serial port
    dev = open( path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if (dev < 0) {
        std::cout << "!! Error could not open serial port: " << path << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr (dev, &tty) != 0) {
        std::cerr << "error " << errno << '(' << strerror(errno) << ") from tcgetattr" << std::endl;
        return -1;
    }

    // https://en.wikibooks.org/wiki/Serial_Programming/termios
    
    cfsetospeed (&tty, baud_rate);
    cfsetispeed (&tty, baud_rate);

    // struct fields are populated in order-of-definition in `struct termios`:
    { // input specific flags (bitmask)
        tty.c_iflag = 0;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff flow control

        // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
        tty.c_iflag |= IGNBRK;     // ignore break processing
        tty.c_iflag |= IGNPAR;      // Ignore bytes with parity errors
        tty.c_iflag |= ICRNL;       // map CR to NL (otherwise a CR input on the other computer
                                    // will not terminate input)
        tty.c_iflag |= IGNCR;      // ignore carriage return on input

    // }{ // output specific flags (bitmask)
    //     tty.c_oflag = 0;                // no remapping, no delays

    // }{ // control flags (bitmask)
    //     tty.c_cflag = 0;
    //     tty.c_cflag &= ~CSIZE;      // CHARACTER_SIZE_MASK
        tty.c_cflag |= CS8;         // 8-bit characters
        tty.c_cflag &= ~PARENB;     // no parity bit
        tty.c_cflag &= ~CSTOPB;     // only need 1 stop bit
        tty.c_cflag &= ~CRTSCTS;    // no hardware flowcontrol
        tty.c_cflag |= CLOCAL;      // ignore modem controls,
        tty.c_cflag |= CREAD;       // enable reading

    }{ // local flags (bitmask)
        tty.c_lflag = 0; // non-canonical mode; no echo

    }{ // special characters
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout
    
    }

    // tcflush(dev, TCIFLUSH);

    if (tcsetattr (dev, TCSANOW, &tty) != 0) {
        std::cerr << "::Error::Serial::Configure:" << errno << " from tcsetattr" << std::endl;
        return -1;
    }
    return 0;
}

Eigen::Quaternionf& Interface::load_quaternion( uint8_t* source, Eigen::Quaternionf& dest ){
    dest.w() = load_float(source + 0);
    dest.x() = load_float(source + 4);
    dest.y() = load_float(source + 8);
    dest.z() = load_float(source + 12);

    return dest;
}

float Interface::load_float( uint8_t* source ){
    // note that ALL binary data returned from the 
    float return_value;

    uint8_t* dest = reinterpret_cast<uint8_t*>(&return_value);

    dest[0] = source[3];
    dest[1] = source[2];
    dest[2] = source[1];
    dest[3] = source[0];

    return return_value;
}

int Interface::stream() {
    std::cerr << ".... ## Starting IMU data stream:" << std::endl;
 
    // { // DEBUG
    //     readSingleQuaternionCommand.fprinth(stderr);
    //     return EXIT_FAILURE;
    // } // DEBUG

    const uint8_t* command = readSingleQuaternionCommand.data();
    constexpr size_t command_length = readSingleQuaternionCommand.size();

    std::cout << "requesting quaternion: [";
    readSingleQuaternionCommand.fprinth(stdout);
    std::cout << "]:" << command_length << std::endl;

    // std::cout << "write() [Serial_communication] started with command: " << command << std::endl;
    if( 0 > write( dev, command, command_length)) {
        std::cout << "Error::Serial::write::(" << errno << "): " << strerror(errno) << std::endl;
        return -1;
    }
    tcflush(dev, TCIFLUSH);

    // empirical.  Less than about this, and not enough bytes are received....
    usleep( 15000 );

    constexpr size_t receive_length = 32;
    char receive_buffer[receive_length];

    auto bytes_read = read( dev, receive_buffer, receive_length);
    // if (bytes_read == 0) {
    //     break;
    // } else if (bytes_read > 0) {
    //     response[spot1++] = buf;
    // } else if (bytes_read == EAGAIN || bytes_read == EWOULDBLOCK)
    //     continue;
    // } else { /*unrecoverable error */
    //     perror("Error reading");
    //     break;
    // }
    if( 0 >= bytes_read){
        std::cout << "Error::Serial::read::(" << errno << "): " << strerror(errno) << std::endl;
    }else{
        fprintf(stdout, ">> Read %ld bytes:\n", bytes_read);
        if(16 == bytes_read){
            Eigen::Quaternionf orientation(0,0,0,0);
            /* ? = */ load_quaternion( reinterpret_cast<uint8_t*>(receive_buffer), orientation );

            fprintf( stdout, "    >> Received Quaternion: \n");
            fprintf( stdout, "        :w: %+f\n", orientation.w() );
            fprintf( stdout, "        :x: %+f\n", orientation.x() );
            fprintf( stdout, "        :y: %+f\n", orientation.y() );
            fprintf( stdout, "        :z: %+f\n", orientation.z() );
        }

    } // DEBUG

    //SENDING FINISHED
    std::cout << "write() [serial_communcation] finished..." << std::endl;
    return 1;
}