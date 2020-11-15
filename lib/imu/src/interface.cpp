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
#include <Eigen/Dense>

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
    std::cout << "    baud:  " << (baud_rate==  B9600?  "9600":
                                  (baud_rate==B115200?"115200":
                                    "??"))
                               << std::endl;

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

ssize_t Interface::command( const uint8_t* command, const ssize_t command_length ){

    auto bytes_written = write( dev, command, command_length);
    
    if( command_length != bytes_written){
        std::cerr << "Error::Serial::write::(" << errno << "): " << strerror(errno) << std::endl;
        return bytes_written;
    }

    return bytes_written;
}

ssize_t Interface::receive( uint8_t* receive_buffer, ssize_t receive_length ){

    if( nullptr == receive_buffer ){
        std::cerr << "Error::Serial::receive::(" << EFAULT << "): for destination data pointer: " << strerror(EFAULT) << std::endl;
        return -1;
    }else{
        // Rule of thumb.  Not strictly necessary, but helps to lessen the I/O load
        usleep( 10000 ); // wait 10ns, at first
    }

    // std::cerr << "## 2: Waiting for data... " << std::endl;
    uint8_t * receive_at = receive_buffer;
    uint8_t * receive_until = receive_buffer + receive_length;
    ssize_t receive_for = receive_length;
    ssize_t bytes_read = 0;

    while( receive_at < receive_until){
        bytes_read = read( dev, receive_at, receive_for);
        // std::cerr << "    >> 3: 'read' return: " << bytes_read << " bytes." << std::endl;

        if( -1 == bytes_read ){
            // empirical.  Not actually necessary, but lightens the load, instead of spinning on the i/o call ...
            usleep( 2500 );
            continue;
        
        }else if (bytes_read == EAGAIN || bytes_read == EWOULDBLOCK) {
            std::cerr << "Warning:  socket should be non-blocking... EAGAIN / EWOULDBLOCK return from 'read' call..." << std::endl;
            continue;

        }else if( 0 < bytes_read ) {
            // std::cerr << "    >> 4: Received " << bytes_read << " bytes." << std::endl;
            receive_at += bytes_read;
            receive_for -= bytes_read;

            // make sure we don't overshoot our goal
            assert( 0 <= receive_for );
            
            if( receive_at >= receive_until ) {
                // fprintf(stderr, "    << 5. Received all bytes (%ld); returning.\n", bytes_read);
                return receive_length;
            }
            continue;
        
        } else {
            std::cerr << "Error::Serial::unknown error::(" << errno << "): " << strerror(errno) << std::endl;
            abort();
        }
    }

    // Unknown state! Control should not reach here...
    assert(false);
}

float Interface::load_float( uint8_t* source, float& dest ){

    uint8_t* raw_float_bytes = reinterpret_cast<uint8_t*>(&dest);

    raw_float_bytes[0] = source[3];
    raw_float_bytes[1] = source[2];
    raw_float_bytes[2] = source[1];
    raw_float_bytes[3] = source[0];

    return dest;
}


ssize_t Interface::get_euler_angles(){

    std::cerr << "## 1: Requesting Euler Angles: (Filtered, Tared) " << std::endl;
    if( 0 > command(IMU::Interface::readSingleEulerAnglesCommand) ) {
        std::cerr << "    <<!!: Command Error... abort!" << std::endl;
        abort();
    }

    // to: receive-buffer:
    constexpr size_t receive_length = 12;
    uint8_t receive_buffer[receive_length];
    ssize_t bytes_received = 0;
    if( 0 > (bytes_received = receive( receive_buffer, receive_length))){
        std::cerr << "    <<!!: Receive Error... abort!" << std::endl;
        abort();
    }

    fprintf(stdout, ">> Received Euler Angles (%+ld bytes).\n", bytes_received);

    for( size_t i = 0; i < receive_length; ++i){
        if( 0 == i%8 ){
            fputc(' ', stderr);
        }
        fprintf(stderr, "%02X ", receive_buffer[i]);
    }
    fputc('\n', stderr);
     

    Eigen::Vector3d euler_angles(0,0,0);
    euler_angles[0] = byte4_to_double(receive_buffer + 4 );
    euler_angles[1] = byte4_to_double(receive_buffer + 0 );
    euler_angles[2] = byte4_to_double(receive_buffer + 8 );

    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        >> angle[0]:  %+f\n", euler_angles[0] );
    fprintf( stdout, "        >> angle[1]:  %+f\n", euler_angles[1] );
    fprintf( stdout, "        >> angle[2]:  %+f\n", euler_angles[2] );
    
    return bytes_received;
}

ssize_t Interface::get_quaternion(){

    // DEBUG
    std::cerr << "## 1: Requesting Quaternion: (Filtered, Tared)" << std::endl;
    if( 0 > command(IMU::Interface::readSingleQuaternionCommand) ) {
        std::cerr << "    <<!!: Command Error... abort!" << std::endl;
        abort();
    }

    // to: receive-buffer:
    constexpr size_t receive_length = 16;
    uint8_t receive_buffer[receive_length];
    ssize_t bytes_received = 0;
    if( 0 > (bytes_received = receive( receive_buffer, receive_length))){
        std::cerr << "    <<!!: Receive Error... abort!" << std::endl;
        abort();
    }

    Eigen::Quaternionf orientation_raw(0,0,0,0);
    // confirmed that the return order is: [X,Y,Z,W]
    /* _ = */ load_float(receive_buffer + 0, orientation_raw.x());
    /* _ = */ load_float(receive_buffer + 4, orientation_raw.y());
    /* _ = */ load_float(receive_buffer + 8, orientation_raw.z());
    /* _ = */ load_float(receive_buffer + 12, orientation_raw.w());

    fprintf( stdout, "    >> Received Quaternion:    (%+ld bytes).\n", bytes_received);

    Eigen::Quaterniond orientation_expanded( orientation_raw );
    fprintf( stdout, "        :w: %+f\n", orientation_expanded.w() );
    fprintf( stdout, "        :x: %+f\n", orientation_expanded.x() );
    fprintf( stdout, "        :y: %+f\n", orientation_expanded.y() );
    fprintf( stdout, "        :z: %+f\n", orientation_expanded.z() );

    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Matrix3d rotation = orientation_expanded.toRotationMatrix();
    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(0,0), rotation(0,1), rotation(0,2) );
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(1,0), rotation(1,1), rotation(1,2) );
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(2,0), rotation(2,1), rotation(2,2) );
    
    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Vector3d euler_angles = rotation.eulerAngles( 1, 0, 2);

    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        >> angle[0]:  %+f\n", euler_angles[0] );
    fprintf( stdout, "        >> angle[1]:  %+f\n", euler_angles[1] );
    fprintf( stdout, "        >> angle[2]:  %+f\n", euler_angles[2] );

    return bytes_received;
}


ssize_t Interface::get_rotation_matrix(){

    // DEBUG 
    std::cerr << "## 1: Requesting Rotation Matrix: (Filtered, Tared) " << std::endl;
    if( 0 > command(IMU::Interface::readSingleRotationMatrixCommand) ) {
        std::cerr << "    <<!!: Command Error... abort!" << std::endl;
        abort();
    }

    // to: receive-buffer:
    constexpr size_t receive_length = 36;
    uint8_t receive_buffer[receive_length];
    ssize_t bytes_received = 0;
    if( 0 > (bytes_received = receive( receive_buffer, receive_length))){
        std::cerr << "    <<!!: Receive Error... abort!" << std::endl;
        abort();
    }

    fprintf(stdout, ">> Received Rotation Matrix (%+ld bytes).\n", bytes_received);

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    rotation(0,0) = byte4_to_double(receive_buffer + 0 );
    rotation(0,1) = byte4_to_double(receive_buffer + 4 );
    rotation(0,2) = byte4_to_double(receive_buffer + 8 );
    rotation(1,0) = byte4_to_double(receive_buffer + 12);
    rotation(1,1) = byte4_to_double(receive_buffer + 16);
    rotation(1,2) = byte4_to_double(receive_buffer + 20);
    rotation(2,0) = byte4_to_double(receive_buffer + 24);
    rotation(2,1) = byte4_to_double(receive_buffer + 28);
    rotation(2,2) = byte4_to_double(receive_buffer + 32);

    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(0,0), rotation(0,1), rotation(0,2) );
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(1,0), rotation(1,1), rotation(1,2) );
    fprintf( stdout, "        [ %+f, %+f, %+f ]\n", rotation(2,0), rotation(2,1), rotation(2,2) );

    // composition order of euler angles: YXZ
    // ... in theory, this corresponds to: 2-1-3 => 1, 0, 2 .... but this doesn't produce the right anwsers :(
    Eigen::Vector3d euler_angles = rotation.eulerAngles( 1, 0, 2);

    fprintf( stdout, "    >> Converted to Euler Angles: \n");
    fprintf( stdout, "        >> angle[0]:  %+f\n", euler_angles[0] );
    fprintf( stdout, "        >> angle[1]:  %+f\n", euler_angles[1] );
    fprintf( stdout, "        >> angle[2]:  %+f\n", euler_angles[2] );

    return bytes_received;
}
int Interface::stream() {

    // 1) Set up the streaming to call the commands you want data from:
    std::cerr << "## (1) Configure Streams: " << std::endl;
    // 0(0x00), Read tared orientation as quaternion

    std::cerr << ".... ## Starting IMU data stream:" << std::endl;
 
    // { // DEBUG
    //     readSingleQuaternionCommand.fprinth(stderr);
    //     return EXIT_FAILURE;
    // } // DEBUG

    get_quaternion();

    //SENDING FINISHED
    std::cout << "write() [serial_communcation] finished..." << std::endl;
    return 1;
}