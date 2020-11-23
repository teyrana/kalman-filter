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
#include "connection.hpp"

using IMU::Connection;

Connection::Connection()
    : baud_rate_(9600)
    , fd_(-1)
    , path_("/dev/ttyS0")
    , stream_interval( 250 * 1000)    // 250 ms
    , stream_duration( 25000 * 1000)  // 2.5s
    , stream_delay( 1000 )            // minimum delay
    , stream_receive_buffer({0})
    , state_(ERROR)
{

}

Connection::Connection( const std::string _path, uint32_t _baud)
    : baud_rate_(_baud)
    , fd_(-1)      // error value
    , path_(_path)
    , stream_interval( 250 * 1000)    // 250 ms
    , stream_duration( 25000 * 1000)  // 2.5s
    , stream_delay( 1000 )            // minimum delay
    , stream_receive_buffer({0})
    , state_(ERROR)
{
    if( 0 != open(path_, baud_rate_) ){
        state_ = ERROR;
        return;
    }
    
    if( 0 != configure() ){
        state_ = ERROR;
        return;
    }

    state_ = IDLE;
}

Connection::~Connection(){

    if( STREAM == state_ ){
        const auto bytes_written = write( stop_stream_command );

        // ensure the command is sent, before we close the socket:
        usleep( bytes_written * 10 );  // nominal write speed: <10 usec/byte
    }

    if( 0 <= fd_){
        close(fd_);
    }
}

int Connection::configure(){
    if( 0 >= fd_){
        return -1;
    }

    fprintf(stderr, ">> Configuring serial device...\n");

    
    {
        auto set_axis_directions_command = Command<1>( 116, 0, nullptr );
#ifdef REMAP_AXES
        // Remap axes: from: "Natural Axes" (hardware default)
        //               to: Conventional Axes (UUV Literature)
        // enum for: right-up-forward => forward-right-down
        set_axis_directions_command.data(0) = 0x03 | 0x02; 
#else
        // explicitly set to hardware defaults:
        set_axis_directions_command.data(0) = 0;
#endif 
        set_axis_directions_command.pack();
        auto bytes_written = write( set_axis_directions_command );
        usleep( bytes_written * 10 );
    }{
        // set euler angle order:

    }

    state_ = IDLE;
    return 0;
}

int Connection::monitor(){

    while( STREAM == state_ ){
        ssize_t bytes_read = receive( stream_receive_buffer.data(), stream_receive_buffer.size() );

        if (0 == bytes_read) {
            fprintf(stderr, "    !! Error while streaming!! \n");
            fprintf(stderr, "    !! [%d]: %s !!\n", errno, strerror(errno) );
            state_ = ERROR;
        }

        if(expected_receive_bytes != bytes_read){
            continue;
        } else { 
            // TODO: implement happy path here

        }
    }

    return -1;  // error return path
}

int Connection::open( const std::string& _path, uint32_t _baudrate){ 
    // https://en.wikibooks.org/wiki/Serial_Programming/termios

    if( _path.empty() || 0 == _baudrate){
        return -1;
    }
    path_ = _path;
    baud_rate_ = _baudrate;

    std::cout << "    >> Opening Serial port:\n"
              << "        Path: " << path_ << std::endl;
    //open serial port
    fd_ = ::open( path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if (fd_ < 0) {
        std::cerr << "    !!!! Error: could not open serial port !!!!" << std::endl;
        state_ = ERROR;
        return -1;
    }

    std::cout << "    >> Configuring serial port:\n"
              << "        Baud: " << baud_rate_ << std::endl;

    struct termios tty;
    if (tcgetattr (fd_, &tty) != 0) {
        std::cerr << "error " << errno << '(' << strerror(errno) << ") from tcgetattr" << std::endl;
        return -1;
    }
    
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

    }{ // output specific flags (bitmask)
    //     tty.c_oflag = 0;                // no remapping, no delays

    }{ // control flags (bitmask)
        tty.c_cflag = 0;

        {
            const speed_t configure_baud_rate = (
                            (baud_rate_==  9600) ?   B9600:
                            (baud_rate_== 19200) ?  B19200:
                            (baud_rate_== 38400) ?  B38400:
                            (baud_rate_== 57600) ?  B57600:
                            (baud_rate_==115200) ? B115200:
                                                        B0 );

            // these modify the 'c_cflag' field:
            cfsetospeed (&tty, configure_baud_rate );
            cfsetispeed (&tty, configure_baud_rate );
        }

        // tty.c_cflag &= ~CSIZE;      // CHARACTER_SIZE_MASK
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

    if (tcsetattr (fd_, TCSANOW, &tty) != 0) {
        std::cerr << "::Error::Serial::Configure:" << errno << " from tcsetattr" << std::endl;
        return -1;
    }

    state_ = STARTUP;
    return 0;
}

ssize_t Connection::receive( uint8_t* receive_buffer, ssize_t receive_length ){

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
        bytes_read = read( fd_, receive_at, receive_for);
        // std::cerr << "    >> 3: 'read' return: " << bytes_read << " bytes." << std::endl;

        if( -1 == bytes_read ){
            usleep( 2500 );  // just a heuristic; arrived at empirically
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
    // but may, if too many bytes are requested.
    return -1;
}

float Connection::load_float( uint8_t* source, float& dest ){

    uint8_t* raw_float_bytes = reinterpret_cast<uint8_t*>(&dest);

    raw_float_bytes[0] = source[3];
    raw_float_bytes[1] = source[2];
    raw_float_bytes[2] = source[1];
    raw_float_bytes[3] = source[0];

    return dest;
}


ssize_t Connection::get_euler_angles(){

    std::cerr << "## 1: Requesting Euler Angles: (Filtered, Tared) " << std::endl;
    if( 0 > write(IMU::Connection::readSingleEulerAnglesCommand) ) {
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

ssize_t Connection::get_quaternion(){

    // DEBUG
    std::cerr << "## 1: Requesting Quaternion: (Filtered, Tared)" << std::endl;
    if( 0 > write(IMU::Connection::readSingleQuaternionCommand) ) {
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


ssize_t Connection::get_rotation_matrix(){

    // DEBUG 
    std::cerr << "## 1: Requesting Rotation Matrix: (Filtered, Tared) " << std::endl;
    if( 0 > write(IMU::Connection::readSingleRotationMatrixCommand) ) {
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

int Connection::stream() {
    ssize_t bytes_written = 0;

    // (1) Set up the streaming slots to stream the desire commands:
    IMU::Command<8> slot_command(0x50, 0, nullptr);
    slot_command[2] = 1;
    memset( const_cast<uint8_t*>(slot_command.data() + 3), 0xFF, 7 );
    slot_command.pack();
    bytes_written = write( slot_command );
    fprintf(stdout, ">>>> Wrote Streaming Slots.\n");
    // slot_command.fprinth(stdout);
    usleep( bytes_written * 5000 );

    // (2) Set streaming timing:
    IMU::Command<12> timing_command(0x52, 0, nullptr);
    timing_command.store_int32_data( stream_interval, 0 );
    timing_command.store_int32_data( stream_duration, 4 );
    timing_command.store_int32_data( stream_delay, 8 );
    timing_command.pack();

    bytes_written = write( timing_command );
    fprintf(stdout, ">>>> Wrote stream Timing.\n");
    usleep( bytes_written * 5000 );

    // // (3) Start streaming, using the current configuration:
    bytes_written = write( start_stream_command );
    fprintf(stdout, ">>>> Wrote Start-Streaming Command.\n");
    usleep( bytes_written * 5000 );

    return 0;
}

ssize_t Connection::write( const uint8_t* command, const ssize_t command_length ){

    auto bytes_written = ::write( fd_, command, command_length);
    
    if( command_length != bytes_written){
        std::cerr << "Error::Serial::write::(" << errno << "): " << strerror(errno) << std::endl;
        return bytes_written;
    }

    return bytes_written;
}