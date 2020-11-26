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

// 1st-Party. Program Includes
#include "connection.hpp"

using Serial::Connection;

Connection::Connection()
    : baud_rate_(0)
    , fd_(-1)      // error value
    , log(*spdlog::get("console"))
    , path_("")
{}

Connection::Connection( const std::string _path, uint32_t _baud)
    : baud_rate_(_baud)
    , fd_(-1)      // error value
    , log(*spdlog::get("console"))
    , path_(_path)
{
    open();
}

Connection::~Connection(){
    if( is_open() ){
        close(fd_);
    }
}

bool Connection::is_open() const {
    return (0 <= fd_);
}

int Connection::open( ){ 
    // https://en.wikibooks.org/wiki/Serial_Programming/termios

    if( (path_.empty()) || (0 == baud_rate_) ){
        return -1;
    }

    log.info("    >> Opening Serial port:");
    log.info("        Path: {}", path_ );
    //open serial port
    fd_ = ::open( path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if (fd_ < 0) {
        log.error("    !!!! Error: could not open serial port !!!!");
        return -1;
    }

    log.info("    >> Configuring serial port:");
    log.info("        Baud: {}", baud_rate_ );

    struct termios tty;
    if (tcgetattr (fd_, &tty) != 0) {
        log.error( "Could not set serial settings: {}: {}", errno, strerror(errno) );
        fd_ = -1;
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
        log.error( "Could not set serial settings: {}: {}", errno, strerror(errno) );
        return -1;
    }

    return 0;
}

int Connection::open( const std::string& _path, uint32_t _baud_rate){ 
    if( _path.empty() || 0 == _baud_rate){
        return -1;
    }

    path_ = _path;
    baud_rate_ = _baud_rate;

    return open();
}

ssize_t Connection::receive( uint8_t* receive_buffer, ssize_t receive_count ){
    if( nullptr == receive_buffer ){
        log.error("Error::Serial::receive::({}) -- null receive_buffer", EFAULT); 
        return -1;
    }else{
        // Rule of thumb.  Not strictly necessary, but helps to lessen the I/O load
        usleep( 10000 ); // wait 10ms, at first
    }

    // std::cerr << "## 2: Waiting for " << receive_count << " bytes..." << std::endl;
    uint8_t * receive_at = receive_buffer;
    uint8_t * receive_until = receive_buffer + receive_count;
    ssize_t receive_for = receive_count;
    ssize_t bytes_read = 0;

    while( receive_at < receive_until){
        bytes_read = read( fd_, receive_at, receive_for);
        // std::cerr << "    >> 3: 'read' return: " << bytes_read << " bytes." << std::endl;

        if( -1 == bytes_read ){
            usleep( 2500 );  // just a heuristic; arrived at empirically
            continue;
        
        }else if (bytes_read == EAGAIN || bytes_read == EWOULDBLOCK) {
            usleep( 2500 );  // just a heuristic; arrived at empirically
            // std::cerr << "Warning:  socket should be non-blocking... EAGAIN / EWOULDBLOCK return from 'read' call..." << std::endl;
            continue;

        }else if( 0 < bytes_read ) {
            // std::cerr << "    >> 4: Received " << bytes_read << " bytes." << std::endl;
            receive_at += bytes_read;
            receive_for -= bytes_read;
            
            // make sure we don't overshoot our goal
            if( 0 < receive_for ){
                return -1;
            }
            
            if( receive_at >= receive_until ) {
                // log.errorstderr, "    << 6. Received all bytes (%ld); returning.\n", bytes_read);
                return receive_count;
            }
            continue;
        
        } else {
            log.error("Error::Serial::unknown error::({}): {}", errno, strerror(errno) );
            return -1;
        }
    }

    // Unknown state! Control should not reach here... 
    // but may, if too many bytes are requested.
    return -1;
}

ssize_t Connection::write( const uint8_t * write_buffer, ssize_t write_count ){
    auto bytes_written = ::write( fd_, write_buffer, write_count );
    if( write_count != bytes_written ){
        log.error("Error::Serial::write::({}): {}", errno, strerror(errno) );
    }
    
    return bytes_written;
}