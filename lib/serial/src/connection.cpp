// GPL v3 (c) 2020, Daniel Williams 

// Standard Library Includes
#include <chrono>
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

Connection::~Connection(){
    if( is_open() ){
        close(fd_);
    }
}

void Connection::drain(){
    ::tcdrain(fd_);
}

void Connection::flush(int _queue_selector){
    tcflush( fd_, _queue_selector);
}

bool Connection::is_open() const {
    return (0 <= fd_);
}


int Connection::open( const std::string& _path, uint32_t _baud_rate){ 
    if( _path.empty() || 0 == _baud_rate){
        return -1;
    }

    path_ = _path;
    baud_rate_ = _baud_rate;
 
    // https://en.wikibooks.org/wiki/Serial_Programming/termios
    log.info("    >> Opening Serial port:");
    log.info("        Path: {}", path_);
    //open serial port
    fd_ = ::open( path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if ( -1 == fd_ ) {
        log.error("    !!!! Error: could not open serial port !!!!");
        return -1;
    }

    log.info("    >> Configuring serial port:");
    log.info("        Baud: {}", baud_rate_);

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

ssize_t Connection::receive( uint8_t* receive_buffer, ssize_t receive_count, 
                             std::chrono::microseconds start_timeout, std::chrono::microseconds duration_timeout)
{
    if( nullptr == receive_buffer ){
        log.error("Error::Serial::receive::({}) -- null receive_buffer", EFAULT); 
        return -1;
    }
    auto call_time = std::chrono::steady_clock::now();
    auto end_timeout = start_timeout + duration_timeout;

    // Rule of thumb.  Not strictly necessary, but helps to lessen the I/O load
    usleep( poll_interval );

    // fmt::print(stderr, "    ## 2: Waiting for {} bytes...\n", receive_count );
    // fmt::print(stderr, "        >> start_timeout => {} ms \n", std::chrono::duration_cast<std::chrono::milliseconds>(start_timeout).count() );
    // fmt::print(stderr, "        >> end_timeout  =>  {} ms \n", std::chrono::duration_cast<std::chrono::milliseconds>(end_timeout).count() );
    uint8_t * receive_at = receive_buffer;
    const uint8_t * const receive_until = receive_buffer + receive_count;
    ssize_t receive_for = receive_count;
    ssize_t bytes_read = 0;

    // receive until we've filled up the buffer
    while( receive_at < receive_until){

        bytes_read = read( fd_, receive_at, receive_for);
        auto read_time = std::chrono::steady_clock::now() - call_time;
        // fmt::print( stderr, "        >> 3: read return: {} bytes @ t={} ms \n", bytes_read, std::chrono::duration_cast<std::chrono::milliseconds>(read_time).count() );

        if( receive_buffer == receive_at){
            if( start_timeout < read_time ) {
                log.error("<<<< !! Timed out before first byte @ {} ms !!\n", std::chrono::duration_cast<std::chrono::milliseconds>(read_time).count() );
                errno = ETIMEDOUT;
                return -2;
            }if( (0 < bytes_read)){
                // fmt::print(stderr, "        >> 4: Received first byte @ {} ms \n", std::chrono::duration_cast<std::chrono::milliseconds>(read_time).count() );
                // if this is the first received byte, record the a new 'first_byte_time':
                // this should _always_ result in pushing the 'first_byte_time' earlier.
                end_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(read_time) + duration_timeout;
                // fmt::print(stderr, "            ## update end_timeout => {} ms \n", std::chrono::duration_cast<std::chrono::milliseconds>(end_timeout).count() );
            }
        }else if( end_timeout < read_time ){
            log.error("<<<< !! Timed out before last byte @ {} ms !!\n", std::chrono::duration_cast<std::chrono::milliseconds>(read_time).count() );
            errno = ETIMEDOUT;
            return -2;
        }

        if( -1 == bytes_read ){
            usleep(poll_interval);
            continue;
        }

        if( 0 < bytes_read ) {
            receive_at += bytes_read;
            // receive_for -= bytes_read;
            receive_for = (receive_until - receive_at);
           
            if( receive_at >= receive_until ) {
                // fmt::print(stderr, "        << 6. Received all bytes ({}) @ {} ms; returning.\n", (receive_at - receive_buffer), 
                //                     std::chrono::duration_cast<std::chrono::milliseconds>(read_time).count() );
                return receive_count;
            }
            continue;
        
        } else {
            log.error("Error::Serial::unknown error::({}): {}", errno, strerror(errno) );
            return -1;
        }
    }

    log.error("??? how are we getting here? ({}) {}\n", errno, strerror(errno) );

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