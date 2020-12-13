// GPL v3 (c) 2020, Daniel Williams 

#ifndef _SERIAL_INTERFACE_HPP_
#define _SERIAL_INTERFACE_HPP_


// Serial API: POSIX termios
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
#include <fcntl.h>
#include <termios.h>


#include <spdlog/spdlog.h>

namespace Serial {

class Connection {
public:
    Connection();

    ~Connection();

    /// \brief sleep this connection until all outgoing bytes have been sent.
    void drain();

    /// \brief flush the input buffer
    void flush(int queue=TCIFLUSH);

    bool is_open() const;

    int open( const std::string& _path, uint32_t _baudrate);

    /// \brief receive a response of a given type (i.e. size)
    ///
    /// \param receive_buffer write received bytes into this buffer
    /// \param receive_count receive at most this many bytes into the above buffer
    /// \param start_timeout milliseconds to wait for the first byte of traffic
    /// \param duration_timeout milliseconds to wait for the remainder of bytes (from the first 
    ssize_t receive( uint8_t* receive_buffer, ssize_t receive_count, std::chrono::microseconds start_timeout, std::chrono::microseconds duration_timeout);

    ssize_t write( const uint8_t * write_buffer, ssize_t write_count );

public:
    
    /// \brief the driver will receive sereial data every 'poll_interval' microseconds
    // just a heuristic; arrived at empirically
    static constexpr useconds_t poll_interval = 25 * 1000;  // == 25 ms

private:
    uint32_t baud_rate_;

    int fd_;

    /// \brief handle for this class's logger
    spdlog::logger& log;

    std::string path_;

}; // class Interface

} // namespace IMU

#endif // #ifndef _IMU_INTERFACE_HPP_
