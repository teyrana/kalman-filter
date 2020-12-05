// GPL v3 (c) 2020, Daniel Williams 

#ifndef _SERIAL_INTERFACE_HPP_
#define _SERIAL_INTERFACE_HPP_

// Serial API
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

#include <spdlog/spdlog.h>

namespace Serial {

class Connection {
public:
    Connection();
    Connection(std::string _path, uint32_t _baudrate);
    ~Connection();

    bool is_open() const;

    int open();
    int open( const std::string& _path, uint32_t _baudrate);

    ssize_t receive( uint8_t* receive_buffer, ssize_t receive_count );

    ssize_t write( const uint8_t * write_buffer, ssize_t write_count );

private:
    uint32_t baud_rate_;

    int fd_;

    /// \brief handle for this class's logger
    spdlog::logger& log;

    std::string path_;

}; // class Interface

} // namespace IMU

#endif // #ifndef _IMU_INTERFACE_HPP_
