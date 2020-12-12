// GPL v3 (c) 2020, Daniel Williams 


#ifndef _SERIAL_MESSAGE_HPP_
#define _SERIAL_MESSAGE_HPP_

#include <array>
#include <cmath>

// 3rd-Party Library Includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

// 3rd-Party Library Includes
#include "spdlog/spdlog.h"


namespace IMU {

template<size_t payload_length_>
class Response {
public:

    Response() = default;

    // constexpr Message(uint8_t b0, ... )
    //     : buffer({sync_byte, command_byte, data_byte})
    // {}

    ~Response() = default;

    void clear() { buffer.fill(0); }

    uint8_t* data() { return buffer.data(); }
    
    uint8_t const * data() const { return buffer.data(); }

    // convenience method specifically to set data bytes:
    uint8_t& data(size_t index ){
        return buffer[ payload_index + index];
    }

    static constexpr uint8_t flags() { return response_flags_; }

    void log( spdlog::logger& log, spdlog::level::level_enum level, const std::string& preamble="") const;

    // constexpr uint8_t* operator*() { return buffer.data(); }

    // constexpr const uint8_t* operator*() const { return buffer.data(); }

    constexpr uint8_t& operator[](size_t index ){ return buffer[index]; }
    constexpr const uint8_t& operator[](size_t index ) const { return buffer[index]; }

    /// \brief read a value, starting from the given data index 
    float_t read_float32( size_t index ) const;

    /// \brief read a value, starting from the given data index 
    double_t read_float64( size_t index ) const;

    /// \brief read a value, starting from the given data index 
    int32_t read_int32( size_t index ) const;

    /// \brief read a value, starting from the given data index 
    uint32_t read_uint32( size_t index ) const;
    
    Eigen::Vector3d& read_vector3d( size_t index, Eigen::Vector3d& dest) const;

    Eigen::Vector3i& read_vector3i( size_t index, Eigen::Vector3i& dest) const;

    Eigen::Vector4d& read_vector4d( size_t index, Eigen::Vector4d& dest) const;

    constexpr size_t size() const { return buffer.size(); }

    // 0x1 (Bit 0) – Success/Failure; comprised of one byte with non-zero values indicating failure.
    constexpr bool provides_success() const { return (response_flags_& SUCCESS_FLAG); }
    bool success() const { return (0==buffer[success_index]); }

    // 0x2 (Bit 1) – Timestamp -- uint32; milliseconds since hardware power-up 
    constexpr bool provides_timestamp() const { return (response_flags_& TIMESTAMP_FLAG); }

    /// \brief retreives response timestamp -- uint32; milliseconds since hardware power-up 
    uint32_t timestamp() const;

    // 0x4 (Bit 2) – Command Echo -- echos the same command as sent:
    constexpr bool provides_command() const { return (response_flags_& COMMAND_ECHO_FLAG); }
    uint8_t command() const { return (provides_command()?buffer[command_echo_index]:0xFF); }


private:  // private member functions

    static void read_chunk32( const uint8_t* const read_bytes, uint8_t * write_bytes);

private: // private static constexpr properties
    // these flags enable various header fields. They are defined in the IMU hardware manuual

    // 221 (0xDD) Set response Header
    //     0x1 (Bit 0) – Success/Failure; comprised of one byte with non-zero values indicating failure.
    static constexpr uint8_t SUCCESS_FLAG = 0x01;
    //     0x2 (Bit 1) – Timestamp; comprised of four bytes representing the most recent sample time in
    //                   microseconds. Note that this is not a difference, but a total accumulated time.
    static constexpr uint8_t TIMESTAMP_FLAG = 0x02;
    //     0x4 (Bit 2) – Command echo; comprised of one byte. Echoes back the previous command.
    static constexpr uint8_t COMMAND_ECHO_FLAG = 0x04;

    // these are the flags the code will actually configure
    static const constexpr uint8_t response_flags_ = SUCCESS_FLAG | TIMESTAMP_FLAG | COMMAND_ECHO_FLAG;

    // header section
    static constexpr size_t header_index = 0; // by definition
    static constexpr size_t success_index = header_index;
    static constexpr size_t timestamp_index = header_index + ((response_flags_&SUCCESS_FLAG)?1:0);
    static constexpr size_t command_echo_index = timestamp_index + ((response_flags_&TIMESTAMP_FLAG)?4:0);

    // // other header-fields NYI !
    static constexpr size_t next_index = command_echo_index + ((response_flags_&COMMAND_ECHO_FLAG)?1:0);

    // static constexpr uint8_t header_???_flag = ...
    // static constexpr uint8_t header_command_echo_index = header_index + ....

    static constexpr size_t header_length = next_index;

    // payload section
    static constexpr size_t payload_index = header_length;
    static constexpr size_t payload_length = payload_length_;

private:  // private instance attributes

    std::array<uint8_t, header_length + payload_length> buffer;


}; // class Serial::Message
} // namespace Serial

// to include the template-function definitions
#include "response.inl"

#endif // #ifndef _SERIAL_MESSAGE_HPP_
