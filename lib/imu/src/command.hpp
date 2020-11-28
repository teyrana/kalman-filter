// GPL v3 (c) 2020, Daniel Williams 

#ifndef _IMU_COMMAND_HPP_
#define _IMU_COMMAND_HPP_

#include <array>
#include <cmath>
#include <cstring>

// 3rd-Party Library Includes
#include <spdlog/spdlog.h>


namespace IMU {

template<size_t payload_length, size_t response_length>
class Command {

    static constexpr uint8_t header_length = 2;
    static constexpr uint8_t footer_length = 1;

    // header fields
    // static constexpr uint8_t sync_byte_value = 0xF7;  // normal
    static constexpr uint8_t sync_byte_value = 0xF9;  // requests response header
    static constexpr uint8_t sync_byte_index = 0;
    static constexpr uint8_t command_id_index = 1;

    // payload field
    static constexpr uint8_t payload_index = header_length;

    // footer fields
    static constexpr uint8_t footer_index = header_length + payload_length;
    static constexpr uint8_t checksum_index = footer_index;

public:
    Command() = delete;

    constexpr Command(uint8_t command_id)
        // optimized for zero-data-length commands -- the most common usage
        :buffer({ sync_byte_value, command_id})
    {
        if( 0 == payload_length){
            buffer[2] = command_id;
        } 
    }

    Command(uint8_t command_id, uint8_t data_byte)
        :buffer({ sync_byte_value, command_id, data_byte})
    {
        if( 1 == payload_length){
            pack();
        }
    }

    // ~Command();

    uint8_t command() const { return buffer[1]; }

    constexpr uint8_t* data() { return buffer.data(); }
    constexpr const uint8_t* data() const { return buffer.data(); }

    constexpr uint8_t* header() { return buffer.data(); }
    constexpr const uint8_t* header() const { return buffer.data(); }

    constexpr uint8_t* payload() { return buffer.data() + payload_index; }
    constexpr const uint8_t* payload() const { return buffer.data() + payload_index; }
    uint8_t& payload(size_t index ){ return buffer[ payload_index + index];}

    constexpr uint8_t* footer() { return buffer.data() + footer_index; }
    constexpr const uint8_t* footer() const { return buffer.data() + footer_index; }

    void log( spdlog::logger& log, spdlog::level::level_enum level, const std::string& preamble="") const {
        fmt::memory_buffer logbuf;

        // write preamble to buffer
        fmt::format_to( logbuf, preamble);

        // add command header
        fmt::format_to( logbuf, "  {:02X} {:02X}", buffer[0], buffer[1]);

        // add command data body
        size_t i = 0;
        if( 0 < payload_length ){
            format_to( logbuf, "   ");
            for( i = payload_index; i < footer_index; ++i ){
                format_to( logbuf, " {:02X}", logbuf[i]);
            }
        }

        // add command footer
        format_to( logbuf, "    {:02X}", buffer[i]);

        // // finally, log the result:
        log.log( level, fmt::to_string(logbuf) );
    }

    uint8_t& operator[](size_t index ){
        return buffer[index];
    }

    constexpr void pack(){
        buffer[checksum_index] = checksum();
    }

    static constexpr size_t response_payload_size = response_length;

    constexpr size_t size() const { return buffer.size(); }

    /// \brief read a value, starting from the given data index 
    void write_float32( float source, size_t index ) { 
        _write_float32( source, payload_index + index);
    }

    /// \brief read a value, starting from the given data index 
    void write_uint32( uint32_t source, size_t index ) { 
        _write_uint32( source, payload_index + index);
    }

    void write_bytes(uint8_t* source, size_t dest_index, size_t len) {
        _write_bytes( source, payload_index + dest_index, len);
    }

    void _write_bytes(uint8_t* source, size_t dest_index, size_t len) {
        memcpy( buffer.data() + dest_index, source, len);
    }

    void _write_float32( float source, size_t dest_index ){
        auto source_bytes = reinterpret_cast<uint8_t*>(&source);

        // 2 == length of command header
        auto dest_bytes = buffer.data() + dest_index;

        // little-endian -> big-endian
        dest_bytes[0] = source_bytes[3];
        dest_bytes[1] = source_bytes[2];
        dest_bytes[2] = source_bytes[1];
        dest_bytes[3] = source_bytes[0];
    }

    void _write_uint32( uint32_t source, size_t dest_index ){
        auto source_bytes = reinterpret_cast<uint8_t*>(&source);

        // 2 == length of command header
        auto dest_bytes = buffer.data() + dest_index;

        // little-endian -> big-endian
        dest_bytes[0] = source_bytes[3];
        dest_bytes[1] = source_bytes[2];
        dest_bytes[2] = source_bytes[1];
        dest_bytes[3] = source_bytes[0];
    }

private:
    // byte array to holds actual command bytes:
    std::array<uint8_t, header_length+payload_length+footer_length> buffer;

private:
    constexpr uint8_t checksum() const {
        int sum = 0; 
        for(size_t i = 1; i < buffer.size()-1; ++i ){ 
            sum += buffer[i];
        }
        return sum;
    }

}; // class IMU::Command

} // namespace IMU

#endif // #ifndef _IMU_COMMAND_HPP_
