// GPL v3 (c) 2020, Daniel Williams 

#ifndef _IMU_COMMAND_HPP_
#define _IMU_COMMAND_HPP_

#include "../../serial/src/message.hpp"

#include <array>

namespace IMU {

template<size_t payload_length>
class Command {
    static constexpr uint8_t header_length = 2;
    static constexpr uint8_t footer_length = 1;

    // header fields
    static constexpr uint8_t sync_byte_value = 0xF7;
    static constexpr uint8_t sync_byte_index = 0;
    static constexpr uint8_t command_id_index = 1;

    // data field
    static constexpr uint8_t data_index = header_length;

    // footer fields
    static constexpr uint8_t footer_index = header_length + payload_length;
    static constexpr uint8_t checksum_index = footer_index;

public:
    Command() = delete;

    constexpr Command(uint8_t command_id)
        // optimized for zero-data-length commands -- the most common usage
        :message(sync_byte_value, command_id)
    {
        if( 0 == payload_length){
            pack();
        } 
    }

    Command(uint8_t command_id, uint8_t data_byte)
        : message(sync_byte_value, command_id)
    {
        if( 1 == payload_length){
            message[2] = data_byte;
            pack();
        }
    }
    
    // ~Command();

    constexpr uint8_t* data() { return message.data(); }

    constexpr const uint8_t* data() const { return message.data(); }

    // convenience method specifically to set data bytes:
    uint8_t& data(size_t index ){
        return message[ 2 + index];
    }

    int fprinth(FILE* dest) const {
        int total = 0;
        size_t i;

        // print header:
        total += fprintf(stdout, "    ");
        for( i = 0; i < header_length; ++i ){
            total += fprintf( dest, "%02X", message[i] );
        }

        // print data:
        if( 0 < payload_length){
            total += fprintf(stdout, "  ");
            for( i = data_index; i < footer_index; ++i ){
                total += fprintf( dest, "%02X", message[i] );
            }
        }

        // print footer
        total += fprintf(stdout, "    ");
        for( i = footer_index; i < message.size(); ++i ){
            total += fprintf( dest, "%02X", message[i] );
        }

        return total;
    }

    uint8_t& operator[](size_t index ){
        return message[index];
    }

    constexpr void pack(){
        message[checksum_index] = checksum();
    }

    constexpr size_t size() const { return message.size(); }

    void write_bytes(uint8_t* source, size_t dest_index, size_t len) {
        return message.write_bytes( source, data_index + dest_index, len);
    }

    void write_uint32(uint32_t source, size_t dest_index) {
        return message.write_uint32( source, data_index + dest_index);
    }

private:
    // byte array to holds actual command bytes
    Serial::Message< header_length, payload_length, footer_length> message;

private:
    constexpr uint8_t checksum() const {
        int sum = 0; 
        for(size_t i = 1; i < message.size()-1; ++i ){ 
            sum += message[i];
        }
        return sum;
    }

}; // class IMU::Command

#ifdef DEBUG
// 0 (0x00), Read tared orientation as quaternion
constexpr IMU::Command<0> get_quaternion_command = Command<0>(0);

// 1 (0x01), Read tared orientation as euler angles
constexpr IMU::Command<0> get_euler_angles_command = Command<0>(1);

// 2 (0x02), Get tared orientation as rotation matrix
constexpr IMU::Command<0> get_rotation_matrix_command = Command<0>(2);
#endif  // #ifdef DEBUG

// 96 (0x60)  Set Tare with current orientation
constexpr IMU::Command<0> set_tare_command = Command<0>( 96 );

// 85 (0x55)  start streaming
constexpr IMU::Command<0> start_stream_command = Command<0>( 85 );

// 86 (0x56) Stop Streaming
constexpr IMU::Command<0> stop_stream_command = Command<0>( 86 );

} // namespace IMU

#endif // #ifndef _IMU_COMMAND_HPP_
