// GPL v3 (c) 2020, Daniel Williams 


#ifndef _IMU_COMMAND_HPP_
#define _IMU_COMMAND_HPP_

#include <array>

namespace IMU {

template<size_t data_length>
class Command {
public:
    Command() = delete;


    constexpr Command(uint8_t command_id)
        : buffer( { 0xF7, 
                    command_id,  // command field
                    command_id,  // checksum field
                    } )
    {}

    Command(uint8_t command_id, size_t length, uint8_t * data){
        buffer[0] = 0xF7;
        buffer[1] = command_id;
        
        if( 0 < length ){
            memcpy( buffer.data() + 2, data, length);
            buffer[ buffer.size() - 2 ] = checksum();
        }
    }
    
    // ~Command();

    constexpr const uint8_t* data() const { return buffer.data(); }

    // convenience method specifically to set data bytes:
    uint8_t& data(size_t index ){
        return buffer[ 2 + index];
    }

    uint8_t& operator[](size_t index ){
        return buffer[index];
    }


    void pack() { 
        buffer[buffer.size() - 1] = checksum();
    }

    constexpr size_t size() const { return buffer.size(); }

    void store_int32_data( uint32_t source, ssize_t dest_index ){
        auto source_bytes = reinterpret_cast<uint8_t*>(&source);

        // 2 == length of command header
        auto dest_bytes = buffer.data() + 2 + dest_index;

        // little-endian -> big-endian
        dest_bytes[0] = source_bytes[3];
        dest_bytes[1] = source_bytes[2];
        dest_bytes[2] = source_bytes[1];
        dest_bytes[3] = source_bytes[0];
    }

    int fprinth(FILE* dest) const {
        int total = 0;
        uint8_t const * cur = buffer.data();
        fprintf(stdout, "    %02X %02X    ", cur[0], cur[1] );
        uint8_t const * const checksum_location = buffer.data() + buffer.size() - 1;
        for( cur = buffer.data() + 2; cur < checksum_location; ++cur ){
            fprintf( dest, "%02X ", *cur );
        }
        fprintf(stdout, "    %02X\n", *checksum_location );
    
        return total;
    }

private:
    // byte array: holds actual command
    // data length = length of command data, _not total length_! 
    // total length == fence-post-byte + command_id + [data bytes] + checksum
    /*const*/ std::array<uint8_t, data_length + 3 > buffer;

    uint8_t checksum() const {
        int sum = 0; 
        for(size_t i = 1; i < buffer.size()-1; ++i ){ 
            sum += buffer[i];
        }
        return sum;
    }

}; // class IMU::Command

} // namespace IMU

#endif // #ifndef _IMU_COMMAND_HPP_
