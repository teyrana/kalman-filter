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
        buffer[2] = checksum();
    }
    
    // ~Command();

    constexpr const uint8_t* data() const { return buffer.data(); }
    constexpr size_t size() const { return buffer.size(); }

    int fprinth(FILE* fd) const {
        int total = 0;
        for( size_t i = 0; i < buffer.size(); ++i){
            if( 0 == i%8 ){
                total += fputc(' ', fd);
            }
            fprintf(fd, "%02X ", buffer[i]);
        }
        return total;
    }

private:
    // byte array: holds actual command
    // data length = length of command data, _not total length_! 
    // total length == fence-post-byte + command_id + [data bytes] + checksum + new-line 
    /*const*/ std::array<uint8_t, data_length + 3 > buffer;    

    uint8_t checksum() const {
        int sum = 0; 
        for(size_t i = 0; i < buffer.size()-1; ++i ){ 
            sum += buffer[i];
        }
        return sum;
    }

}; // class IMU::Command

} // namespace IMU

#endif // #ifndef _IMU_COMMAND_HPP_
