// GPL v3 (c) 2020, Daniel Williams 


#ifndef _SERIAL_MESSAGE_HPP_
#define _SERIAL_MESSAGE_HPP_

#include <array>

namespace Serial {

template<size_t header_length, size_t data_length, size_t footer_length>
class Message {
public:

    Message() = default;

    constexpr Message(uint8_t b0, uint8_t b1)
        : buffer({b0, b1})
    {}

    constexpr Message(uint8_t b0, uint8_t b1, uint8_t b2)
        : buffer({b0, b1, b2})
    {}

    // constexpr Message(uint8_t b0, ... )
    //     : buffer({sync_byte, command_byte, data_byte})
    // {

    ~Message() = default;

    uint8_t* data() { return buffer.data(); }
    uint8_t const * data() const { return buffer.data(); }

    // convenience method specifically to set data bytes:
    uint8_t& data(size_t index ){
        return buffer[ data_index + index];
    }

    constexpr uint8_t* operator*() { return buffer.data(); }

    constexpr const uint8_t* operator*() const { return buffer.data(); }

    constexpr uint8_t& operator[](size_t index ){ return buffer[index]; }
    constexpr const uint8_t& operator[](size_t index ) const { return buffer[index]; }

    float read_float32( size_t index ){
        float dest;
        uint8_t* source_bytes = reinterpret_cast<uint8_t*>(buffer.data() + index);
        uint8_t* dest_bytes = reinterpret_cast<uint8_t*>(&dest);

        dest_bytes[0] = source_bytes[3];
        dest_bytes[1] = source_bytes[2];
        dest_bytes[2] = source_bytes[1];
        dest_bytes[3] = source_bytes[0];

        return dest;
    }

    constexpr size_t size() const { return buffer.size(); }

    int fprinth(FILE* dest) const {
        int total = 0;
        for( size_t i=0; i < buffer.size(); ++i ){
            if( 0 == (i%8)){
                fputc(' ', dest);
            }
            total += fprintf( dest, "%02X", buffer[i] );
        }
        
        return total;
    }

    void write_bytes(uint8_t* source, size_t dest_index, size_t len) {
        memcpy( buffer.data() + dest_index, source, len);
    }

    void write_uint32( uint32_t source, size_t dest_index ){
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
    static constexpr uint8_t header_index = 0; // by definition
    static constexpr uint8_t data_index = header_length;
    static constexpr uint8_t footer_index = header_length + data_length;

    std::array<uint8_t, header_length+data_length+footer_length> buffer;

}; // class Serial::Message

} // namespace Serial

#endif // #ifndef _SERIAL_MESSAGE_HPP_
