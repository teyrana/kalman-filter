// GPL v3 (c) 2020, Daniel Williams 

// this file is included directly into "response.hpp"

template<size_t payload_length>
void IMU::Response<payload_length>::log( spdlog::logger& log, spdlog::level::level_enum level, const std::string& preamble) const {
    fmt::memory_buffer logbuf;

    // write preamble to buffer
    fmt::format_to( logbuf, preamble);

    size_t index = 0;
    { // add response header
        fmt::format_to( logbuf, "   ");
        if( 0 < (response_flags_& SUCCESS_FLAG) ){
            fmt::format_to( logbuf, " {:02X}", buffer[0]);
            index=1;
        }

        if( 0 < (response_flags_& TIMESTAMP_FLAG) ){
            fmt::format_to( logbuf, " {:02X} {:02X} {:02X} {:02X}",
                                    buffer[index], buffer[index+1], buffer[index+2], buffer[index+3] );
            index += 4;
        }
        if( 0 < (response_flags_& COMMAND_ECHO_FLAG) ){
            fmt::format_to( logbuf, " {:02X}", buffer[index]);
            index++;
        }
    }

    // print response payload / body:
    fmt::format_to( logbuf, "    ");
    if( 0 < payload_length ){
        for( index = payload_index; index < buffer.size(); ++index ){
            fmt::format_to( logbuf, " {:02X}", buffer[index] );
        }
    }

    // // finally, log the result:
    log.log( level, fmt::to_string(logbuf) );
}

template<size_t payload_length>
void IMU::Response<payload_length>::read_chunk32( const uint8_t* const read_bytes, uint8_t* write_bytes ) {
    write_bytes[0] = read_bytes[3];
    write_bytes[1] = read_bytes[2];
    write_bytes[2] = read_bytes[1];
    write_bytes[3] = read_bytes[0];
}

template<size_t payload_length>
float_t IMU::Response<payload_length>::read_float32( size_t index ) const { 
    float_t result;
    const uint8_t* from = (buffer.data() + payload_index + index);
    uint8_t* to = reinterpret_cast<uint8_t*>(&result);
    read_chunk32( from, to );
    return result; // should RVO
}

template<size_t payload_length>
double_t IMU::Response<payload_length>::read_float64( size_t index ) const { 
    float_t result;
    const uint8_t* from = (buffer.data() + payload_index + index);
    uint8_t* to = reinterpret_cast<uint8_t*>(&result);
    read_chunk32( from, to );
    return static_cast<double_t>(result); // should RVO
}

template<size_t payload_length>
int32_t IMU::Response<payload_length>::read_int32( size_t index ) const { 
    int32_t result;
    const uint8_t* from = (buffer.data() + payload_index + index);
    uint8_t* to = reinterpret_cast<uint8_t*>(&result);
    read_chunk32( from, to );
    return result; // should RVO
}

template<size_t payload_length>
uint32_t IMU::Response<payload_length>::read_uint32( size_t index ) const { 
    uint32_t result;
    const uint8_t* from = (buffer.data() + payload_index + index);
    uint8_t* to = reinterpret_cast<uint8_t*>(&result);
    read_chunk32( from, to );
    return result; // should RVO
}

template<size_t payload_length>
Eigen::Vector3d& IMU::Response<payload_length>::read_vector3d( size_t index, Eigen::Vector3d& dest) const {
    dest[0] = read_float64( index + 0 );
    dest[1] = read_float64( index + 4 );
    dest[2] = read_float64( index + 8 );
    return dest;
}

template<size_t payload_length>
Eigen::Vector3i& IMU::Response<payload_length>::read_vector3i( size_t index, Eigen::Vector3i& dest) const {
    dest[0] = read_int32( index + 0 );
    dest[1] = read_int32( index + 4 );
    dest[2] = read_int32( index + 8 );
    return dest;
}

template<size_t payload_length>
Eigen::Vector4d& IMU::Response<payload_length>::read_vector4d( size_t index, Eigen::Vector4d& dest) const {
    dest[0] = read_float64( index + 0 );
    dest[1] = read_float64( index + 4 );
    dest[2] = read_float64( index + 8 );
    dest[3] = read_float64( index +12 );
    return dest;
}

template<size_t payload_length>
uint32_t IMU::Response<payload_length>::timestamp() const { 
    uint32_t _timestamp = 0;

    if(provides_timestamp()){
        const uint8_t* from = (buffer.data() + timestamp_index);
        uint8_t* to = reinterpret_cast<uint8_t*>(&_timestamp);
        read_chunk32( from, to );
    }

    return _timestamp;  // should RVO
}
