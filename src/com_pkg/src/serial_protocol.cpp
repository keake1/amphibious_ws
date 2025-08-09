#include "com_pkg/serial_protocol.hpp"
#include <cstring>

namespace com_pkg {

SerialProtocol::SerialProtocol() 
    : parse_state_(ParseState::WAIT_HEADER_1)
    , frame_type_(0)
    , frame_length_(0)
    , frame_checksum_(0)
    , frame_add_checksum_(0) {
}

bool SerialProtocol::process_byte(uint8_t byte) {
    switch (parse_state_) {
        case ParseState::WAIT_HEADER_1:
            if (byte == 0xAA) {
                parse_state_ = ParseState::WAIT_HEADER_2;
                frame_checksum_ = byte;
                frame_add_checksum_ = byte;
            }
            break;
            
        case ParseState::WAIT_HEADER_2:
            if (byte == 0xFF) {
                parse_state_ = ParseState::WAIT_TYPE;
                frame_checksum_ += byte;
                frame_add_checksum_ += frame_checksum_;
            } else {
                parse_state_ = ParseState::WAIT_HEADER_1;
            }
            break;
            
        case ParseState::WAIT_TYPE:
            frame_type_ = byte;
            frame_checksum_ += byte;
            frame_add_checksum_ += frame_checksum_;
            parse_state_ = ParseState::WAIT_LENGTH;
            break;
            
        case ParseState::WAIT_LENGTH:
            frame_length_ = byte;
            frame_checksum_ += byte;
            frame_add_checksum_ += frame_checksum_;
            frame_data_.clear();
            
            if (frame_length_ > 0) {
                parse_state_ = ParseState::WAIT_DATA;
            } else {
                parse_state_ = ParseState::WAIT_CHECKSUM;
            }
            break;
            
        case ParseState::WAIT_DATA:
            frame_data_.push_back(byte);
            frame_checksum_ += byte;
            frame_add_checksum_ += frame_checksum_;
            
            if (frame_data_.size() >= frame_length_) {
                parse_state_ = ParseState::WAIT_CHECKSUM;
            }
            break;
            
        case ParseState::WAIT_CHECKSUM:
            if (byte == frame_checksum_) {
                parse_state_ = ParseState::WAIT_ADD_CHECKSUM;
            } else {
                parse_state_ = ParseState::WAIT_HEADER_1;
                return false;
            }
            break;
            
        case ParseState::WAIT_ADD_CHECKSUM:
            parse_state_ = ParseState::WAIT_HEADER_1;
            return (byte == frame_add_checksum_);
    }
    
    return false;
}

std::vector<uint8_t> SerialProtocol::create_packet(uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> packet;
    packet.reserve(6 + payload.size()); // 预分配空间
    
    packet.push_back(0xAA); // 起始字节
    packet.push_back(0xFF); // 标识字节
    packet.push_back(type); // 数据类型
    packet.push_back(static_cast<uint8_t>(payload.size())); // 数据长度
    packet.insert(packet.end(), payload.begin(), payload.end()); // 插入数据负载

    auto [checksum, add_checksum] = calculate_checksums(packet);
    packet.push_back(checksum);
    packet.push_back(add_checksum);
    
    return packet;
}

void SerialProtocol::reset() {
    parse_state_ = ParseState::WAIT_HEADER_1;
    frame_data_.clear();
}

std::pair<uint8_t, uint8_t> SerialProtocol::calculate_checksums(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0, add_checksum = 0;
    for (auto byte : data) {
        checksum += byte;
        add_checksum += checksum;
    }
    return {checksum, add_checksum};
}

} // namespace com_pkg
