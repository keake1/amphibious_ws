#pragma once

#include <vector>
#include <cstdint>
#include <utility>

namespace com_pkg {

// 数据包解析状态
enum class ParseState {
    WAIT_HEADER_1,
    WAIT_HEADER_2,
    WAIT_TYPE,
    WAIT_LENGTH,
    WAIT_DATA,
    WAIT_CHECKSUM,
    WAIT_ADD_CHECKSUM
};

// 串口协议处理类
class SerialProtocol {
public:
    SerialProtocol();
    
    // 处理接收到的字节，返回是否有完整数据包
    bool process_byte(uint8_t byte);
    
    // 获取最后解析的数据包
    uint8_t get_frame_type() const { return frame_type_; }
    const std::vector<uint8_t>& get_frame_data() const { return frame_data_; }
    
    // 创建发送数据包
    static std::vector<uint8_t> create_packet(uint8_t type, const std::vector<uint8_t>& payload);
    
    // 重置解析状态
    void reset();
    
private:
    ParseState parse_state_;
    uint8_t frame_type_;
    uint8_t frame_length_;
    std::vector<uint8_t> frame_data_;
    uint8_t frame_checksum_;
    uint8_t frame_add_checksum_;
    
    // 计算校验和
    static std::pair<uint8_t, uint8_t> calculate_checksums(const std::vector<uint8_t>& data);
};

} // namespace com_pkg
