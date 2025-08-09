#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <cstring>

namespace com_pkg {

// 数据工具类
class DataUtils {
public:
    // 将任意类型的数据转换为字节数组
    template <typename T>
    static std::vector<uint8_t> to_bytes(T value) {
        std::vector<uint8_t> bytes(sizeof(T));
        std::memcpy(bytes.data(), &value, sizeof(T));
        return bytes;
    }
    
    // 从字节数组中提取指定类型的值
    template <typename T>
    static T from_bytes(const std::vector<uint8_t>& data, size_t offset = 0) {
        if (offset + sizeof(T) > data.size()) {
            return T{}; // 返回默认值
        }
        
        T value;
        std::memcpy(&value, data.data() + offset, sizeof(T));
        return value;
    }
    
    // 将数据追加到字节数组
    template <typename T>
    static void append_to_vector(std::vector<uint8_t>& vec, T value) {
        auto bytes = to_bytes(value);
        vec.insert(vec.end(), bytes.begin(), bytes.end());
    }
};

// 人员坐标聚合器
class PersonAggregator {
public:
    struct Person {
        std::vector<std::pair<float, float>> samples;
        float avg_x = 0.0f;
        float avg_y = 0.0f;
        bool ready = false;
        
        void add_sample(float x, float y);
        void update_average();
        void reset();
    };
    
    PersonAggregator(int max_persons = 3, int min_samples = 10, float merge_distance = 0.2f);
    
    // 添加新的坐标样本，返回是否所有人员都准备好
    bool add_coordinate(float x, float y);
    
    // 获取所有人员的平均坐标
    std::vector<std::pair<float, float>> get_all_coordinates() const;
    
    // 重置所有聚合器
    void reset_all();
    
private:
    std::vector<Person> persons_;
    const int max_persons_;
    const int min_samples_;
    const float merge_distance_;
};

} // namespace com_pkg
