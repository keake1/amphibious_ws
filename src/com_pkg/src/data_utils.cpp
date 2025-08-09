#include "com_pkg/data_utils.hpp"
#include <cmath>

namespace com_pkg {

// Person 类方法实现
void PersonAggregator::Person::add_sample(float x, float y) {
    samples.emplace_back(x, y);
    update_average();
}

void PersonAggregator::Person::update_average() {
    if (samples.empty()) return;
    
    float sum_x = 0.0f, sum_y = 0.0f;
    for (const auto& sample : samples) {
        sum_x += sample.first;
        sum_y += sample.second;
    }
    avg_x = sum_x / samples.size();
    avg_y = sum_y / samples.size();
}

void PersonAggregator::Person::reset() {
    samples.clear();
    avg_x = avg_y = 0.0f;
    ready = false;
}

// PersonAggregator 类方法实现
PersonAggregator::PersonAggregator(int max_persons, int min_samples, float merge_distance)
    : max_persons_(max_persons)
    , min_samples_(min_samples)
    , merge_distance_(merge_distance) {
    persons_.resize(max_persons_);
}

bool PersonAggregator::add_coordinate(float x, float y) {
    // 尝试归类到已有人员
    bool merged = false;
    for (auto& person : persons_) {
        if (!person.samples.empty()) {
            float dx = x - person.avg_x;
            float dy = y - person.avg_y;
            if (std::sqrt(dx*dx + dy*dy) < merge_distance_) {
                person.add_sample(x, y);
                if (static_cast<int>(person.samples.size()) >= min_samples_ && !person.ready) {
                    person.ready = true;
                }
                merged = true;
                break;
            }
        }
    }
    
    // 如果没有归类且还有空位，则新建一个
    if (!merged) {
        for (auto& person : persons_) {
            if (person.samples.empty()) {
                person.add_sample(x, y);
                break;
            }
        }
    }
    
    // 检查是否所有人员都准备好
    int ready_count = 0;
    for (const auto& person : persons_) {
        if (person.ready) ready_count++;
    }
    
    return ready_count == max_persons_;
}

std::vector<std::pair<float, float>> PersonAggregator::get_all_coordinates() const {
    std::vector<std::pair<float, float>> coordinates;
    coordinates.reserve(max_persons_);
    
    for (const auto& person : persons_) {
        if (person.ready) {
            coordinates.emplace_back(person.avg_x, person.avg_y);
        }
    }
    
    return coordinates;
}

void PersonAggregator::reset_all() {
    for (auto& person : persons_) {
        person.reset();
    }
}

} // namespace com_pkg
