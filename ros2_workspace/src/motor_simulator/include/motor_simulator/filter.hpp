#pragma once

#include <vector>
#include <algorithm>

class Filter {
public:
    Filter() = default;
    virtual ~Filter() = default;
    virtual double process(double input_value) = 0;
    virtual void reset() = 0;
};

class MedianFilter : public Filter {
private:
    std::vector<double> buffer;
    size_t buffer_size;
    size_t index;

public:
    explicit MedianFilter(size_t buffer_size = 5) 
        : buffer_size(buffer_size), index(0) {
        buffer.resize(buffer_size, 0.0);
    }

    double process(double input_value) override {
        buffer[index] = input_value;
        index = (index + 1) % buffer_size;

        auto sortedBuffer = buffer;
        std::sort(sortedBuffer.begin(), sortedBuffer.end());

        if (buffer_size % 2 == 1) {
            return sortedBuffer[buffer_size / 2];
        } else {
            return (sortedBuffer[buffer_size / 2 - 1] + sortedBuffer[buffer_size / 2]) / 2.0;
        }
    }

    void reset() override {
        std::fill(buffer.begin(), buffer.end(), 0.0);
        index = 0;
    }
};

class LowPassFilter : public Filter {
private:
    double alpha;      
    double previous_output;  
    bool is_first;      

public:
    explicit LowPassFilter(double alpha = 0.3) 
        : alpha(alpha), previous_output(0.0), is_first(true) {
        // 确保alpha在有效范围内
        if (alpha <= 0 || alpha > 1) {
            this->alpha = 0.3;
        }
    }

    double process(double input_value) override {
        if (is_first) {
            previous_output = input_value;
            is_first = false;
            return input_value;
        }

        // 低通滤波公式: 本次值 = 上次值 * (1 - alpha) + 本次原始值 * alpha
        auto output = previous_output * (1 - alpha) + input_value * alpha;
        previous_output = output;

        return previous_output;
    }

    void reset() override {
        previous_output = 0.0;
        is_first = true;
    }
};

// 均值滤波器
class MeanFilter : public Filter {
private:
    std::vector<double> buffer;
    size_t buffer_size;
    size_t index;
    double sum;        

public:
    explicit MeanFilter(size_t buffer_size = 3) 
        : buffer_size(buffer_size), index(0), sum(0.0) {
        buffer.resize(buffer_size, 0.0);
    }

    double process(double input_value) override {
        sum -= buffer[index];
        buffer[index] = input_value;
        sum += input_value;

        index = (index + 1) % buffer_size;

        return sum / (double)buffer_size;
    }

    void reset() override {
        std::fill(buffer.begin(), buffer.end(), 0.0);
        sum = 0.0;
        index = 0;
    }
};



