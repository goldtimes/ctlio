#pragma once
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace ctlio {
// 这里会用在代码的任何地方，所以要么用单例模式，要么为static
class Timer {
   public:
    // 定义一个结构体，保存函数和调用该函数的时间
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& name, double time_usage) {
            func_name = name;
            time_usage_in_ms.push_back(time_usage);
        }

        std::string func_name;
        std::vector<double> time_usage_in_ms;
    };

   public:
    // 同样这里用static是不需要在任何地方创建Timer对象
    template <class F>
    static void Evaluate(F&& func, const std::string& func_name) {
        auto start = std::chrono::steady_clock::now();
        // 调用函数
        std::forward<F>(func)();
        auto end = std::chrono::steady_clock::now();
        double time_used = (end - start).count() * 1000;  // s->ms

        if (records_.find(func_name) == records_.end()) {
            TimerRecord record(func_name, time_used);
            records_.insert({func_name, record});
        } else {
            records_[func_name].time_usage_in_ms.push_back(time_used);
        }
    }
    static void Clear() {
        records_.clear();
    }

   private:
    static std::map<std::string, TimerRecord> records_;
};

class TicToc {
   public:
    TicToc() {
        tic();
    }

    TicToc(bool display) {
        display_ = display;
        tic();
    }
    void tic() {
        start = std::chrono::steady_clock::now();
    }
    void toc(std::string _about_task) {
        end = std::chrono::steady_clock::now();
        auto time_used = (end - start).count() * 1000;

        if (display_) {
            std::cout.precision(3);
            std::cout << _about_task << ": " << time_used << " msec." << std::endl;
        }
    }
    double toc() {
        end = std::chrono::steady_clock::now();
        auto time_used = (end - start).count() * 1000;
        return time_used;
    }

   private:
    bool display_ = false;
    std::chrono::time_point<std::chrono::steady_clock> start;
    std::chrono::time_point<std::chrono::steady_clock> end;
};
}  // namespace ctlio