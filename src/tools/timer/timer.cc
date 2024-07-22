#include "tools/timer/timer.hh"

namespace ctlio {
// 静态成员初始化
std::map<std::string, Timer::TimerRecord> Timer::records_;
}  // namespace ctlio