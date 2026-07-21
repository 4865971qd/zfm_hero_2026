#ifndef AUTO_AIM_EXITER_HPP_
#define AUTO_AIM_EXITER_HPP_

#include <atomic>

namespace autoaim {

// 键盘 'q' 键优雅退出
class Exiter {
public:
    Exiter();
    ~Exiter();
    bool exit() const { return exit_.load(); }

private:
    std::atomic<bool> exit_{false};
};

}  // namespace autoaim

#endif  // AUTO_AIM_EXITER_HPP_
