#include "exiter.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>

namespace autoaim {

Exiter::Exiter() {
    // 在后台检测键盘输入
    std::thread([this] {
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= (~ICANON & ~ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
        int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);

        while (!exit_.load()) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) > 0 && (c == 'q' || c == 'Q')) {
                exit_.store(true);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        fcntl(STDIN_FILENO, F_SETFL, old_flags);
    }).detach();
}

Exiter::~Exiter() = default;

}  // namespace autoaim
