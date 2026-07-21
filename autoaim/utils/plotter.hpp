#ifndef AUTO_AIM_PLOTTER_HPP_
#define AUTO_AIM_PLOTTER_HPP_

#include <netinet/in.h>
#include <mutex>
#include <string>

namespace autoaim {

// PlotJuggler UDP output.
class Plotter {
public:
    Plotter(const std::string &host = "127.0.0.1", uint16_t port = 9870);
    ~Plotter();
    void plot(const std::string &data);

private:
    int sock_;
    sockaddr_in dest_;
    std::mutex mtx_;
};

}  // namespace autoaim

#endif  // AUTO_AIM_PLOTTER_HPP_
