#include "plotter.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace autoaim {

Plotter::Plotter(const std::string &host, uint16_t port) {
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    dest_.sin_family = AF_INET;
    dest_.sin_port = ::htons(port);
    dest_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter::~Plotter() { ::close(sock_); }

void Plotter::plot(const std::string &data) {
    std::lock_guard<std::mutex> lock(mtx_);
    ::sendto(sock_, data.c_str(), data.size(), 0,
             reinterpret_cast<sockaddr *>(&dest_), sizeof(dest_));
}

}  // namespace autoaim
