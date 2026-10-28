#ifndef SIMULATION_UDP_SENDER_H_
#define SIMULATION_UDP_SENDER_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>
#include <string>

// Fire-and-forget UDP datagrams to a fixed address (live plot feed).
class UDPSender {
 public:
  UDPSender(const std::string& ip, int port) : _sock(socket(AF_INET, SOCK_DGRAM, 0)) {
    if (_sock < 0) {
      throw std::runtime_error("UDPSender: cannot create socket");
    }
    _addr.sin_family = AF_INET;
    _addr.sin_port = htons(static_cast<uint16_t>(port));
    _addr.sin_addr.s_addr = inet_addr(ip.c_str());
  }
  UDPSender(const UDPSender&) = delete;
  UDPSender& operator=(const UDPSender&) = delete;
  ~UDPSender() { close(_sock); }

  void send(const std::string& message) {
    sendto(_sock, message.data(), message.size(), 0, reinterpret_cast<const sockaddr*>(&_addr),
           sizeof(_addr));
  }

 private:
  int _sock;
  sockaddr_in _addr{};
};

#endif  // SIMULATION_UDP_SENDER_H_
