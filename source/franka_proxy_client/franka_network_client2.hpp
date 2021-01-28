#ifndef FRANKA_NETWORK_CLIENT2_HPP
#define FRANKA_NETWORK_CLIENT2_HPP

#include <memory>
#include <string>
#include <type_traits>

#include <franka_proxy/franka_proxy_protocol.hpp>
#include <franka_proxy_share/asio_forward.hpp>

namespace franka_proxy {

    class franka_network_client2 {
        public:

            franka_network_client2(const std::string& addr, std::uint16_t port);

            template<class TPacket>
            void send_command(const TPacket& p) {
                
            }

        private:
            void send_command_raw(const void* data, std::size_t data_size);

            std::unique_ptr<asio::io_context>   io_context_;
            std::unique_ptr<asio_tcp_socket>    connection_;
    };

}

#endif  // FRANKA_NETWROK_CLIENT2_HPP