#include <franka_proxy_client/franka_network_client2.hpp>

#include <asio/connect.hpp>
#include <asio/read.hpp>
#include <asio/write.hpp>
#include <asio/ip/tcp.hpp>

namespace {

    std::unique_ptr<asio::ip::tcp::socket> connect(const std::string& ip, std::uint16_t port, asio::io_context& ctx)
    {
        asio::ip::tcp::resolver resolver(ctx);
        asio::ip::tcp::resolver::results_type endpoints{ resolver.resolve(ip, std::to_string(port)) };

        auto socket = std::make_unique<asio::ip::tcp::socket>(ctx);
        asio::connect(*socket, endpoints);

        return socket;
    }

}

using namespace franka_proxy;

franka_network_client2::franka_network_client2(const std::string& addr, std::uint16_t port)
: io_context_{new asio::io_context}
, connection_{connect(addr, port, *io_context_)}
{

}

void franka_network_client2::send_command_raw(const void* data, std::size_t data_size)
{
    auto buf = asio::buffer(data, data_size);
    asio::write(*connection_, buf);
}