#ifndef INCLUDED__FRANKA_PROXY_SHARE__ASIO_FORWARD_HPP
#define INCLUDED__FRANKA_PROXY_SHARE__ASIO_FORWARD_HPP
/**
 *************************************************************************
 *
 * @file asio_forward.hpp
 *
 * Forward declarations for the asio library.
 *
 ************************************************************************/

namespace asio
{
namespace ip
{
class tcp;
} /* namespace asio::ip */

class any_io_executor;
template <typename Protocol, typename Executor> class basic_stream_socket;
class io_context;
} /* namespace asio */


namespace franka_proxy
{
using asio_tcp_socket = asio::basic_stream_socket<asio::ip::tcp, asio::any_io_executor>;
} /* namespace franka_proxy */

#endif // INCLUDED__FRANKA_PROXY_SHARE__ASIO_FORWARD_HPP
