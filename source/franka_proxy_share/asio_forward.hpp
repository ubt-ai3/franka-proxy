/**
 *************************************************************************
 *
 * @file asio_forward.hpp
 *
 * Forward declarations for the asio library.
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY_SHARE__ASIO_FORWARD_HPP)
#define INCLUDED__FRANKA_PROXY_SHARE__ASIO_FORWARD_HPP


namespace asio
{
	class io_context;

	template <typename Protocol>
		class basic_stream_socket;


	namespace ip
	{
		class tcp;
	} /* namespace ip */


} /* namespace asio */




namespace franka_proxy
{


using asio_tsp_socket = asio::basic_stream_socket<asio::ip::tcp>;


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_SHARE__ASIO_FORWARD_HPP) */