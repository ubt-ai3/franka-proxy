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
class execution_context;


namespace ip
	{ class tcp; }


namespace execution
{


template <typename... SupportableProperties>
	class any_executor;
template<class T> struct
	context_as_t;
template <typename Property>
	struct prefer_only;


namespace detail
{
namespace relationship
{
template <int I> struct fork_t;
template <int I> struct continuation_t;
}
namespace outstanding_work
{
template <int I> struct untracked_t;
template <int I> struct tracked_t;
}
namespace blocking
{
template <int I> struct possibly_t;
template <int I> struct always_t;
template <int I> struct never_t;
}
}
}


typedef execution::any_executor
	<execution::context_as_t<execution_context&>,
	 execution::detail::blocking::never_t<0>,
	 execution::prefer_only<execution::detail::blocking::possibly_t<0>>,
	 execution::prefer_only<execution::detail::outstanding_work::tracked_t<0>>,
	 execution::prefer_only<execution::detail::outstanding_work::untracked_t<0>>,
	 execution::prefer_only<execution::detail::relationship::fork_t<0>>,
	 execution::prefer_only<execution::detail::relationship::continuation_t<0>>> any_io_executor;


template <typename Protocol, typename Executor>
	class basic_stream_socket;



} /* namespace asio */




namespace franka_proxy
{


using asio_tcp_socket = asio::basic_stream_socket<asio::ip::tcp, asio::any_io_executor>;


} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY_SHARE__ASIO_FORWARD_HPP) */