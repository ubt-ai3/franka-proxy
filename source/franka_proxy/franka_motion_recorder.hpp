/**
 *************************************************************************
 *
 * @file franka_motion_recorder.hpp
 * 
 * todo
 *
 ************************************************************************/


#if !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_RECORDER_HPP)
#define INCLUDED__FRANKA_PROXY__FRANKA_MOTION_RECORDER_HPP
#include <vector>


namespace franka_proxy
{
namespace detail
{


/**
 *************************************************************************
 *
 * @class motion_recorder
 *
 * todo
 *
 ************************************************************************/
class motion_recorder
{
public:
	motion_recorder
		(double rate);

	void start();

	void stop();

	std::vector<int> latest_record();

private:
	std::vector<int> record_;
};


} /* namespace detail */
} /* namespace franka_proxy */


#endif /* !defined(INCLUDED__FRANKA_PROXY__FRANKA_MOTION_RECORDER_HPP) */
