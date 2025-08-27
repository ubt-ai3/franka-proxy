#ifndef INCLUDED__FT_SENSOR__FT_SENSOR_HPP
#define INCLUDED__FT_SENSOR__FT_SENSOR_HPP


#include <exception>
#include <mutex>

#include <Eigen/Geometry>


namespace franka_proxy
{
struct ft_sensor_response
{
	uint32_t ft_sequence_number;
	std::array<double, 6> data;
};


class ft_sensor
{
public:
	virtual ~ft_sensor() = default;

	ft_sensor_response read() const
	{
		ft_sensor_response result = read_raw();

		for (int i = 0; i < 6; i++)
			result.data[i] -= bias_(i);

		return result;
	}

	ft_sensor_response read_raw() const
	{
		std::lock_guard lock(current_ft_sensor_response_mutex_);
		return current_ft_sensor_response_;
	}

	Eigen::Affine3f fts_t_flange() const
	{
		return fts_T_flange_;
	}

	Eigen::Affine3f ee_t_kms() const
	{
		return ee_t_fts_;
	}

	Eigen::Vector<double, 6> bias() const
	{
		return bias_;
	}

	double load_mass() const
	{
		return load_mass_;
	}

	void set_bias(const Eigen::Vector<double, 6>& bias)
	{
		bias_ = bias;
	}

	void set_load_mass(double load_mass)
	{
		load_mass_ = load_mass;
	}

protected:
	ft_sensor(
		Eigen::Affine3f transform,
		Eigen::Affine3f affine3_f,
		Eigen::Vector<double, 6> bias,
		double load_mass)
		: current_ft_sensor_response_(),
		  fts_T_flange_(std::move(transform)),
		  ee_t_fts_(std::move(affine3_f)),
		  bias_(std::move(bias)),
		  load_mass_(load_mass)
	{
	}

	mutable std::mutex current_ft_sensor_response_mutex_;
	ft_sensor_response current_ft_sensor_response_;

	Eigen::Affine3f fts_T_flange_;
	Eigen::Affine3f ee_t_fts_;
	Eigen::Vector<double, 6> bias_; //[fx, fy, fz, tx, ty, tz]
	double load_mass_;
};


class ft_sensor_connection_exception
	: public std::exception
{
public:
	const char* what() const noexcept override
	{
		return "No force/torque sensor is available.";
	}
};
} /* namespace franka_proxy */

#endif // INCLUDED__FT_SENSOR__FT_SENSOR_HPP
