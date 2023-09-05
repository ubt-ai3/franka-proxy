#if !defined(INCLUDED__FT_SENSOR__FT_SENSOR_HPP)
#define INCLUDED__FT_SENSOR__FT_SENSOR_HPP

#include <functional>
#include <exception>
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
		auto ret = current_ft_sensor_response_.load();
		for (int i = 0; i < 6; i++)
			ret.data[i] -= bias_(i);
		return ret;
	}

	ft_sensor_response read_raw() const
	{
		return current_ft_sensor_response_.load();
	}

	Eigen::Affine3f fts_t_flange() const
	{
		return fts_t_flange_;
	}

	Eigen::Affine3f ee_t_kms() const
	{
		return ee_t_fts_;
	}

	Eigen::Vector<double, 6> bias() const
	{
		return bias_;
	}

	Eigen::Vector3d load_mass() const
	{
		return load_mass_;
	}

	void set_bias(const Eigen::Vector<double, 6>& bias)
	{
		bias_ = bias;
	}
	void set_load_mass(const Eigen::Vector3d& load_mass)
	{
		load_mass_ = load_mass;
	}

protected:
	ft_sensor(Eigen::Affine3f transform,
	          Eigen::Affine3f affine3_f,
	          Eigen::Vector<double, 6> bias,
	          Eigen::Vector3d load_mass)
		: fts_t_flange_(std::move(transform)),
		  ee_t_fts_(std::move(affine3_f)),
		  bias_(std::move(bias)),
		  load_mass_(std::move(load_mass))
	{
	}

	std::atomic<ft_sensor_response> current_ft_sensor_response_;

	Eigen::Affine3f fts_t_flange_;
	Eigen::Affine3f ee_t_fts_;
	Eigen::Vector<double, 6> bias_; //[fx, fy, fz, tx, ty, tz]
	Eigen::Vector3d load_mass_;
	Eigen::Matrix3d load_inertia_;
};

class ft_sensor_connection_exception : public std::exception
{
public:
	char* what() {
		return "no force/torque sensor available";
	}
};
} //franka_proxy

#endif /* !defined(INCLUDED__FT_SENSOR__FT_SENSOR_HPP) */
