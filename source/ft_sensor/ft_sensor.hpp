#if !defined(INCLUDED__FT_SENSOR__FT_SENSOR_HPP)
#define INCLUDED__FT_SENSOR__FT_SENSOR_HPP

#include <functional>
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

	virtual ft_sensor_response read() = 0;

	Eigen::Affine3f fts_t_flange() const
	{
		return fts_t_flange_;
	}

	Eigen::Affine3f ee_t_kms() const
	{
		return ee_t_fts_;
	}

	Eigen::Vector<float, 6> bias() const
	{
		return bias_;
	}

	Eigen::Affine3f load() const
	{
		return load_;
	}

protected:
	ft_sensor(Eigen::Affine3f transform,
	          Eigen::Affine3f affine3_f,
	          Eigen::Vector<float, 6> bias,
	          Eigen::Affine3f load)
		: fts_t_flange_(std::move(transform)),
		  ee_t_fts_(std::move(affine3_f)),
		  bias_(std::move(bias)),
		  load_(std::move(load))
	{
	}

	Eigen::Affine3f fts_t_flange_;
	Eigen::Affine3f ee_t_fts_;
	Eigen::Vector<float, 6> bias_; //[fx, fy, fz, tx, ty, tz]
	Eigen::Affine3f load_;
};
} //franka_proxy

#endif /* !defined(INCLUDED__FT_SENSOR__FT_SENSOR_HPP) */
