#if !defined(INCLUDED__FT_SENSOR__FT_SENSOR_HPP)
#define INCLUDED__FT_SENSOR__FT_SENSOR_HPP

#include <functional>
#include <Eigen/Geometry>


struct response
{
	uint32_t rdt_sequence; // sequence number, counting send packages
	uint32_t ft_sequence; // sequence number, counting internal control-loop
	uint32_t status; //page 104 of https://www.ati-ia.com/app_content/documents/9610-05-1022.pdf
	std::array<double, 6> FTData; //Force and Torque in Newton [fx, fy, fz, tx, ty, tz]
};


class ft_sensor
{
public:
	virtual ~ft_sensor() = default;

	virtual void set_response_handler(const std::function<void(const response&)>& functor) = 0;
	virtual void remove_response_handler() = 0;

	const Eigen::Affine3f kms_T_flange_;
	const Eigen::Affine3f EE_T_kms_;

protected:
	ft_sensor(const Eigen::Affine3f& transform,
	          const Eigen::Affine3f& affine3_f,
	          const std::array<double, 6>& bias,
	          const Eigen::Affine3f& load)
		: kms_T_flange_(transform),
		  EE_T_kms_(affine3_f),
		  bias_(bias), load_(load)
	{
	}

	std::array<double, 6> bias_; //[fx, fy, fz, tx, ty, tz]
	Eigen::Affine3f load_;
};

#endif /* !defined(INCLUDED__FT_SENSOR__FT_SENSOR_HPP) */
