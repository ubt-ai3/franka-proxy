#ifndef INCLUDED__FORCEBACK__FORCE_TORQUE_SENSOR_HPP
#define INCLUDED__FORCEBACK__FORCE_TORQUE_SENSOR_HPP

#include <array>


namespace franka_proxy
{


class ft_sensor_jr3
{
public:
	
	ft_sensor_jr3();

	/**
	 * records 10000 values and sets offsets to means of those
	 */
	void set_offsets_to_zero();

	/**
	 * not implemented; see rvcr manual
	 * note: set offsets to zero afterwards
	 */
	void set_toolframe_transformation();

	/**
	 * not implemented; see rvcr manual
	 * note: set offsets to zero afterwards
	 */
	void set_full_scales();

	/**
	 * updates raw_values_f0_, raw_values_f2_, raw_values_f3_, raw_values_f4_ and values_
	 * note: uses raw_values_f3_ for value calculation
	 */
	void update();

	std::array<double, 6> current_values() const;
	std::array<int, 6> current_raw_values_f0() const;
	std::array<int, 6> current_raw_values_f2() const;
	std::array<int, 6> current_raw_values_f3() const;
	std::array<int, 6> current_raw_values_f4() const;

private:
	
	void init_jr3();
	void reset_offsets();


	std::string device_name_;
	void* jr3_pci_device_{nullptr};
	unsigned char channel{0};

	std::array<double, 6> ranges{};
	std::array<double, 6> adapt{};

	float full_scale{16384.f};

	std::array<double, 6> values_{};
	std::array<int, 6> raw_values_f0_{};
	std::array<int, 6> raw_values_f2_{};
	std::array<int, 6> raw_values_f3_{};
	std::array<int, 6> raw_values_f4_{};

	static constexpr unsigned long offset_command0 = 0x00e7;
};


} // franka_proxy


#endif // INCLUDED__FORCEBACK__FORCE_TORQUE_SENSOR_HPP
