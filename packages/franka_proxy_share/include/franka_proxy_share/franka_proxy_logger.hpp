#ifndef INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_LOGGER_HPP
#define INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_LOGGER_HPP
/**
 *************************************************************************
 *
 * @file franka_proxy_logger.hpp
 *
 * Generalized logger for use with motion generators, recorder and tests.
 *
 ************************************************************************/




#include <array>
#include <fstream>
#include <vector>

#include <Eigen/Core>


namespace franka_proxy
{
/***************************************************
* 
* @class logger
* 
* Provides franka_proxy_logging functionality with
* pre-defined data formats for joint data, various
* 3D cartesian data (e.g. velocity), forces and
* torques, single-valued data (e.g. timestamps or
* errors) and additional, arbitrary data (which
* must be provided as strings in order to allow
* for really anything).
* 
***************************************************/
class logger
{
public:
	/**
	* Motion logger for writing various data into a .csv file. Expects double for fixed
	* data formats and strings for arbitrary data.
	* IMPORTANT: Exceptions will only be thrown when header parts mismatch the given number
	* of data sets in each category during a call of start_logging(). Whenever log() is called,
	* data provided in excess of the specified number of entries will be ignored,
	* while providing too few entries will lead to padding with "0.0" or "none" entries.
	* 
	* @param filename: file for the logger to write into
	* @param num_joint_data: number of joint data sets (7 entries per data set)
	* @param num_cart_data: number of cartesian data sets (3 entries per data set)
	* @param num_ft_data: number of force-torque data sets (6 entries per data set);
	* @param num_single: number of single-valued entries
	* @param size_arbitrary: length of additional arbitrary data (single entries)
	**/
	logger(
		std::string filename,
		int num_joint_data,
		int num_cart_data,
		int num_ft_data,
		int num_single,
		int size_arbitrary);


	~logger();


	/**
	* Clears all internal storage, (re)starts franka_proxy_logging and writes the header
	* into the internal buffer (NOT the target log file).
	* Vectors for data categories will be checked against the numbers of data sets specified
	* at construction, an exception will be thrown if any mismatches are detected.
	* 
	* @param joint_data_header: header for the joint data sets (7 entries per data set)
	* @param cart_data_header: header for the cartesian data sets (3 entries per data set)
	* @param ft_data_header: header for the force-torque data sets (6 entries per data set)
	* @param single_header: header for single-valued data
	* @param arbitrary_header: header for arbitrary data (single entries)
	**/
	void start_logging(
		const std::vector<std::string>* joint_data_header,
		const std::vector<std::string>* cart_data_header,
		const std::vector<std::string>* ft_data_header,
		const std::vector<std::string>* single_header,
		const std::vector<std::string>* arbitrary_header);


	/**
	* On first call after calling start_logging(), stops franka_proxy_logging and writes buffer contents
	* into the target log file (overwriting any previous contents). Also clears all internal storage.
	* Subsequent calls will be ignored until start_logging() is called again.
	* Will also be called at destruction, if not called prior.
	**/
	void stop_logging();


	/**
	* Writes a single line into the buffer, using the data provided through the various add-methods, and
	* clears all internal data storage. Will add padding values of "0.0" ("none" for arbitrary data)
	* if less data sets than the initially specified number are provided.
	* Will print warnings for excess or missing data sets, but not throw an exception.
	* 
	* IMPORTANT: This will not write into the target log file, only into the internal buffer.
	**/
	void log();


	/**
	* Clears all internal storage for joint, cartesian, force-torque, single-valued and arbitrary data.
	**/
	void clear_all();


	/**
	* Clears the internal buffer without writing its contents into the target log file.
	**/
	void discard_log();

	void add_joint_data(const std::array<double, 7>& data);
	void add_joint_data(const Eigen::Matrix<double, 7, 1>& data);
	void add_joint_data(double j0, double j1, double j2, double j3, double j4, double j5, double j6);

	void add_cart_data(const std::array<double, 6>& data);
	void add_cart_data(const Eigen::Matrix<double, 6, 1>& data);
	void add_cart_data(double x0, double x1, double x2, double x3, double x4, double x5);

	void add_ft_data(const std::array<double, 6>& data);
	void add_ft_data(const Eigen::Matrix<double, 6, 1>& data);
	void add_ft_data(double fx, double fy, double fz, double tx, double ty, double tz);

	void add_single_data(double data);

	void add_arbitrary_data(const std::string& data);
	void add_arbitrary_data(const std::vector<std::string>& data);

private:
	std::string filename_;

	int num_joint_data_;
	int num_cart_data_;
	int num_ft_data_;
	int num_single_;
	int size_arbitrary_;

	std::vector<std::array<double, 7>> joint_data_;
	std::vector<std::array<double, 6>> cart_data_;
	std::vector<std::array<double, 6>> ft_data_;
	std::vector<double> single_data_;
	std::vector<std::string> arbitrary_data_;

	bool logged_ = true;
	std::ostringstream log_;
	std::ofstream logger_;


	//writes a single line (provided as vector of individual entries) into the internal buffer, adding all needed commas 
	void write_line(std::vector<std::string>& data);

	//checks internal data storage against number of elements specified at construction
	//adds padding if necessary and prints warnings for too many / too few entries
	void check_data();
};

}

#endif // INCLUDED__FRANKA_PROXY_SHARE__FRANKA_PROXY_LOGGER_HPP
