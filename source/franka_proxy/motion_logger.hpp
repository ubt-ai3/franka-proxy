/**
 *************************************************************************
 *
 * @file motion_logger.hpp
 *
 * Generalized logger for use with motion generators and recorder.
 *
 ************************************************************************/

#pragma once

#include <vector>
#include <array>
#include <fstream>

namespace franka_proxy
{
	/************************************************
	* 
	* @class motion_logger
	* 
	* Provides logging functionality for joint data,
	* various velocities and accelerations (or any
	* 3D vectors, really), forces and torques,
	* timestamps and additional, arbitrary data
	* (which must be provided as strings in order to
	* allow for really anything).
	* 
	************************************************/
	class motion_logger
	{
	
	public:
		/**
		* Motion logger for writing various data into a .csv file. Expects double for fixed
		* data formats and strings for arbitrary data.
		* IMPORTANT: Exceptions will only be thrown when header parts mismatch the given number
		* of data sets in each category. During logging, data provided in excess of the specified
		* number of entries will be ignored, while providing too few entries will lead to padding
		* with "0.0" or "none" entries.
		* 
		* @param filename: file for the logger to write into
		* @param num_joint_data: number of joint data sets (1 set = 7 entries)
		* @param num_vel_acc_data: number of velocity/acceleration data sets (1 set = 3 entries)
		* @param num_ft_data: number of force-torque data sets (1 set = 6 entries);
		* @param num_timestamps: number of timestamps (single entries)
		* @param size_arbitrary: length of additional arbitrary data (= number of entries)
		**/
		motion_logger
		(std::string& filename,
			int num_joint_data,
			int num_vel_acc_data,
			int num_ft_data,
			int num_timestamps,
			int size_arbitrary);

		/**
		* Starts logging and writes the header into the target file provided at construction.
		* Vectors for data categories will be checked against the numbers of data sets specified
		* at construction, an exception will be thrown if any mismatches are detected.
		* 
		* @param joint_data_header: headers for each of the joint data sets (1 set = 7 entries)
		* @param vel_acc_data_header: headers for each of the velocity/acceleration data sets (1 set = 3 entries)
		* @param ft_data_header: headers for each of the force-torque data sets (1 set = 6 entries);
		* @param timestamp_header: headers for each of the timestamps (single entries)
		* @param arbitrary_header: header for arbitrary data (single entries)
		**/
		void start_logging
		(std::vector<std::array<std::string, 7>> joint_data_header,
			std::vector<std::array<std::string, 3>> vel_acc_data_header,
			std::vector<std::array<std::string, 6>> ft_data_header,
			std::vector<std::string> timestamp_header,
			std::vector<std::string> arbitrary_header);


		/**
		* Stops logging and closes the file
		**/
		void stop_logging();


		/**
		* Writes a single line into the log, using the provided data and adding padding values of
		* "0.0" ("none" for arbitrary data) if less data sets than the initially specified number
		* are provided. Will print warnings for excess or missing data sets, but not throw an exception.
		* 
		* @param joint_data: vector of joint data sets (1 set = 7 entries)
		* @param vel_acc_data: vector of velocity/acceleration data sets (1 set = 3 entries)
		* @param ft_data: vector of force-torque data sets (1 set = 6 entries);
		* @param timestamps: vector of timestamps (single entries)
		* @param arbitrary_data: vector of arbitrary data (single entries)
		**/
		void log(std::vector<std::array<double, 7>> joint_data,
			std::vector<std::array<double, 3>> vel_acc_data,
			std::vector<std::array<double, 6>> ft_data,
			std::vector<double> timestamps,
			std::vector<std::string> arbitrary_data);


	private:
		int num_joint_data_;
		int num_vel_acc_data_;
		int num_ft_data_;
		int num_timestamps_;
		int size_arbitrary_;
		std::string filename_;

		std::ofstream logger_;
	};
}