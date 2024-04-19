#include "logger.hpp"

#include <iostream>
#include <sstream>

namespace logging
{
	logger::logger
	(std::string& filename,
		int num_joint_data,
		int num_cart_data,
		int num_ft_data,
		int num_timestamps,
		int size_arbitrary)
		:
		filename_(filename),
		num_joint_data_(num_joint_data),
		num_cart_data_(num_cart_data),
		num_ft_data_(num_ft_data),
		num_timestamps_(num_timestamps),
		size_arbitrary_(size_arbitrary)
	{}


	void logger::start_logging(std::vector<std::array<std::string, 7>>* joint_data_header,
		std::vector<std::array<std::string, 3>>* cart_data_header,
		std::vector<std::array<std::string, 6>>* ft_data_header,
		std::vector<std::string>* timestamp_header,
		std::vector<std::string>* arbitrary_header)
	{

		//first, we check for inconsistencies with the header (bc these will mess up our log pretty badly and render it useless)
		bool bad = false;
		std::string baddies("");

		if (num_joint_data_ && joint_data_header->size() != num_joint_data_)
		{
			bad = true;
			baddies.append(" joints");
		}
		if (num_cart_data_ && cart_data_header->size() != num_cart_data_) {
			bad = true;
			baddies.append(" velocities_accelerations");
		}
		if (num_ft_data_ && ft_data_header->size() != num_ft_data_) {
			bad = true;
			baddies.append(" forces_torques");
		}
		if (num_timestamps_ && timestamp_header->size() != num_timestamps_) {
			bad = true;
			baddies.append(" timestamps");
		}
		if (size_arbitrary_ && arbitrary_header->size() != size_arbitrary_) {
			bad = true;
			baddies.append(" arbitrary_data");
		}

		if (bad)
		{
			throw std::runtime_error("LOGGER ERROR: Header mismatches specified number of elements in the following categories: " + baddies);
		}


		//now, we build the header from all the input parameters
		std::string header("");

		if (num_joint_data_){
			for (int i = 0; i < num_joint_data_; i++) {
				std::array<std::string, 7> jd(joint_data_header->at(i));
				header.append(jd[0] + "," + jd[1] + "," + jd[2] + "," + jd[3] + "," + jd[4] + "," + jd[5] + "," + jd[6]);
				if ((num_joint_data_ - i) > 1) {
					header.append(",");
				}
			}
		}

		if (num_cart_data_) {
			if (num_joint_data_) {
				header.append(",");
			}
			for (int i = 0; i < num_cart_data_; i++) {
				std::array<std::string, 3> vd(cart_data_header->at(i));
				header.append(vd[0] + "," + vd[1] + "," + vd[2]);
				if ((num_cart_data_ - i) > 1) {
					header.append(",");
				}
			}
		}

		if (num_ft_data_) {
			if (num_joint_data_ || num_cart_data_) {
				header.append(",");
			}
			for (int i = 0; i < num_ft_data_; i++) {
				std::array<std::string, 6> fd(ft_data_header->at(i));
				header.append(fd[0] + "," + fd[1] + "," + fd[2] + "," + fd[3] + "," + fd[4] + "," + fd[5]);
				if ((num_ft_data_ - i) > 1) {
					header.append(",");
				}
			}
		}

		if (num_timestamps_) {
			if (num_joint_data_ || num_cart_data_ || num_ft_data_) {
				header.append(",");
			}
		}
		header.append(timestamp_header->at(0));
		if (num_timestamps_ > 1) {
			for (int i = 1; i < num_timestamps_; i++) {
				header.append("," + timestamp_header->at(i));
			}
		}

		if (size_arbitrary_) {
			if (num_joint_data_ || num_cart_data_ || num_ft_data_ || num_timestamps_) {
				header.append(",");
			}
			header.append(arbitrary_header->at(0));
		}
		if (size_arbitrary_ > 1) {
			for (int i = 1; i < size_arbitrary_; i++) {
				header.append("," + arbitrary_header->at(i));
			}
		}


		//finally, we can get to logging
		logger_.open(filename_);
		logger_ << header << "\n";
	}


	void logger::stop_logging() {
		logger_.close();
	}


	void logger::log
	(std::vector<std::array<double, 7>>* joint_data,
		std::vector<std::array<double, 3>>* cart_data,
		std::vector<std::array<double, 6>>* ft_data,
		std::vector<double>* timestamps,
		std::vector<std::string>* arbitrary_data)
	{

		// let's deal with missing or too many entries and padding first
		int j_pad = 0;
		int c_pad = 0;
		int f_pad = 0;
		int t_pad = 0;
		int a_pad = 0;
		bool miss = false;
		std::string misses("");
		bool over = false;
		std::string excess("");


		if (num_joint_data_ && joint_data->size() > num_joint_data_) {
			over = true;
			excess.append(" joints");
		}
		if (num_cart_data_ && cart_data->size() > num_cart_data_) {
			over = true;
			excess.append(" cartesian");
		}
		if (num_ft_data_ && ft_data->size() > num_ft_data_) {
			over = true;
			excess.append(" forces_torques");
		}
		if (num_timestamps_ && timestamps->size() > num_timestamps_) {
			over = true;
			excess.append(" timestamps");
		}
		if (size_arbitrary_ && arbitrary_data->size() > size_arbitrary_) {
			over = true;
			excess.append(" arbitrary_data");
		}
		if (over) {
			std::cout << "LOGGER WARNING: Too many elements in the following categories: " + misses << std::endl;
			std::cout << "Excess entries will be ignored, log will only contain entries up to specified numbers." << std::endl;
		}


		if (num_joint_data_ && joint_data->size() < num_joint_data_) {
			j_pad = num_joint_data_ - joint_data->size();
			miss = true;
			misses.append(" joints");
		}
		if (num_cart_data_ && cart_data->size() < num_cart_data_) {
			c_pad = num_joint_data_ - cart_data->size();
			miss = true;
			misses.append(" cartesian");
		}
		if (num_ft_data_ && ft_data->size() < num_ft_data_) {
			f_pad = num_ft_data_ - ft_data->size();
			miss = true;
			misses.append(" forces_torques");
		}
		if (num_timestamps_ && timestamps->size() < num_timestamps_) {
			t_pad = num_timestamps_ - timestamps->size();
			miss = true;
			misses.append(" timestamps");
		}
		if (size_arbitrary_ && arbitrary_data->size() < size_arbitrary_) {
			a_pad = size_arbitrary_ - arbitrary_data->size();
			miss = true;
			misses.append(" arbitrary_data");
		}
		if (miss) {
			std::cout << "LOGGER WARNING: Missing elements in the following categories: " + misses << std::endl;
			std::cout << "Log will contain padding with zero values or none entries." << std::endl;
		}


		// same as with the header, we add all values into the line that will then be logged
		std::ostringstream line;

		if (num_joint_data_) {
			for (int i = 0; i < joint_data->size(); i++) {
				std::array<double, 7> jd(joint_data->at(i));
				line << jd[0] << "," << jd[1] << "," << jd[2] << "," << jd[3] << "," << jd[4] << "," << jd[5] << "," << jd[6];
				if ((joint_data->size() - i) > 1) {
					line << ",";
				}
			}
			if (j_pad) {
				for (int i = 0; i < j_pad; i++) {
					line << ",0.0,0.0,0.0,0.0,0.0,0.0,0.0";
				}
			}
		}

		if (num_cart_data_) {
			if (num_joint_data_) {
				line << ",";
			}
			for (int i = 0; i < cart_data->size(); i++) {
				std::array<double, 3> vd(cart_data->at(i));
				line << vd[0] << "," << vd[1] << "," << vd[2];
				if ((cart_data->size() - i) > 1) {
					line<< ",";
				}
			}
			if (c_pad) {
				for (int i = 0; i < c_pad; i++) {
					line << ",0.0,0.0,0.0";
				}
			}
		}

		if (num_ft_data_) {
			if (num_joint_data_ || num_cart_data_) {
				line << ",";
			}
			for (int i = 0; i < ft_data->size(); i++) {
				std::array<double, 6> fd(ft_data->at(i));
				line << fd[0] << "," << fd[1] << "," << fd[2] << "," << fd[3] << "," << fd[4] << "," << fd[5];
				if ((ft_data->size() - i) > 1) {
					line << ",";
				}
			}
			if (f_pad) {
				for (int i = 0; i < f_pad; i++) {
					line << ",0.0,0.0,0.0,0.0,0.0,0.0";
				}
			}
		}

		if (num_timestamps_) {
			if (num_joint_data_ || num_cart_data_ || num_ft_data_) {
				line << ",";
			}
			for (int i = 0; i < timestamps->size(); i++) {
				line << timestamps->at(i);
				if ((timestamps->size() - i) > 1) {
					line << ",";
				}
			}
			if (t_pad) {
				for (int i = 0; i < t_pad; i++) {
					line << ",0.0";
				}
			}
		}

		if (size_arbitrary_) {
			if (num_joint_data_ || num_cart_data_ || num_ft_data_ || num_timestamps_) {
				line << ",";
			}
			for (int i = 0; i < arbitrary_data->size(); i++) {
				line << arbitrary_data->at(i);
				if ((arbitrary_data->size() - i) > 1) {
					line << ",";
				}
			}
			if (a_pad) {
				for (int i = 0; i < a_pad; i++) {
					line << ",none";
					}
			}
		}

		logger_ << line.str() << "\n";
	}
}

