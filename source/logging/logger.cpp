#include "logger.hpp"

#include <iostream>
#include <sstream>


namespace franka_proxy
{
logger::logger(
	std::string filename,
	int num_joint_data,
	int num_cart_data,
	int num_ft_data,
	int num_single,
	int size_arbitrary)
	: filename_(std::move(filename)),
	  num_joint_data_(num_joint_data),
	  num_cart_data_(num_cart_data),
	  num_ft_data_(num_ft_data),
	  num_single_(num_single),
	  size_arbitrary_(size_arbitrary)
{
}


logger::~logger()
{
	while (!logged_)
	{
		try
		{
			stop_logging();
		}
		catch (...)
		{
		}
	}
}


void logger::start_logging(
	const std::vector<std::string>* joint_data_header,
	const std::vector<std::string>* cart_data_header,
	const std::vector<std::string>* ft_data_header,
	const std::vector<std::string>* timestamp_header,
	const std::vector<std::string>* arbitrary_header)
{
	// initial cleanup of leftover data
	// (useful if restarting franka_proxy_logging with the same logger object)
	clear_all();
	log_.clear();


	// first, we check for inconsistencies with the header
	// (bc these will mess up our log pretty badly and render it useless)
	bool bad = false;
	std::string baddies;

	if (num_joint_data_ && (joint_data_header->size() / 7) != num_joint_data_ && (joint_data_header->size() % 7) != 0)
	{
		bad = true;
		baddies.append(" joints");
	}
	if (num_cart_data_ && (cart_data_header->size() / 6) != num_cart_data_ && (cart_data_header->size() % 6) != 0)
	{
		bad = true;
		baddies.append(" cartesian");
	}
	if (num_ft_data_ && (ft_data_header->size() / 6) != num_ft_data_ && (ft_data_header->size() % 6) != 0)
	{
		bad = true;
		baddies.append(" forces_torques");
	}
	if (num_single_ && static_cast<int>(timestamp_header->size()) != num_single_)
	{
		bad = true;
		baddies.append(" single_values");
	}
	if (size_arbitrary_ && static_cast<int>(arbitrary_header->size()) != size_arbitrary_)
	{
		bad = true;
		baddies.append(" arbitrary_data");
	}

	if (bad)
	{
		throw std::runtime_error(
			"LOGGER ERROR: Header mismatches specified number of elements in the following categories: " + baddies);
	}


	// now, we build the header from all the input parameters
	std::string header;

	if (num_joint_data_)
	{
		for (int i = 0; i < static_cast<int>(joint_data_header->size()); i++)
		{
			header.append(joint_data_header->at(i));
			if ((joint_data_header->size() - i) > 1)
			{
				header.append(",");
			}
		}
	}

	if (num_cart_data_)
	{
		if (num_joint_data_)
		{
			header.append(",");
		}
		for (int i = 0; i < static_cast<int>(cart_data_header->size()); i++)
		{
			header.append(cart_data_header->at(i));
			if ((cart_data_header->size() - i) > 1)
			{
				header.append(",");
			}
		}
	}

	if (num_ft_data_)
	{
		if (num_joint_data_ || num_cart_data_)
		{
			header.append(",");
		}
		for (int i = 0; i < static_cast<int>(ft_data_header->size()); i++)
		{
			header.append(ft_data_header->at(i));
			if ((ft_data_header->size() - i) > 1)
			{
				header.append(",");
			}
		}
	}

	if (num_single_)
	{
		if (num_joint_data_ || num_cart_data_ || num_ft_data_)
		{
			header.append(",");
		}
		header.append(timestamp_header->at(0));
	}
	if (num_single_ > 1)
	{
		for (int i = 1; i < static_cast<int>(timestamp_header->size()); i++)
		{
			header.append("," + timestamp_header->at(i));
		}
	}

	if (size_arbitrary_)
	{
		if (num_joint_data_ || num_cart_data_ || num_ft_data_ || num_single_)
		{
			header.append(",");
		}
		header.append(arbitrary_header->at(0));
	}
	if (size_arbitrary_ > 1)
	{
		for (int i = 1; i < static_cast<int>(arbitrary_header->size()); i++)
		{
			header.append("," + arbitrary_header->at(i));
		}
	}


	// finally, we can get to franka_proxy_logging
	log_ << header << "\n";
	logged_ = false;
}


void logger::stop_logging()
{
	if (!logged_)
	{
		logger_.open(filename_);
		logger_ << log_.str();
		logger_.close();
		log_.clear();
		logged_ = true;
	}
}


void logger::log()
{
	// let's deal with missing or too many entries and padding first
	int j_pad = 0;
	int c_pad = 0;
	int f_pad = 0;
	int t_pad = 0;
	int a_pad = 0;
	bool miss = false;
	std::string misses;
	bool over = false;
	std::string excess;


	if (num_joint_data_ && static_cast<int>(joint_data_.size()) > num_joint_data_)
	{
		over = true;
		excess.append(" joints");
	}
	if (num_cart_data_ && static_cast<int>(cart_data_.size()) > num_cart_data_)
	{
		over = true;
		excess.append(" cartesian");
	}
	if (num_ft_data_ && static_cast<int>(ft_data_.size()) > num_ft_data_)
	{
		over = true;
		excess.append(" forces_torques");
	}
	if (num_single_ && static_cast<int>(single_data_.size()) > num_single_)
	{
		over = true;
		excess.append(" single_values");
	}
	if (size_arbitrary_ && static_cast<int>(arbitrary_data_.size()) > size_arbitrary_)
	{
		over = true;
		excess.append(" arbitrary_data");
	}
	if (over)
	{
		std::cout << "LOGGER WARNING: Too many elements in the following categories: " + excess << std::endl;
		std::cout << "Excess entries will be ignored, log will only contain entries up to specified numbers." <<
			std::endl;
	}


	if (num_joint_data_ && static_cast<int>(joint_data_.size()) < num_joint_data_)
	{
		j_pad = num_joint_data_ - static_cast<int>(joint_data_.size());
		miss = true;
		misses.append(" joints");
	}
	if (num_cart_data_ && static_cast<int>(cart_data_.size()) < num_cart_data_)
	{
		c_pad = num_cart_data_ - static_cast<int>(cart_data_.size());
		miss = true;
		misses.append(" cartesian");
	}
	if (num_ft_data_ && static_cast<int>(ft_data_.size()) < num_ft_data_)
	{
		f_pad = num_ft_data_ - static_cast<int>(ft_data_.size());
		miss = true;
		misses.append(" forces_torques");
	}
	if (num_single_ && static_cast<int>(single_data_.size()) < num_single_)
	{
		t_pad = num_single_ - static_cast<int>(single_data_.size());
		miss = true;
		misses.append(" single_values");
	}
	if (size_arbitrary_ && static_cast<int>(arbitrary_data_.size()) < size_arbitrary_)
	{
		a_pad = size_arbitrary_ - static_cast<int>(arbitrary_data_.size());
		miss = true;
		misses.append(" arbitrary_data");
	}
	if (miss)
	{
		std::cout << "LOGGER WARNING: Missing elements in the following categories: " + misses << std::endl;
		std::cout << "Log will contain padding with zero values or none entries." << std::endl;
	}


	// same as with the header, we add all values into the line that will then be logged
	std::ostringstream line;

	if (num_joint_data_)
	{
		for (int i = 0; i < joint_data_.size(); i++)
		{
			std::array<double, 7> jd(joint_data_.at(i));
			line << jd[0] << "," << jd[1] << "," << jd[2] << "," << jd[3] << "," << jd[4] << "," << jd[5] << "," << jd[
				6];
			if ((joint_data_.size() - i) > 1)
			{
				line << ",";
			}
		}
		if (j_pad)
		{
			for (int i = 0; i < j_pad; i++)
			{
				line << ",0.0,0.0,0.0,0.0,0.0,0.0,0.0";
			}
		}
	}

	if (num_cart_data_)
	{
		if (num_joint_data_)
		{
			line << ",";
		}
		for (int i = 0; i < cart_data_.size(); i++)
		{
			std::array<double, 6> vd(cart_data_.at(i));
			line << vd[0] << "," << vd[1] << "," << vd[2] << "," << vd[3] << "," << vd[4] << "," << vd[5];
			if ((cart_data_.size() - i) > 1)
			{
				line << ",";
			}
		}
		if (c_pad)
		{
			for (int i = 0; i < c_pad; i++)
			{
				line << ",0.0,0.0,0.0,0.0,0.0,0.0";
			}
		}
	}

	if (num_ft_data_)
	{
		if (num_joint_data_ || num_cart_data_)
		{
			line << ",";
		}
		for (int i = 0; i < ft_data_.size(); i++)
		{
			std::array<double, 6> fd(ft_data_.at(i));
			line << fd[0] << "," << fd[1] << "," << fd[2] << "," << fd[3] << "," << fd[4] << "," << fd[5];
			if ((ft_data_.size() - i) > 1)
			{
				line << ",";
			}
		}
		if (f_pad)
		{
			for (int i = 0; i < f_pad; i++)
			{
				line << ",0.0,0.0,0.0,0.0,0.0,0.0";
			}
		}
	}

	if (num_single_)
	{
		if (num_joint_data_ || num_cart_data_ || num_ft_data_)
		{
			line << ",";
		}
		for (int i = 0; i < single_data_.size(); i++)
		{
			line << single_data_.at(i);
			if ((single_data_.size() - i) > 1)
			{
				line << ",";
			}
		}
		if (t_pad)
		{
			for (int i = 0; i < t_pad; i++)
			{
				line << ",0.0";
			}
		}
	}

	if (size_arbitrary_)
	{
		if (num_joint_data_ || num_cart_data_ || num_ft_data_ || num_single_)
		{
			line << ",";
		}
		for (int i = 0; i < arbitrary_data_.size(); i++)
		{
			line << arbitrary_data_.at(i);
			if ((arbitrary_data_.size() - i) > 1)
			{
				line << ",";
			}
		}
		if (a_pad)
		{
			for (int i = 0; i < a_pad; i++)
			{
				line << ",none";
			}
		}
	}

	log_ << line.str() << "\n";

	clear_all();
}


void logger::clear_all()
{
	joint_data_.clear();
	cart_data_.clear();
	ft_data_.clear();
	single_data_.clear();
	arbitrary_data_.clear();
}


void logger::discard_log()
{
	log_.clear();
	logged_ = true;
}


void logger::add_joint_data(const std::array<double, 7>& data)
{
	joint_data_.push_back(data);
}

void logger::add_joint_data(const Eigen::Matrix<double, 7, 1>& data)
{
	add_joint_data(std::array<double, 7>{data(0), data(1), data(2), data(3), data(4), data(5), data(6)});
}

void logger::add_joint_data(
	const double j0, const double j1, const double j2, const double j3,
	const double j4, const double j5, const double j6)
{
	add_joint_data(std::array<double, 7>{j0, j1, j2, j3, j4, j5, j6});
}


void logger::add_cart_data(const std::array<double, 6>& data)
{
	cart_data_.push_back(data);
}

void logger::add_cart_data(const Eigen::Matrix<double, 6, 1>& data)
{
	add_cart_data(std::array<double, 6>{data(0), data(1), data(2), data(3), data(4), data(5)});
}

void logger::add_cart_data(
	const double x0, const double x1, const double x2,
	const double x3, const double x4, const double x5)
{
	add_cart_data(std::array<double, 6>{x0, x1, x2, x3, x4, x5});
}


void logger::add_ft_data(const std::array<double, 6>& data)
{
	ft_data_.push_back(data);
}

void logger::add_ft_data(const Eigen::Matrix<double, 6, 1>& data)
{
	add_ft_data(std::array<double, 6>{data(0), data(1), data(2), data(3), data(4), data(5)});
}

void logger::add_ft_data(
	const double fx, const double fy, const double fz,
	const double tx, const double ty, const double tz)
{
	add_ft_data(std::array<double, 6>{fx, fy, fz, tx, ty, tz});
}


void logger::add_single_data(const double data)
{
	single_data_.push_back(data);
}


void logger::add_arbitrary_data(const std::string& data)
{
	arbitrary_data_.push_back(data);
}

void logger::add_arbitrary_data(const std::vector<std::string>& data)
{
	for (const auto& i : data)
		add_arbitrary_data(i);
}

}
