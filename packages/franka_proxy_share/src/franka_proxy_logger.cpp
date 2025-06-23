/**
 *************************************************************************
 *
 * @file franka_proxy_logger.cpp
 *
 * Small logger for franka motions
 *
 ************************************************************************/
#include "franka_proxy_logger.hpp"

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


// LOGGING METHODS

void logger::start_logging(
	const std::vector<std::string>* joint_data_header,
	const std::vector<std::string>* cart_data_header,
	const std::vector<std::string>* ft_data_header,
	const std::vector<std::string>* single_header,
	const std::vector<std::string>* arbitrary_header)
{
	// initial cleanup of leftover data
	// (useful if restarting franka_proxy_logging with the same logger object)
	clear_all();
	discard_log();


	// check for inconsistencies with the header
	// (throws, bc these will mess up the log pretty badly and render it useless)
	bool bad = false;
	std::string baddies;

	if (num_joint_data_ && (!joint_data_header || static_cast<int>(joint_data_header->size()) != (num_joint_data_ * 7)))
	{
		bad = true;
		baddies.append(" joints");
	}
	if (num_cart_data_ && (!cart_data_header || static_cast<int>(cart_data_header->size()) != (num_cart_data_ * 6)))
	{
		bad = true;
		baddies.append(" cartesian");
	}
	if (num_ft_data_ && (!ft_data_header || static_cast<int>(ft_data_header->size()) != (num_ft_data_ * 6)))
	{
		bad = true;
		baddies.append(" forces_torques");
	}
	if (num_single_ && (!single_header || static_cast<int>(single_header->size()) != num_single_))
	{
		bad = true;
		baddies.append(" single_values");
	}
	if (size_arbitrary_ && (!arbitrary_header || static_cast<int>(arbitrary_header->size()) != size_arbitrary_))
	{
		bad = true;
		baddies.append(" arbitrary_data");
	}

	if (bad)
		throw std::runtime_error(
			"LOGGER ERROR: Header mismatches specified number of elements in the following categories: " + baddies);


	// build header and write it into the buffer
	std::vector<std::string> header;
	header.reserve(
		num_joint_data_ * 7 +
		num_cart_data_ * 6 +
		num_ft_data_ * 6 +
		num_single_ +
		size_arbitrary_
	);

	for (int i = 0; i < (num_joint_data_ * 7); i++)
		header.emplace_back(joint_data_header->at(i));
	for (int i = 0; i < (num_cart_data_ * 6); i++)
		header.emplace_back(cart_data_header->at(i));
	for (int i = 0; i < (num_ft_data_ * 6); i++)
		header.emplace_back(ft_data_header->at(i));
	for (int i = 0; i < num_single_; i++)
		header.emplace_back(single_header->at(i));
	for (int i = 0; i < size_arbitrary_; i++)
		header.emplace_back(arbitrary_header->at(i));

	write_line(header);
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
	check_data();

	// build line to be logged
	std::vector<std::string> line;
	line.reserve(
		num_joint_data_ * 7 +
		num_cart_data_ * 6 +
		num_ft_data_ * 6 +
		num_single_ +
		size_arbitrary_
	);

	for (int i = 0; i < num_joint_data_; i++)
		for (int j = 0; j < 7; j++)
			line.emplace_back(std::to_string(joint_data_[i][j]));
	for (int i = 0; i < num_cart_data_; i++)
		for (int j = 0; j < 6; j++)
			line.emplace_back(std::to_string(cart_data_[i][j]));
	for (int i = 0; i < num_ft_data_; i++)
		for (int j = 0; j < 6; j++)
			line.emplace_back(std::to_string(ft_data_[i][j]));
	for (int i = 0; i < num_single_; i++)
		line.emplace_back(std::to_string(single_data_[i]));
	for (int i = 0; i < size_arbitrary_; i++)
		line.emplace_back(arbitrary_data_[i]);

	// write into buffer and clear everything for next line
	write_line(line);
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


// METHODS FOR ADDING DATA

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


// UTILITY METHODS

void logger::write_line(std::vector<std::string>& data)
{
	for (size_t i = 0; i < data.size(); i++)
	{
		log_ << data[i];
		if (i < (data.size() - 1)) log_ << ",";
	}
	log_ << '\n';
}


void logger::check_data()
{
	int j_pad = num_joint_data_ - static_cast<int>(joint_data_.size());
	int c_pad = num_cart_data_ - static_cast<int>(cart_data_.size());
	int f_pad = num_ft_data_ - static_cast<int>(ft_data_.size());
	int s_pad = num_single_ - static_cast<int>(single_data_.size());
	int a_pad = size_arbitrary_ - static_cast<int>(arbitrary_data_.size());
	bool miss = false;
	std::string misses;
	bool over = false;
	std::string excess;


	if (j_pad < 0)
	{
		over = true;
		excess.append(" joints");
	}
	else if (j_pad > 0)
	{
		miss = true;
		misses.append(" joints");
		for (int i = 0; i < j_pad; i++) joint_data_.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	}

	if (c_pad < 0)
	{
		over = true;
		excess.append(" cartesian");
	}
	else if (c_pad > 0)
	{
		miss = true;
		misses.append(" cartesian");
		for (int i = 0; i < c_pad; i++) cart_data_.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	}

	if (f_pad < 0)
	{
		over = true;
		excess.append(" forces_torques");
	}
	else if (f_pad > 0)
	{
		miss = true;
		misses.append(" forces_torques");
		for (int i = 0; i < f_pad; i++) ft_data_.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	}

	if (s_pad < 0)
	{
		over = true;
		excess.append(" single_values");
	}
	else if (s_pad > 0)
	{
		miss = true;
		misses.append(" single_values");
		for (int i = 0; i < s_pad; i++) single_data_.emplace_back(0.0);
	}

	if (a_pad < 0)
	{
		over = true;
		excess.append(" arbitrary_data");
	}
	else if (a_pad > 0)
	{
		miss = true;
		misses.append(" arbitrary_data");
		for (int i = 0; i < a_pad; i++) arbitrary_data_.emplace_back("none");
	}


	if (over)
		std::cout << "LOGGER WARNING: Too many elements in the following categories: " + excess << '\n'
			<< "Excess entries will be ignored, log will only contain entries up to specified numbers." << '\n';
	if (miss)
		std::cout << "LOGGER WARNING: Missing elements in the following categories: " + misses << '\n'
			<< "Log will contain padding with zero values or none entries." << '\n';
}
} /* namespace franka_proxy */
