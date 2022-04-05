#ifndef CSV_DATA_STRUCT
#define CSV_DATA_STRUCT

struct csv_data {
	double duration;

	bool aborted;

	std::array<double, 6> selection_array;

	std::array<double, 6> k_p_f;
	std::array<double, 6> k_i_f;
	std::array<double, 6> k_d_f;
	std::array<double, 6> k_p_p;
	std::array<double, 6> k_i_p;
	std::array<double, 6> k_d_p;

	Eigen::Matrix<double, 6, 1> ise_position;
	Eigen::Matrix<double, 6, 1> ise_force;

	Eigen::Matrix<double, 6, 1> itae_position;
	Eigen::Matrix<double, 6, 1> itae_force;

	std::vector<Eigen::Matrix<double, 6, 7 >> zero_jacobian;

	std::vector<Eigen::Matrix<double, 6, 1 >> o_F_ext_hat_K;
	std::vector<Eigen::Matrix<double, 6, 1>> force_desired;
	std::vector<Eigen::Matrix<double, 6, 1>> force_error;
	std::vector<Eigen::Matrix<double, 6, 1>> force_error_integral;
	std::vector<Eigen::Matrix<double, 6, 1>> force_error_diff_filtered;

	std::vector<Eigen::Matrix<double, 6, 1>> force_command;
	std::vector<Eigen::Matrix<double, 6, 1>> force_command_p;
	std::vector<Eigen::Matrix<double, 6, 1>> force_command_i;
	std::vector<Eigen::Matrix<double, 6, 1>> force_command_d;

	std::vector<Eigen::Matrix<double, 6, 1>> position;
	std::vector<Eigen::Matrix<double, 6, 1>> position_error;
	std::vector<Eigen::Matrix<double, 6, 1>> position_error_integral;
	std::vector<Eigen::Matrix<double, 6, 1>> position_error_diff_filtered;

	std::vector<Eigen::Matrix<double, 6, 1>> position_command;
	std::vector<Eigen::Matrix<double, 6, 1>> position_command_p;
	std::vector<Eigen::Matrix<double, 6, 1>> position_command_i;
	std::vector<Eigen::Matrix<double, 6, 1>> position_command_d;

	std::vector<Eigen::Matrix<double, 7, 1>> tau_command;
	std::vector<Eigen::Matrix<double, 7, 1>> tau_command_filtered;
	std::vector<Eigen::Matrix<double, 7, 1>> coriolis;
};

#endif