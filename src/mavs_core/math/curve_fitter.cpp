/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
#include "mavs_core/math/curve_fitter.h"
#include <math.h>

namespace mavs {
namespace math {
namespace curve_fit {

CurveFitter::CurveFitter() {
	Init();
}

void CurveFitter::Reset() {
	Init();
}

void CurveFitter::Init() {
	num_params_ = 0;
	num_iter_ = 0;
	rmse_last_ = 0.0;
	rmse_threshold_ = 0.001f;
	max_iterations_ = 1000;
	show_warnings_ = false;
	warnings_ = 0;
	params_.clear();
	x_in_.clear();
	y_in_.clear();
	y_fit_.clear();
}

std::vector<double> CurveFitter::Solve(double(*func)(double, std::vector<double>), std::vector<double>(*jac)(double, std::vector<double>), std::vector<double> x, std::vector<double> y, std::vector<double> p_init) {
	// get the number of parameters to solve fore
	num_params_ = (int)p_init.size();
	params_ = p_init;

	// check that the input data are valid
	if (x.size() == 0 || y.size() == 0)return params_;
	if (x.size() != y.size())return params_;

	// create the sample array to fit
	num_samples_ = (int)x.size();
	std::vector<Sample> samples;
	for (int i = 0; i < num_samples_; i++) samples.push_back(Sample(x[i], y[i]));

	mavs::math::Matrix residual(num_samples_, 1);
	mavs::math::Matrix jacobian(num_samples_, num_params_);

	rmse_last_ = 0.0;
	for (int iteration = 0; iteration < max_iterations_; ++iteration) {
		// compute the residual and least squares error, and break if below threshold.
		double rmse = 0.0;
		for (int isample = 0; isample < num_samples_; ++isample) {
			Sample s = samples[isample];

			residual(isample, 0) = (s.y - func(s.x, params_));
			rmse += residual(isample, 0) * residual(isample, 0);

			std::vector<double> temp = jac(s.x, params_);

			for (int iparam = 0; iparam < num_params_; ++iparam) {
				jacobian(isample, iparam) = temp[iparam];
			}
		}
		rmse = sqrt(rmse / float(num_samples_));

		if (rmse < rmse_threshold_) {
			num_iter_ = iteration + 1;
			break;
		}
		if (rmse == rmse_last_) {
			NonConvergeWarning();
			num_iter_ = iteration + 1;
			break;
		}
		rmse_last_ = rmse;

		mavs::math::Matrix JtJ = jacobian.Transpose() * jacobian;
		mavs::math::Matrix Jtr = jacobian.Transpose() * residual;
		mavs::math::Matrix h = JtJ.Inverse() * Jtr;
		for (int iparam = 0; iparam < num_params_; ++iparam) params_[iparam] += h(iparam, 0);

		if (iteration == max_iterations_ - 1) {
			MaxIterationWarning();
			num_iter_ = max_iterations_;
		}
	}

	x_in_ = x;
	y_in_ = y;
	for (int i = 0; i < (int)x_in_.size(); i++)y_fit_.push_back(func(x[i], params_));

	return params_;
}

void CurveFitter::PrintFit() const {
	for (int i = 0; i < (int)x_in_.size(); i++) {
		std::cout << x_in_[i] << " " << y_in_[i] << " " << y_fit_[i] << std::endl;
	}
}

void CurveFitter::NonConvergeWarning() {
	if (show_warnings_)std::cout << "WARNING: SOLUTION FAILED TO IMPROVE AFTER " << (num_iter_ + 1) << " ITERATIONS. CONSIDER INCREASING THE CONVERGENCE TRHESHOLD." << std::endl;
	warnings_ = 1;
}

void CurveFitter::MaxIterationWarning() {
	if (show_warnings_)std::cout << "WARNING: MAX NUMBER OF ITERATIONS, " << max_iterations_ << " REACHED. CONSIDERING INCREASING THE MAX ITERATIONS." << std::endl;
	warnings_ = 2;
}

void CurveFitter::PrintResults() const {
	printf("Iterations: %d\n", num_iter_);
	printf("RMSE: %f\n", rmse_last_);
	printf("Parameters: ");
	for (int i = 0; i < (int)params_.size(); i++) printf(" %f ", params_[i]);
	printf("\n");
	if (warnings_ == 1) {
		printf("Warnings: HIGHRMSE \n");
	}
	else if (warnings_ == 2) {
		printf("Warnings: MAXITERATION \n");
	}
	else {
		printf("Warnings: NONE \n");
	}
}

} // namespace curve_fit
} //namespace math
} // namespace mavs