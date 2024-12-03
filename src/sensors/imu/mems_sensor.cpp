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
#include <sensors/imu/mems_sensor.h>

#include <stdlib.h>
#include <math.h>

#include <algorithm>

namespace mavs {
namespace sensor {
namespace imu {

void MemsSensor::SetAxisMisalignment(glm::vec3 misalign) {
	axis_misalignment_ = misalign;
	alignment_matrix_[0][0] = 1.0f;
	alignment_matrix_[0][1] = misalign.y / 100.0f;
	alignment_matrix_[0][2] = misalign.z / 100.0f;
	alignment_matrix_[1][0] = misalign.x / 100.0f;
	alignment_matrix_[1][1] = 1.0f;
	alignment_matrix_[1][2] = misalign.z / 100.0f;
	alignment_matrix_[2][0] = misalign.x / 100.0f;
	alignment_matrix_[2][1] = misalign.y / 100.0f;
	alignment_matrix_[2][2] = 1.0f;
}

float MemsSensor::Filter1(float z, float sample_rate) {
	float h = 1.0f / (1.0f + ((2.0f/sample_rate)-1.0f)*(1.0f / z));
	return h;
}

float MemsSensor::Filter2(float z) {
	float h = 1.0f / (1.0f - (1.0f / z));
	return h;
}



glm::vec3 MemsSensor::Update(glm::vec3 input, float temperature, float sample_rate) {

	glm::vec3 output(0.0f, 0.0f, 0.0f);

	glm::vec3 b = alignment_matrix_ * input + constant_bias_;

	float w1 = ((float)rand() / (RAND_MAX)) + 1; //0 to 1
	float w = 2.0f*(w1 - 0.5f); //-1 to 1

	//float h1 = 1.0f; // filter?
	glm::vec3 beta1 = w * bias_instability_;
	for (int i = 0; i < 3; i++) {
		beta1[i] = Filter1(beta1[i], sample_rate);
	}

	glm::vec3 beta2 = (w*(float)sqrt(0.5f*sample_rate))*noise_density_;

	//float h2 = 1.0f; //filter?
	glm::vec3 beta3 = (float)(w / sqrt(0.5f*sample_rate))*random_walk_;
	for (int i = 0; i < 3; i++) {
		beta3[i] = Filter2(beta3[i]);
	}
	glm::vec3 env_noise = (temperature - 25.0f)*temperature_bias_;

	glm::vec3 c = beta1 + beta2 + beta3 + env_noise + b;

	glm::vec3 one(1.0f, 1.0f, 1.0f);
	glm::vec3 scale_factor_error = one + ((temperature - 25.0f) / 100.0f)*temperature_scale_factor_;

	glm::vec3 d(c.x * scale_factor_error.x, c.y * scale_factor_error.y, c.z * scale_factor_error.z);
	glm::vec3 e;
	for (int i = 0; i < 3; i++) {
		e[i] = std::max(std::min(measurement_range_, d[i]), -measurement_range_);
		output[i] = resolution_ * round(e[i] / resolution_);
	}
	return output;
}

} //namespace imu
} //namespace sensor
} //namespace mavs