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
#include <mavs_core/terrain_generator/profile_2d.h>

#include <random>
#include <ctime>
#include <limits>
#include <iostream>
#include <fstream>

#include <mavs_core/math/constants.h>
#include <mavs_core/math/segment.h>

namespace mavs {
namespace terraingen {

Profile::Profile() {
	unsigned random_profile_seed = (unsigned)std::time(NULL);
	generator_.seed(random_profile_seed);
}

void Profile::GenerateProfile(float &length, float rms, float acl, float &res) {
	std::normal_distribution<float> distribution(0.0f, rms);
	if (length<200.0f*acl)length = 200.0f*acl;

	int n = (int)(2.0f * length / acl) + 1; // nyquist criteria
	res = length / ((float)n);

	//create initial gaussian height field
	std::vector<float> heights;
	heights.resize(n);
	for (int i = 0; i < n; i++) {
		heights[i] = distribution(generator_);
	}

	//create low pass filter
	float acl2 = acl * acl;
	std::vector<float> g;
	int goff = (int)(2.0*ceil(acl / res));
	int glen = 2 * goff + 1;
	float gscale = (float)(2.0*length / (sqrt(mavs::kPi)*n*acl));
	g.resize(glen, 0.0f);
	for (int i = 0; i < glen; i++) {
		float x = res * (goff - i);
		g[i] = (float)(gscale*exp(-0.5f*x*x / acl2));
	}

	//apply low pass filter
	profile_.resize(n);
	for (int i = 0; i < n; i++) {
		profile_[i].x = i * res;
		profile_[i].y = 0.0f;
		for (int j = 0; j < glen; j++) {
			int jp = goff - j;
			int ip = i + jp;
			while (ip < 0) {
				ip += n;
			}
			while (ip >= n) {
				ip -= n;
			}
			profile_[i].y += heights[ip] * g[j];
		}
	}
}

void Profile::GetRmsOfProfile(float &rms, float &rms_slope) {
	float summ = 0.0f;
	float slope_summ = 0.0f;
	int m = (int)profile_.size();
	for (int i = 0; i < (m - 1); i++) {
		summ += profile_[i].y*profile_[i].y;
		glm::vec2 v = profile_[i + 1] - profile_[i];
		float slope = v.y / v.x;
		slope_summ += slope * slope;
	}
	rms = sqrt(summ / (float)(m - 1));
	rms_slope = sqrt(slope_summ / (float)(m - 2));
}

float Profile::GetMinimumOfProfile() {
	float minimum = std::numeric_limits<float>::min();
	for (int i = 0; i < profile_.size(); i++) {
		if (profile_[i].y < minimum)minimum = profile_[i].y;
	}
	return minimum;
}

void Profile::WriteProfile(std::string fname) {
	std::ofstream pout(fname.c_str());
	for (int i = 0; i < (int)profile_.size(); i++) {
		pout << profile_[i].x << " " << profile_[i].y << std::endl;
	}
	pout.close();
}

float Profile::GetClosestIntersection(glm::vec2 origin, glm::vec2 direction) {
	float closest = std::numeric_limits<float>::max();
	bool intersected = false;
	for (int j = 0; j < (int)(profile_.size() - 1); j++) {
		float d = mavs::math::RaySegmentIntersection(origin, direction, profile_[j], profile_[j + 1]);
		if (d<closest && d> 0.0f) {
			closest = d;
			intersected = true;
		}
	}
	if (intersected) {
		return closest;
	}
	else {
		return -1.0f;
	}
}

} //namespace terraingen
} //namespace mavs