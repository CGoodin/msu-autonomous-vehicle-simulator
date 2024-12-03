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
#include <mavs_core/terrain_generator/species.h>

#include <math.h>
#include <stdlib.h>

#include <iostream>

namespace mavs {
namespace terraingen {

Species::Species() {
	new_plants_per_year_per_meter_ = 10;
	relative_growth_rate_ = 0.15f; //exponential growth
	max_height_ = 5.0f; //meters
	diameter_height_ratio_ = 0.5f; //unitless = diameter = ratio*height
	mesh_default_height_ = 1.0f;
}

float Species::GetNextHeight(float current_height) {
	float age_current = log(1.0f - (current_height / max_height_)) / (-relative_growth_rate_);
	float new_age = age_current + 1.0f;
	float new_height = max_height_ * (1.0f - exp(-relative_growth_rate_ * new_age));
	return new_height;
}

int Species::GetNumNew(float area) {
	float newfac = area * new_plants_per_year_per_meter_;
	float new_over = newfac - floor(newfac);
	float test_val = ((float)rand() / (RAND_MAX));
	int num_new = (int)floor(newfac);
	if (new_over > test_val)num_new = num_new + 1;
	return num_new;
}

} //namespace environment
} //namespace mavs