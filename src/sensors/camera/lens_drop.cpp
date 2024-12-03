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
#include <sensors/camera/lens_drop.h>
#include <iostream>
#include <time.h>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace camera{

static int lens_drop_counter_ = 1;
LensDrop::LensDrop(){
  radius_ = 2.5f/1000.0f;
  radius_pixels_ = 25;
  age_ = 0.0f;
  lifetime_ = 5.0f;
  color_ = glm::vec3(0.5f,0.5f, 0.5f);
  center_pixels_ = glm::ivec2(0, 0);
  int curr_time = (int)time(NULL);
  shape_noise_.SetSeed(curr_time+lens_drop_counter_);
  lens_drop_counter_++;
}

bool LensDrop::PixelInDrop(int pi, int pj) {
	shape_noise_.SetFrequency(0.5f / radius_pixels_);
	int x = pi - center_pixels_.x;
	int y = pj - center_pixels_.y;
	double z = shape_noise_.GetPerlin((double)(x), (double)(y));
	bool in_drop = false;
	if (z > -0.4)in_drop = true;
	return in_drop;
}

void LensDrop::Print() {
	std::cout << "Drop radius = " << radius_ << " "<<radius_pixels_<<std::endl;
	std::cout << "Drop center = (" << center_pixels_.x << ", " << center_pixels_.y << ") " << std::endl;
}

} //namespace camera
} //namespace sensor
} //namespace mavs
