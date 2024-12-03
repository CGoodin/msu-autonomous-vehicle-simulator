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
#include <mavs_core/terrain_generator/plant.h>

namespace mavs {
namespace terraingen {

Plant::Plant() {
	alive_ = true;
	height_ = 0.0f;
	radius_ = 0.0f;
	age_ = 0.0f;
}

void Plant::Grow(){
	height_ = species_.GetNextHeight(height_);
	radius_ = 0.5f*species_.GetDiameter(height_);
	age_ += 1.0f;
	if (age_ > species_.GetLifetime())alive_ = false;
}

} //namespace terraingen
} //namespace mavs