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
#include <sensors/annotation.h>

namespace mavs {
namespace sensor {

Annotation::Annotation(){
	class_number_ = -1;
}

Annotation::Annotation(std::string n, int min_x, int min_y, int max_x, int max_y) {
	name_ = n;
	pix_ll_.x = min_x;
	pix_ll_.y = min_y;
	pix_ur_.x = max_x;
	pix_ur_.y = max_y;
}

Annotation::Annotation(std::string n, glm::vec3 p, int num) {
	name_ = n;
	ll_ = p;
	ur_ = p;
	class_number_ = num;
}

Annotation::Annotation(std::string name, int r, int g, int b) {
	name_ = name;
	color_ = glm::ivec3(r, g, b);
}

} //namespace sensor
} //namespace mavs