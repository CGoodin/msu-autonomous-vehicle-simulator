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
#include <raytracers/bounding_cylinder.h>
#include <limits>

namespace mavs {
namespace raytracer {

BoundingCylinder::BoundingCylinder() {
	float max = std::numeric_limits<float>::max();
	float min = std::numeric_limits<float>::min();
	base_center_ = glm::vec3(0.0f, 0.0f, min);
	radius_ = max;
	height_ = max;
	center_ = glm::vec3(0.0f,0.0f,0.0f);
}

BoundingCylinder::BoundingCylinder(glm::vec3 base_center, float radius, float height) {
	base_center_ = base_center;
	radius_ = radius;
	height_ = height;
	center_ = base_center_;
	center_.z += 0.5f*height;
}

BoundingCylinder::BoundingCylinder(float bsx, float bsy, float bsz,
	float radius, float height) {
	base_center_ = glm::vec3(bsx, bsy, bsz);
	radius_ = radius;
	height_ = height;
	center_ = base_center_;
	center_.z += 0.5f*height;
}

void BoundingCylinder::Transform(glm::mat3x4 aff_rot) {
	float old_height = height_;
	glm::vec4 vc(center_.x, center_.y, center_.z, 1.0f);
	glm::vec3 vcprime = aff_rot * vc;
	center_ = vcprime;

	glm::vec4 vbc(base_center_.x, base_center_.y, base_center_.z, 1.0f);
	glm::vec3 vbcprime = aff_rot * vbc;
	base_center_ = vbcprime;

	height_ = 2.0f*(center_.z - base_center_.z);
	float s = height_ / old_height;
	radius_ = s * radius_;
}

} //namespace raytracer 
} //namespace mavs