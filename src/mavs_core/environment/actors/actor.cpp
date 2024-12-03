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
#include <mavs_core/environment/actors/actor.h>

#include <iostream>

#include <glm/gtx/euler_angles.hpp>

namespace mavs {
namespace actor {
	
Actor::Actor() {
	complete_ = false;
	speed_ = 1.4; //m/s
	current_waypoint_ = 0;
	tolerance_ = 0.1;
	orientation_ = glm::quat(1, 0, 0, 0);
	lock_to_ground_ = false;
	auto_update_ = true;
}

Actor::~Actor() {

}

void Actor::Update(double dt) {

	if (path_.NumWaypoints() <= 0) {
		return;
	}

	glm::dvec2 goal = path_.GetWaypoint(current_waypoint_);
	glm::dvec2 p(position_.x, position_.y);
	glm::dvec2 to_goal = goal - p;
	double dist_to_goal = length(to_goal);
	if (dist_to_goal<tolerance_) {
		current_waypoint_++;
		if (current_waypoint_ >= (int)path_.NumWaypoints()) {
			current_waypoint_ = 0;
		}
		goal = path_.GetWaypoint(current_waypoint_);
		to_goal = goal - p;
		dist_to_goal = length(to_goal);
	}

	glm::vec3 euler = glm::eulerAngles(orientation_);
	float yaw = euler.z;

	if (dist_to_goal > 0.0) {
		to_goal = to_goal / dist_to_goal;
		yaw = (float)atan2(to_goal.y, to_goal.x);
	}

	orientation_ = glm::eulerAngleYXZ(0.0f, 0.0f, yaw);

	double dist_traveled = speed_ * dt;
	if (dist_traveled > (dist_to_goal + tolerance_))dist_traveled = dist_to_goal + 0.8*tolerance_;
	p = p + to_goal * dist_traveled;
	position_.x = (float)p.x;
	position_.y = (float)p.y;
}

void Actor::LoadAnimation(std::string infile) {

}

} //namespace actor
} //namespace mavs