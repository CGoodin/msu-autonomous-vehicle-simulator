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
#include <sensors/ins/ins.h>

#include <string>
#include <fstream>
#include <chrono>

#include <mavs_core/environment/date_time.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/messages.h>

namespace mavs {
namespace sensor {
namespace ins {

void Ins::SetPose(mavs::VehicleState &state) {
	imu_.SetPose(state);
	gps_.SetPose(state);
	current_pose_ = state;
	Sensor::SetPose(state);
}

void Ins::SetPose(glm::vec3 position, glm::quat quaternion) {
	imu_.SetPose(position,quaternion);
	gps_.SetPose(position,quaternion);
	current_pose_.pose.position = position;
	current_pose_.pose.quaternion = quaternion;
	Sensor::SetPose(position,quaternion);
}

mavs::Odometry Ins::GetOdometryMessage(){
	mavs::Odometry odom;
	odom.pose.pose.position = current_pose_.pose.position;
	odom.pose.pose.quaternion = current_pose_.pose.quaternion;
	odom.twist.twist.linear = current_pose_.twist.linear;
	odom.twist.twist.angular = current_pose_.twist.angular;
	return odom;
}

void Ins::Display() {
	const float green[3] = { 0.0f, 255.0f, 0.0f };
	float yellow[3] = { 255.0f,255.0f,0.0f };
	int dim, dim2, dim4, dim8, dim10, dim50;
	dim = 384;
	dim2 = (int)(0.5*dim);
	dim4 = (int)(0.5*dim2);
	dim8 = (int)(0.5*dim4);
	dim10 = (int)(0.1*dim);
	dim50 = (int)(0.02*dim);

	cimg_library::CImg<float> image;
	if (disp_.is_empty())disp_.assign(384, 384, "INS");
	image.assign(dim, dim, 1, 3, 0.0);
	image = 0.0f;
	image.draw_line(0, dim2, dim, dim2, green, 1);
	image.draw_line(dim2, 0, dim2, dim, green, 1);
	image.draw_line(0, 0, dim, dim, green, 1);
	image.draw_line(dim, 0, 0, dim, green, 1);
	image.draw_circle(dim2, dim2, dim4, green, 1.0, 1);
	image.draw_text(dim2 + dim50, dim50, "N", yellow);
	image.draw_text(dim2 + dim50, dim - 2 * dim50, "S", yellow);
	image.draw_text(dim - dim50, dim2, "E", yellow);
	image.draw_text(dim50, dim2, "W", yellow);

	float velocity = (float)glm::length(current_pose_.twist.linear);
	glm::vec3 lla = gps_.GetRecieverPositionLLA();
	std::string pos_str = "Lat, long = (" + mavs::utils::ToString(lla.x) + ", " + mavs::utils::ToString(lla.y) + ")";
	image.draw_text(dim50, dim50, pos_str.c_str(), yellow);
	std::string vel_str = "Velocity (m/s) = " + mavs::utils::ToString(velocity);
	image.draw_text(dim50, 3 * dim50, vel_str.c_str(), yellow);

	glm::vec3 lt = gps_.GetLookTo();
	float heading = atan2(lt.y, lt.x);

	float x = cos(heading);
	float y = sin(heading);
	float arrlen = dim10 + (float)std::min(1.0, (double)(velocity / 27.0))*dim8;
	int x_start = (int)(dim2 - 0.5*x*arrlen);
	int y_start = (int)(dim2 + 0.5*y*arrlen);
	int x_stop = (int)(dim2 + 0.5*x*arrlen);
	int y_stop = (int)(dim2 - 0.5*y*arrlen);
	image.draw_arrow(x_start, y_start, x_stop, y_stop, yellow, 1.0f, 30, 10);

	disp_.display(image);
}
} //namespace ins
} //namespace sensor
} //namespace mavs