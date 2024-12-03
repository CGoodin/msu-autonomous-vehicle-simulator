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
#include <vehicles/autonomous_vehicle.h>

#include <algorithm>
#include <sstream>

#include <mavs_core/math/utils.h>

namespace mavs {
namespace vehicle {

AutonomousVehicle::AutonomousVehicle() {
	vehicle_ = NULL;

	current_steering_angle_degrees_ = 0.0f;
	prnd_ = 0;
	requested_speed_ = 0.0f;
	actual_speed_ = 0.0f;
	speed_diff_ = 0.0f;
	auto_mode_ = 0;
	max_steer_angle_degrees_ = 25.0f;
	append_file_opened_ = false;
	elapsed_time_ = 0.0f;
}

AutonomousVehicle::~AutonomousVehicle() {
	//if (vehicle_ != NULL)delete vehicle_;
}

void AutonomousVehicle::SetVehicle(Vehicle *veh) {
	vehicle_ = veh;
}

void AutonomousVehicle::SetRequests(nvidia::VehicleReq request) {
	current_request_ = request;
}

nvidia::VehicleFeedback AutonomousVehicle::GetFeedback() {
	return current_feedback_;
}

void AutonomousVehicle::GetFeedbackSerial(char *msg) {
	msg = (char *)&current_feedback_;
}

void AutonomousVehicle::WriteFeedbackToFile(std::string ofname) {
	std::ofstream writefile;
	writefile.open(ofname.c_str(), std::ios::binary | std::ios::out);
	writefile.write((char *)&current_feedback_, sizeof(nvidia::VehicleFeedback));
	writefile.close();
}

void AutonomousVehicle::AppendFeedbackToFile(std::string ofname) {
	if (mavs::utils::file_exists(ofname)) {
		if (!append_file_opened_) {
			std::ofstream fout;
			fout.open(ofname.c_str(), std::ios::binary | std::ios::out);
			fout.close();
			append_file_opened_ = true;
		}
	}
	std::ofstream writefile;
	writefile.open(ofname.c_str(), std::ios::binary | std::ios::out | std::ios::app);
	writefile.write((char *)&current_feedback_, sizeof(nvidia::VehicleFeedback));
	writefile.close();
}

void AutonomousVehicle::Update(environment::Environment *env) {
	float requested_steering_angle = ((float)current_request_.SteeringAngleReq) / 0.1f;
	float steering = requested_steering_angle / max_steer_angle_degrees_;
	steering = std::min(std::max(-1.0f, steering), 1.0f);
	float dt = 0.01f;
	requested_speed_ = 0.44704f*((float)current_request_.nVehReq) / 0.1f;
	actual_speed_ = (float)glm::length(vehicle_->GetState().twist.linear);
	speed_control_.SetSetpoint(requested_speed_);
	float throttle = (float)speed_control_.GetControlVariable(actual_speed_, dt);

	vehicle_->Update(env, throttle, steering, 0.0, dt);
	actual_speed_ = (float)glm::length(vehicle_->GetState().twist.linear);
	speed_diff_ = actual_speed_-requested_speed_;
	
	current_feedback_.nVeh = (uint64_t)(2.23694f*actual_speed_ / 0.1f);
	current_feedback_.PRND = 3;
	current_feedback_.nVehError = (int64_t)(2.23694f*speed_diff_ / 0.01f);
	current_feedback_.AutoMode = 1;
	current_feedback_.timestamp = (uint64_t)(elapsed_time_ / 1.0E-6);
	current_feedback_.SteeringAngle = (int64_t)(mavs::kRadToDeg*vehicle_->GetSteeringAngle() / 0.1f);
	elapsed_time_ += dt;
}

} //namespace vehicle
} //namespace mavs