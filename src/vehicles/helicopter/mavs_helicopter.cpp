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

// class definition
#include "vehicles/helicopter/mavs_helicopter.h"
#include "mavs_core/math/utils.h"

namespace mavs {
namespace vehicle {

MavsHelicopter::MavsHelicopter() {
	current_lift_ = glm::vec3(0.0f, 0.0f, 0.0f);
	max_lift_ = 0.0f;
	use_rp3d_ = true;
	veh_id_ = -1;
	hover_ = false;
	g_ = 9.806f;
}

MavsHelicopter::~MavsHelicopter() {

}

void MavsHelicopter::Load(std::string input_file, environment::Environment* env, glm::vec3 init_pos, glm::quat init_ori) {

	vehicle_.Load(input_file);

	vehicle_.SetPosition((double)init_pos.x, (double)init_pos.y, (double)init_pos.z);
	vehicle_.SetOrientation((double)init_ori.w, (double)init_ori.x, (double)init_ori.y, (double)init_ori.z);

	for (int i = 0; i < 1000; i++) {
		vehicle_.Update(env, 0.0f, 0.0f, 0.0f, 0.01f);
	}

	glm::vec3 tire_pos = vehicle_.GetTirePosition(0);
	glm::vec3 veh_pos = vehicle_.GetPosition();
	tire_offset_ = veh_pos.z - tire_pos.z;
	veh_id_ = vehicle_.GetVehicleIdNum();
	mass_ = vehicle_.GetChassis()->GetMass();
	max_lift_ = 9.806f * 3.0f * mass_;
}

void MavsHelicopter::CheckTireContact(environment::Environment* env) {
	use_rp3d_ = false;
	glm::vec3 veh_pos = vehicle_.GetPosition();
	float vz = veh_pos.z;
	float gh = env->GetGroundHeight(veh_pos.x, veh_pos.y);
	float radius = vehicle_.GetTire(0)->GetRadius();
	if (vz - tire_offset_ <= gh + radius) use_rp3d_ = true;
}

void MavsHelicopter::Update(environment::Environment* env, float throttle, float steer, float brake, float dt) {
	CheckTireContact(env);
	
	if (use_rp3d_) {
		vehicle_.SetGravity(g_);
		vehicle_.SetExternalForceOnCg(current_lift_);
		vehicle_.Update(env, throttle, steer, brake, dt);
	}
	else { // use the helicopter kinematic model to update
		VehicleState currstate = vehicle_.GetState();
		if (hover_) {
			current_lift_.z = g_ * mass_;
			currstate.twist.linear.z *= 0.99f;
		}
		currstate.twist.linear.z += dt*( (current_lift_.z/ mass_) - g_);
		
		double pitch, yaw, roll;
		math::QuatToEulerAngle(currstate.pose.quaternion, pitch, roll, yaw);
		yaw += 0.0025 * steer;
		pitch *= 0.99f;
		currstate.pose.quaternion = math::EulerToQuat(pitch, roll, yaw);
		double vmag = sqrt(currstate.twist.linear.x * currstate.twist.linear.x + currstate.twist.linear.y * currstate.twist.linear.y);
		vmag += 0.1 * throttle;
		vmag -= 0.1 * brake;
		vmag = std::max(0.0, vmag);

		currstate.twist.linear.x = vmag * cos(yaw);
		currstate.twist.linear.y = vmag * sin(yaw);

		currstate.pose.position += (double)dt * currstate.twist.linear;
		env->SetActorPosition(veh_id_, currstate.pose.position, currstate.pose.quaternion, true);
		env->SetActorVelocity(veh_id_, currstate.twist.linear);
		vehicle_.SetState(currstate);
	}
}

void MavsHelicopter::IncreaseLift(float dt) {
	current_lift_.z += (max_lift_ / 10.0f) * dt;
	current_lift_.z = std::min(current_lift_.z, max_lift_);
	hover_ = false;
}

void MavsHelicopter::DecreaseLift(float dt) {
	current_lift_.z -= (max_lift_ / 10.0f) * dt;
	current_lift_.z = std::max(current_lift_.z, 0.0f);
	hover_ = false;
}

} // namespace vehicle
} // namespace mavs
