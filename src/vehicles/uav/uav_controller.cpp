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
// project includes
#include "vehicles/uav/uav_controller.h"
// c++ includes
#include <iostream>
#include <algorithm>

namespace mavs {
namespace vehicle {
UavController::UavController() {
	speed_pid_.SetKp(2.0);
	speed_pid_.SetKi(0.1);
	speed_pid_.SetKd(0.1);
	speed_pid_.SetSetpoint(20.0f);
	alt_pid_.SetKp(0.01);
	alt_pid_.SetKi(0.0);
	alt_pid_.SetKd(0.0);
	alt_pid_.SetSetpoint(250.0f);
	roll_control_.TurnOnLooping();
	roll_control_.SetMaxLookAhead(80.0f);
	roll_control_.SetMinLookAhead(10.0f);
	roll_control_.SetSteeringParam(10.0f);
	roll_control_.SetWheelbase(25.0f);
	roll_control_.SetGoalThreshhold(50.0f);
}

FlightControl UavController::UpdateControl(Uav* uav, float dt) {
	FlightControl control;
	control.throttle = (float)speed_pid_.GetControlVariable(uav->GetAirspeed(), dt);
	control.pitch = -(float)alt_pid_.GetControlVariable(uav->GetAltitude(), dt);

	roll_control_.SetVehiclePosition(uav->GetPosition().x, uav->GetPosition().y);
	roll_control_.SetVehicleOrientation(uav->GetHeadingRadians());
	roll_control_.SetVehicleSpeed(0.0f);
	float thrott, steer, brake;
	roll_control_.GetDrivingCommand(thrott, steer, brake, dt);
	control.roll = std::max(-max_roll_, std::min(max_roll_, -10.0f * steer));
	return control;
}

void UavController::SetSpeedControllerParams(float p, float i, float d) {
	speed_pid_.SetKp(p);
	speed_pid_.SetKi(i);
	speed_pid_.SetKd(d);
}

void UavController::SetAltitudeControllerParams(float p, float i, float d) {
	alt_pid_.SetKp(p);
	alt_pid_.SetKi(i);
	alt_pid_.SetKd(d);
}
} // namespace vehicle
} //namespace mavs
