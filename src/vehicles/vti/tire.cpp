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
#include "vti/tire.h"

namespace mavs {
namespace vti {

Tire::Tire() {

}

void Tire::Update(double dt, double z, double h,  double slip, double slip_angle) {
	double r = z - h;
	if (r > 0.0 && r < radius_) {
		double deflection = radius_ - r;
		double normal_force = spring_tire_.Update(dt, r);
		vti_.Update(100.0, normal_force, deflection, slip, slip_angle);
		forces_.normal = normal_force;
		forces_.longitudinal = vti_.GetLongitudinalTraction();
		forces_.lateral = vti_.GetLateralTraction();
	}
	else {
		double normal_force = spring_tire_.Update(dt, 0.0);
		forces_.normal = 0.0;
		forces_.longitudinal = 0.0;
		forces_.lateral = 0.0;
	}
}

} //namespace vti
} //namespace mavs

