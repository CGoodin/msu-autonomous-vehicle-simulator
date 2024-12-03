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
#include "vti/spring_tire.h"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace mavs {
namespace vti {

SpringTire::SpringTire() {
	//SetLoadDeflection(1000.0, 0.05);
	spring_constant_ = 27500.0;//275000.0;
	damping_coefficient_ = 16000.0;//160000.0;
	current_velocity_ = 0.0;
	current_compression_ = 0.0;
}

void SpringTire::SetLoadDeflection(double load, double deflection) {
	spring_constant_ = (load / deflection);
	SetCriticalDamping(load*0.10197);
}

void SpringTire::SetSpringDamping(double k, double c) {
	spring_constant_ = k;
	damping_coefficient_ = c;
}

void SpringTire::SetCriticalDamping(double mass) {
	damping_coefficient_ = 2.0*sqrt(spring_constant_*mass);
}

double SpringTire::Update(double dt, double x) {
	x = std::min(std::max(0.0, x), section_height_);
	double dx = x - current_compression_;
	//double compression = std::max(0.0, radius_ - x);
	//std::cout << "forces " << x << " " << spring_constant_*x << std::endl;
	//double force = -damping_coefficient_ * (dx / dt) + spring_constant_ * x;
	double force =  spring_constant_ * x;
	force = std::max(0.0, force);
	current_compression_ = x;
	return force;
}

} //namespace vti
} //namespace mavs
