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
#include "vti/nicolas_comstock.h"
#include <math.h>
#include <algorithm>

/** Based on the papers :
* Brach, R., & Brach, M. (2011). 
* The tire-force ellipse (friction ellipse) and tire characteristics 
* (No. 2011-01-0094). SAE Technical Paper.
*
* Brach, R. M., & Brach, R. M. (2009). 
* Tire models for vehicle dynamic simulation and accident reconstruction 
* (No. 2009-01-0102). SAE Technical Paper.
*/

namespace mavs {
namespace vti {

NicolasComstock::NicolasComstock() {
	C_a_ = 66723.0; //Newtons/radian
	C_s_ = 44482.0; //Newtons
	C_a_2_ = C_a_ * C_a_;
	C_s_2_ = C_s_ * C_s_;
}

TireForces NicolasComstock::GetForces(double pure_longi, double pure_lat, double s, double a) {
	//s = slip (-1 to 1)
	//a = steering angle (radians)
	//Equations 3 and 4
	TireForces forces;
	double s2 = s * s;
	double ta = tan(fabs(a));
	double sa = sin(fabs(a));
	double sa2 = sa * sa;
	double ca = cos(a);
	double ca2 = ca * ca;
	double fx = pure_longi;
	double fy = pure_lat;
	double fx2 = fx * fx;
	double fy2 = fy * fy;
	double ta2 = ta * ta;
	double denom = sqrt(s2*fy2 + fx2 * ta2);
	double fac = pow(1.0 - fabs(s), 2);
	forces.longitudinal = (fx*fabs(fy*s) / denom)*(sqrt(s2*C_a_2_ + fac * ca2*a*fx2) / (fabs(s)*C_a_));
	forces.lateral = (fy*fabs(fx*ta) / denom)*(sqrt(fac*ca2*fy2 + sa2 * C_s_2_) / (C_s_*fabs(sa)));
	return forces;
}

} //namespace vti
} //namespace mavs