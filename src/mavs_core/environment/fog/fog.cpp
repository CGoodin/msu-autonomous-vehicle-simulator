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
#include <mavs_core/environment/fog/fog.h>
#include <time.h>
#include <iostream>

namespace mavs {
namespace environment {

Fog::Fog() {
	fog_spatial_lo_.SetFrequency(0.02); //Wavelength = 50.0
	fog_spatial_hi_.SetFrequency(0.2);
	int curr_time = (int)time(NULL);
	fog_spatial_lo_.SetSeed(curr_time);
	fog_spatial_hi_.SetSeed(curr_time+1);
	k_ = 0.0f;
}

float Fog::GetK(glm::vec3 &p0, glm::vec3 &p1) {
	float k = 0.0f;
	float t = 0.0f;
	float dt = 0.1f;
	while (t <= 1.0f) {
		float tp = 1.0f - t;
		glm::vec3 p = tp * p0 + t * p1;
		float f_lo = (float)fog_spatial_lo_.GetPerlin(p.x, p.y, p.z);
		if (f_lo > 0.0f) {
			k += ((float)(f_lo + 0.5f*(1.0f + fog_spatial_hi_.GetPerlin(p.x, p.y, p.z))))*exp(-0.1f*p.z);
		}
		t += dt;
	}
	k = k_*0.5f*k*dt;
	return k;
}

} // namespace environment
} // namespace mavs
