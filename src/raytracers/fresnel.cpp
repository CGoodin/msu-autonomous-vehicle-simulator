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

#include <raytracers/fresnel.h>

namespace mavs{
namespace raytracer{

// The indices of refraction should be complex

float GetFresnelTransmissionAngle(float n1, float n2, float theta_i) {
	float theta_t = (float)(asin((n1/n2)*sin(theta_i)));
	return theta_t;
}

float GetFresnelReflectance(float n1, float n2, float theta_i, float theta_t) {
	glm::vec2 r = GetPolarizedFresnelReflectance(n1, n2, theta_i, theta_t);
	float refl = 0.5f*(r.x + r.y);
	return refl;
}

glm::vec2 GetPolarizedFresnelReflectance(float n1, float n2, float theta_i, float theta_t) {
	float ci = cos(theta_i);
	float ct = cos(theta_t);
	float n1ci = n1 * ci;
	float n2ct = n2 * ct;
	float n1ct = n1 * ct;
	float n2ci = n2 * ci;
	float rs = (float)pow((n1ci - n2ct) / (n1ci + n2ct), 2.0f);
	float rp = (float)pow((n1ct - n2ci) / (n1ct + n2ci), 2.0f);
	glm::vec2 r(rs, rp);
	return r;
}

glm::vec4 GetFresnelCoeffs(float n1, float n2, float theta_i, float theta_t) {
	glm::vec2 c_r = GetPolarizedFresnelReflectance(n1, n2, theta_i, theta_t);
	glm::vec4 coeffs(c_r.x, c_r.y, 1.0f - c_r.x, 1.0f - c_r.y);
	return coeffs;
}


} //namespace raytracer
} //namespace mavs
