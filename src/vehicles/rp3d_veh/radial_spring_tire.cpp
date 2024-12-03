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
#include <vehicles/rp3d_veh/radial_spring_tire.h>

namespace mavs {
namespace vehicle {
namespace radial_spring {

Tire::Tire() {
	orientation_[0][0] = 1.0f;
	orientation_[0][1] = 0.0f;
	orientation_[0][2] = 0.0f;
	orientation_[1][0] = 0.0f;
	orientation_[1][1] = 1.0f;
	orientation_[1][2] = 0.0f;
	orientation_[2][0] = 0.0f;
	orientation_[2][1] = 0.0f;
	orientation_[2][2] = 1.0f;
	tire_position_ = glm::vec3(0.0f, 0.0f, 1.0f);
	initialized_ = false;
}

void Tire::Initialize(float sw, float ud_r, float k, float dtheta_degrees, int num_slices) {
	section_width_ = sw; // 0.25f;
	undeflected_radius_ = ud_r; // 0.254f; // 10 inches converted to meters
	spring_constant_ = k; // 68947.6f; // 10 PSI converted to pascals

	nsprings_ = (int)ceil(360.0f / dtheta_degrees);
	dtheta_ = (float)(2.0f*mavs::kPi / (float)nsprings_);
	nslices_ = num_slices;
	float slice_size = section_width_ / (num_slices + 1.0f);
	slices_.resize(num_slices);
	for (int j = 0; j<num_slices; j++) {
		slices_[j].offset = (j + 1)*slice_size - 0.5f*section_width_;
		slices_[j].springs.resize(nsprings_);
		// set the angle for each spring
		for (int i = 0; i<slices_[j].springs.size(); i++) {
			slices_[j].springs[i].SetTheta(i*dtheta_);
		}
	}
	dslice_ = section_width_ / (1.0f*num_slices);
	initialized_ = true;
}

float Tire::GetNormalForce(environment::Environment *env) {
	// loop through and put the slices in the right position
	glm::vec3 look_side = orientation_[1];
	for (int j = 0; j<slices_.size(); j++) {
		slices_[j].position = tire_position_ + slices_[j].offset*look_side;
	}

	// in this loop, calculate the deflections at each sample point
	// this is different that Equation 1 of DAVIS because it assumes we'll have raytracing
	float ncontact = 0.0f;
#pragma omp parallel for reduction(+: ncontact) schedule(dynamic)
	for (int j = 0; j<nslices_; j++) {
		for (int i = 0; i<nsprings_; i++) {
			slices_[j].springs[i].SetDeflection(0.0f);
			glm::vec3 dir = orientation_ * slices_[j].springs[i].GetDirection();
			if (dir.z<0.0f) {
				glm::vec3 pos = slices_[j].position;
				raytracer::Intersection inter = env->GetClosestTerrainIntersection(pos, dir);
				float r = inter.dist;
				if (r > 0.0f && r < undeflected_radius_) {
					ncontact += 1.0f;
					slices_[j].springs[i].SetDeflection(undeflected_radius_ - r);
				}
			}
			slices_[j].springs[i].SetPosition(slices_[j].position + slices_[j].springs[i].GetDirection()*(undeflected_radius_ - slices_[j].springs[i].GetDeflection()));
		}
	}
	// check if tire is not touching
	if (ncontact == 0)return 0.0f;
	// DAVIS, Equations 2, 4, and 5
	float theta_contact = ncontact * dtheta_ / nslices_;
	float d_max = undeflected_radius_ * (1.0f - cos(0.5f*theta_contact));
	float a_s = 0.5f*undeflected_radius_*undeflected_radius_*(theta_contact - sinf(theta_contact));
	float v_s = section_width_ * a_s;

	// in the this loop, calculate the deflected volume
	float v_t = 0.0f;
	//float a_t = 0.0f;
#pragma omp parallel for reduction(+: v_t) schedule(dynamic)
	for (int j = 0; j<slices_.size(); j++) {
		for (int i = 0; i<nsprings_; i++) {
			float di = slices_[j].springs[i].GetDeflection();
			// DAVIS Equation 3
			float displaced_area = (undeflected_radius_*di - 0.5f*di*di)*dtheta_;
			//a_t += displaced_area;
			v_t += displaced_area * dslice_;
		}
	}
	//a_t = a_t / nslices_;

	// DAVIS Equation 6, equivalent deflection
	float d_e = d_max * v_t / v_s;
	//float d_e = d_max * a_t / a_s;
	current_equivalent_deflection_ = d_e;
	// DAVIS Equation 17
	float fn = spring_constant_ * d_e; // *d_e;
	return fn;
}


void Tire::PrintSprings() {
	for (int j = 0; j<slices_.size(); j++) {
		for (int i = 0; i<nsprings_; i++) {
			glm::vec3 p = slices_[j].springs[i].GetPosition();
			std::cout << j << " " << i << " " <<
				p.x << " " << p.y << " " << p.z << " " <<
				slices_[j].springs[i].GetDeflection() << std::endl;
		}
	}
}

} // namespace radial_spring
}// namespace vehicle
}// namespace mavs
