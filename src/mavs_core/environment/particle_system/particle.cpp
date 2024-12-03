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
#include <mavs_core/environment/particle_system/particle.h>

#include <algorithm>
#include <iostream>

namespace mavs{
namespace environment{

Particle::Particle(){
  color_ = glm::vec3(1.0,1.0,1.0);
  transparency_ = 0.75;
  age_ = 0.0;
  radius_ = 0.1f;
}

static bool SolveQuadratic(const float &a, const float &b, const float &c,
			   float &x0, float &x1){
  float discr = b*b - 4*a*c;
  if (discr<0) return false;
  else if (discr==0) x0 = x1 = -0.5f*b/a;
  else {
    float q = (b>0.0f)?
      -0.5f*(b+sqrt(discr)) :
      -0.5f*(b-sqrt(discr));
    x0 = q/a;
    x1 = c/q;
  }
  if (x0>x1) std::swap(x0,x1);
  return true;
}

glm::vec2 Particle::GetIntersection(glm::vec3 orig, glm::vec3 dir){

	glm::vec2 inter;
	inter.y = -1.0;
	
	glm::vec3 L = orig - position_;
	float Ldot = glm::dot(L, dir);
	if (Ldot > 0.0) return inter;
  float r2_ = radius_*radius_;
  float t0,t1;
  glm::vec3 v = L-Ldot*dir;
  inter.x = glm::length(v)/radius_;
  float a = glm::dot(dir,dir);
  float b = 2*Ldot;
  float c = glm::dot(L,L)-r2_;
  if(!SolveQuadratic(a,b,c,t0,t1)) return inter;
  if(t0>t1) std::swap(t0,t1);
  if (t0<0){
    t0 = t1;
    if (t0<0) return inter;
  }
  inter.y = t0;
	//std::cout << Ldot << std::endl;
  return inter;
}

} // namespace environemnt
} // namespace mavs

