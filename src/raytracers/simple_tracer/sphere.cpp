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
#include <iostream>

#include <raytracers/simple_tracer/sphere.h>

namespace mavs{
namespace raytracer{

Sphere::Sphere(){
  radius_ = 1.0f;
  r2_ = 1.0f;
}

Sphere::~Sphere(){

}

static bool SolveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1){
  float discr = b*b - 4.0f*a*c;
  if (discr<0) return false;
  else if (discr==0.0f) x0 = x1 = -0.5f*b/a;
  else {
    float q = (b>0.0f)?
      -0.5f*(float)(b+sqrt(discr)) :
      -0.5f*(float)(b-sqrt(discr));
    x0 = q/a;
    x1 = c/q;
  }
  if (x0>x1) std::swap(x0,x1);
  return true;
}

Intersection Sphere::GetIntersection(glm::vec3 orig, glm::vec3 dir){
  Intersection inter;
  inter.dist = -1.0;
  float t0,t1;
  glm::vec3 L = orig - position_;
  float a = glm::dot(dir,dir);
  float b = 2*glm::dot(dir,L);
  float c = glm::dot(L,L)-r2_;
  if(!SolveQuadratic(a,b,c,t0,t1)) return inter;
  if(t0>t1) std::swap(t0,t1);
  if (t0<0){
    t0 = t1;
    if (t0<0) return inter;
  }
  inter.dist = t0;
  inter.normal = (orig + t0*dir)-position_;
  inter.normal = inter.normal/length(inter.normal);
	inter.material.kd = color_;
	//inter.material.ns = ns_;
	inter.color = color_;
	inter.object_name = "sphere";
	inter.material = material_;
  return inter;
}

} //namespace raytracer
} //namespace mavs
