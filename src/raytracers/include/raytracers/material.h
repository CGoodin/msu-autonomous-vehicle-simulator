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
#ifndef MATERIAL_H
#define MATERIAL_H

#include <string>

#include <glm/glm.hpp>

#include <mavs_core/math/constants.h>

namespace mavs{

/// Material structure based on wavefront .mtl file
struct Material{
Material() : ka(zero_vec3), kd(one_vec3), ks(zero_vec3), tr(zero_vec3),
    ke(zero_vec3), ns(2.0f), ni(-1.0f), dissolve(0.0f) {}
  /// Name of the material
  std::string name;
  /// Ambient RGB reflectance
  glm::vec3 ka;
  /// Diffuse RGB reflectance
  glm::vec3 kd;
  /// Specular RGB reflectance
  glm::vec3 ks;
  glm::vec3 tr;
  glm::vec3 ke;
  /// Specular reflecance exponent for the Phong model
  float ns;
	// index of refraction
  float ni;
  float dissolve;
  float illum;
  /// Diffuse reflectance texture map
  std::string map_kd;
  /// Ambient reflectance texture map
  std::string map_ka;
  /// Specular reflectance texture map
  std::string map_ks;
  /// Phong specular exponent texture map
  std::string map_ns;
  /// Normal texture map
  std::string map_bump;
  /// Alpha (transparency) texture map
  std::string map_d;
  /// Height texture map
  std::string disp;
  /// Name of spectrum file for the  material
  std::string refl;
	Material(const Material &mat){
		name = mat.name;
		ka = mat.ka;
		kd = mat.kd;
		ks = mat.ks;
		tr = mat.tr;
		ke = mat.ke;
		ns = mat.ns;
		ni = mat.ni;
		dissolve = mat.dissolve;
		illum = mat.illum;
		map_kd = mat.map_kd;
		map_ka = mat.map_ka;
		map_ks = mat.map_ks;
		map_ns = mat.map_ns;
		map_bump = mat.map_bump;
		map_d = mat.map_d;
		disp = mat.disp;
		refl = mat.refl;
	}
};

} // namespace mavs

#endif
