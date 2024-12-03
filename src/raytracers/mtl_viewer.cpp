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
#include <raytracers/mtl_viewer.h>
#include <raytracers/mesh.h>
#include <mavs_core/math/utils.h>
#include <iostream>
//#include "raytracers/material.h"

namespace mavs {
namespace raytracer {

MtlViewer::MtlViewer() {
	camera.Initialize(384, 384, 0.0035f, 0.0035f, 0.0035f);
	camera.SetElectronics(1.0f, 1.0f);

	//double range = -10.0;
	//double height = 0.0;
	double range = -20.0;
	double height = 10.0;

	glm::dvec3 position(range, 0.0, height);
	glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
	float time = 0.0f;
	float dt = 0.1f;
	camera.SetPose(position, orientation);

	mavs::Material shiny_red;
	shiny_red.kd = glm::vec3(0.7, 0.15, 0.15);
	shiny_red.ks = glm::vec3(0.3, 0.3, 0.3);
	shiny_red.ns = 30.0;
	mavs::raytracer::Sphere sred;
	sred.SetPosition(0.0f, 0.0f, 10.0f);
	sred.SetMaterial(shiny_red);
	sred.SetRadius(5.0f);
	prim_num_ = scene.AddPrimitive(sred);
	mavs::raytracer::Aabb box;
	box.SetSize(1.0E6f, 1.0E6f, 0.01f);
	box.SetColor(0.757f, 0.604f, 0.420f);
	box.SetPosition(0.0f, 0.0f, 5.0f);
	scene.AddPrimitive(box);

	env.SetRaytracer(&scene);
	camera.SetEnvironmentProperties(&env);
}

void MtlViewer::LoadMesh(std::string meshfile) {
	std::string path = mavs::utils::GetPathFromFile(meshfile);
	mesh_.Load((path+'/'), meshfile);
	for (int i = 0; i < mesh_.GetNumMaterials(); i++) {
		avail_mats_.push_back(*mesh_.GetMaterial(i));
	}
}

int MtlViewer::GetNumMats() {
	return (int)avail_mats_.size();
}

char* MtlViewer::GetMatName(int id) {
	std::string name = "";
	if (id >= 0 && id < avail_mats_.size()) {
		name = avail_mats_[id].name;
	}
	char* p = new char[name.length() + 1];
	strcpy(p, name.c_str());
	return p;
}

char* MtlViewer::GetSpectrumName(int id) {
	std::string name = "";
	if (id >= 0 && id < avail_mats_.size()) {
		name = avail_mats_[id].refl;
	}
	char* p = new char[name.length() + 1];
	strcpy(p, name.c_str());
	return p;
}

Material MtlViewer::GetMaterial(int id) {
	Material mat;
	if (id >= 0 && id < avail_mats_.size()) {
		mat = avail_mats_[id];
	}
	return mat;
}

void MtlViewer::Update() {
	camera.Update(&env, 0.03);
	camera.Display();
}

void MtlViewer::SetMaterial(Material material) {
	material_ = material;
	scene.SetPrimitiveMaterial(prim_num_, material_);
}

} // namespace raytracer
} //namespace mavs

