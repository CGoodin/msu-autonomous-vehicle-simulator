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
/**
* \file chamfer_distance_example.cpp
*
* Demonstrates calculation of chamfer distance between two point clouds
*
* \date 9/23/2024
*/
// c++ includes
#include <iostream>
// mavs includes
#include <sensors/lidar/os2.h>
#include <mavs_core/math/utils.h>
#include <raytracers/embree_tracer/embree_tracer.h>

int main(int argc, char *argv[]) {

	if (argc < 2) {
		std::cerr << "Usage: ./chamfer_distance_example scenefile.json " << std::endl;
		return 1;
	}
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	std::cout << "Loading " << scene_file << std::endl;
	scene.Load(scene_file);

	mavs::environment::Environment env;
	env.SetRaytracer(&scene);


	mavs::sensor::lidar::OusterOS2 lidar;

	double elapsed_time = 0.0;

	glm::dvec3 pos1(0.0, 0.0, 1.0);
	glm::dvec3 pos2(0.5, 0.5, 1.0);
	glm::dquat orientation(1.0, 0.0, 0.0, 0.0);

	std::cout << "Done loading, scanning" << std::endl;
	
	lidar.SetPose(pos1, orientation);
	lidar.Update(&env, 0.1);
	lidar.Display();
	std::vector<glm::vec3> pc1_0 =  lidar.GetRegisteredPoints();
	std::vector<glm::vec3> pc1;
	for (int i = 0; i < pc1_0.size(); i++) {
		if (!(pc1_0[i].x == 0.0f && pc1_0[i].y == 0.0f && pc1_0[i].z == 0.0f))pc1.push_back(pc1_0[i]);
	}

	lidar.SetPose(pos2, orientation);
	lidar.Update(&env, 0.1);
	lidar.Display();
	std::vector<glm::vec3> pc2_0 = lidar.GetRegisteredPoints();
	std::vector<glm::vec3> pc2;
	for (int i = 0; i < pc2_0.size(); i++) {
		if (!(pc2_0[i].x == 0.0f && pc2_0[i].y == 0.0f && pc2_0[i].z == 0.0f))pc2.push_back(pc2_0[i]);
	}

	std::cout << "Calculating chamfer distance...." << std::endl;
	double t0 = omp_get_wtime();
	float cd = mavs::math::ChamferDistance(pc1, pc2);
	double t1 = omp_get_wtime();
	std::cout << "Chamfer distance = " << cd << " " << (t1-t0) << std::endl;

	std::vector<glm::vec3> pca, pcb;
	pca.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
	pca.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
	pca.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	pca.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
	pca.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
	pca.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
	pcb.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	std::cout << "Chamfer distance " << mavs::math::ChamferDistance(pca, pcb) << std::endl;

	return 0;
}

