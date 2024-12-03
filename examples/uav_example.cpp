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
* \file uav_example.cpp
*
* Fly a MAVS uav 
*
* Usage: >./uav_example mavs_uav_file.json
*
* mavs_scene_file.json is a MAVS UAV file, examples of which can
* be found in mavs/data/vehicles/uav.
*
* \author Chris Goodin
*
* \date 11/18/24
*/
//#include <iostream>
//#include <stdlib.h>
//#include <mavs_core/math/utils.h>
#include "vehicles/uav/uav_sim.h"
//#include <sensors/mavs_sensors.h>
#ifdef USE_OMP
#include <omp.h>
#endif
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

int main(int argc, char *argv[]) {
#ifndef USE_EMBREE
	std::cerr << "ERROR, MUST HAVE EMBREE ENABLED TO RUN THIS EXAMPLE! " << std::endl;
	exit(12);
#endif
	if (argc < 2) {
		std::cerr << "ERROR, must provide UAV input file as argument" << std::endl;
		return 1;
	}
	std::string uav_input_file(argv[1]);

	mavs::vehicle::UavSim uav_sim;
	mavs::raytracer::embree::EmbreeTracer scene;
	mavs::environment::Environment env;
	uav_sim.LoadSimulation(uav_input_file, &scene, &env);
	uav_sim.LoadAnimations(&scene, &env);
	uav_sim.SetRenderDebug(true);
	uav_sim.SetControllerActive(false); // fly manually
	
	float dt = 0.01f;
	while (uav_sim.IsActive()) {
		double t0 = omp_get_wtime();
		uav_sim.Update(&env, dt);
		while ((omp_get_wtime() - t0) < dt) {}
	}

	return 0;
}
