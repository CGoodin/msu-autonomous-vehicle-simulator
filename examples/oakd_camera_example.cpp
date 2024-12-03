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
* \file camera_example.cpp
*
* An example MAVS Oak-D depth camera sensor
*
* Usage: >./oakd_camera_example scene.json
*
* scene.json examples found in mavs/data/scenes
*
* \author Chris Goodin
*
* \date 9/11/2024
*/
#include <sensors/camera/oak_d_camera.h>
#include <iostream>
#include <raytracers/embree_tracer/embree_tracer.h>

int main(int argc, char* argv[]) {
	if (argc < 2)std::cerr << "Usage: ./camera_example scenefile.json" << std::endl;

	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);

	mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	mavs::sensor::camera::OakDCamera camera;
	camera.SetMaxDepth(4.0f);
	camera.FreePose();
	camera.SetDisplayType("both");

	camera.SetPose(glm::dvec3(0.0f, -2.25f, 1.25f), glm::quat(0.70710678f, 0.0f, 0.0f, 0.70710678f));
	camera.Update(&env, 0.1);
	camera.Display();

	while (camera.DisplayOpen()) {
		camera.Update(&env, 0.1);
		mavs::Pose pose = camera.GetPose();
		//float depth_at_center = camera.GetCmDepthAtPixel(camera.GetDepthImageWidth() / 2, camera.GetDepthImageHeight() / 2);
		camera.Display();
	}
	return 0;
}

