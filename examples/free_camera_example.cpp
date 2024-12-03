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
* \file free_camera_example.cpp
*
* Fly the camera around the scene with the W-A-S-D keys.
* Page Up & Page Down to move up and down.
* Arrow keys rotate the view (left,right,up,down)
* Home and End keys rotate the roll of the camera
* Close the view window to finish the program.
*
* Usage: >./free_camera_example mavs_scene_file.json 
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
* 
* \author Chris Goodin
*
* \date 8/16/2018
*/

#include <sensors/mavs_sensors.h>

#include <iostream>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#else
#include <raytracers/simple_tracer/simple_tracer.h>
#endif

int main(int argc, char *argv[]) {

	mavs::sensor::camera::RgbCamera camera;
	//mavs::sensor::camera::Phantom4Camera camera;
	//mavs::sensor::camera::Phantom4CameraPathTracedLowRes camera(100, 10, 0.55f);

	camera.FreePose();
	camera.SetRaindropsOnLens(false);
	mavs::environment::Environment env;

#ifdef USE_EMBREE
	if (argc < 2) {
		std::cerr << "Usage: ./free_camera_example scenefile.json envfile.json" << std::endl;
		return 1;
	}
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
#else
	mavs::raytracer::SimpleTracer scene;
	scene.CreateTestScene();
#endif

	if (argc > 2) {
		std::string env_file(argv[2]);
		env.Load(env_file);
	}

  env.SetRaytracer(&scene);
  glm::vec3 position(0.0f, 0.0f, 3.0f);
  glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);
	//glm::vec3 position(0.0f, 0.0f, 350.0f);
	//glm::quat orientation(0.7071f, 0.0f, 0.7071f, 0.0f);
	camera.SetPose(position, orientation);

	while (true){
		camera.Update(&env,0.1);
		camera.Display(); 
  }
  
  return 0;
}

