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
* \file sky_example.cpp
*
* Renders the day and night sky over 24 hours using the fisheye camera model
*
* Usage: >./sky_example
*
* \author Chris Goodin
*
* \date 10/15/2018
*/

#include <iostream>
#include <sensors/mavs_sensors.h>
#include <mavs_core/math/utils.h>
#include <raytracers/simple_tracer/simple_tracer.h>

int main (int argc, char *argv[]){
  
  mavs::sensor::camera::FisheyeCamera camera;
	camera.Initialize(1024, 1024, 0.025f, 0.025f, 0.008f);
	camera.SetPixelSampleFactor(3);
  mavs::environment::Environment env;
	double origin_lat = 32.0;
	double origin_lon = 90.0;
	double elevation = 73.152;
	env.SetLocalOrigin(origin_lat, origin_lon, elevation); //Vicksburg, MS
	int hour = 0;
	int minute = 0;
	env.SetDateTime(2004, 6, 5, hour, minute, 0, 6);

  mavs::raytracer::SimpleTracer scene;
	mavs::raytracer::Aabb ground; 
  ground.SetSize(1.0E6f, 1.0E6f, 0.01f);
  ground.SetColor(0.25f,1.0f,0.25f);
  ground.SetPosition(0.0f,0.0f,0.0f);
  scene.AddPrimitive(ground);

  env.SetRaytracer(&scene);
  camera.SetEnvironmentProperties(&env);	
  glm::vec3 position(0.0f, 0.0f, 1.0f);
  glm::quat orientation(0.707107f, 0.0f, -0.707107f, 0.0f);
	camera.SetPose(position, orientation);

	int elapsed_time = 0;
	int secs_per_day = 86400;
	int minstep = 3;
	int dt = minstep*60;
	while (elapsed_time<secs_per_day){
    camera.Update(&env,0.1);
    camera.Display();
		std::string fname = "sky_"+mavs::utils::ToString(hour,2)+mavs::utils::ToString(minute,2)+".bmp";
		env.SetDateTime(2004, 6, 5, hour, minute, 0, 6);
		camera.SetEnvironmentProperties(&env);
		minute = minute + minstep;
		if (minute >= 60) {
			hour = hour + 1;
			minute = 0;
		}
		elapsed_time += dt;
		camera.SaveImage(fname);
  }
  
  return 0;
}

