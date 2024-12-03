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
* \file utest_mavs_snow.cpp
*
* Unit test to evaluate the mavs snow renderer
*
* Usage: >./utest_mavs_snow
*
* \author Chris Goodin
*
* \date 6/7/2019
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <mavs_core/environment/environment.h>
#include <raytracers/simple_tracer/simple_tracer.h>
#include <sensors/camera/rgb_camera.h>
#include <iostream>

int main(int argc, char *argv[]){
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	float snow_rate = 10.0f;
	if (argc > 1)snow_rate = (float)atof(argv[1]);

	mavs::environment::Environment env;
	env.SetTemperature(-10.0f);
	env.SetSnowRate(snow_rate);
	env.SetCloudCover(0.65f);
	env.SetTurbidity(8.0f);
	env.SetSnowAccumulation(1.0f);

	mavs::raytracer::SimpleTracer scene;
	scene.CreateTestScene();
	//scene.CreateForest();

	env.SetRaytracer(&scene);

	mavs::sensor::camera::RgbCamera cam;

	cam.SetEnvironmentProperties(&env);

	//cam.SetPose(glm::vec3(-35.0f, 0.0f, 1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	cam.SetPose(glm::vec3(-35.0f, 0.0f, 10.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

	float dt = 0.03f;
	while (true) {
		env.AdvanceTime(dt);
		cam.Update(&env, dt);
		if (myid==0)cam.Display();
	}
	//cam.SaveImage("snowing.bmp");
#ifdef USE_MPI
	MPI_Finalize();
#endif
  return 0;
}
