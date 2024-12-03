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
* \file utest_mavs_radar.cpp
*
* Unit test to evaluate the mavs radar
*
* Usage: >./utest_mavs_radar
*
* If working correctly, should show a simple scene
* with several columns. A camera and a radar will rotate
* through the scene, and the radar returns will show up in yellow.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <stdlib.h>
#include <time.h> 

#include <iostream>
#include <fstream>

#include <raytracers/simple_tracer/simple_tracer.h>
#include <sensors/radar/radar.h>
#include <sensors/camera/rgb_camera.h>
#include <mavs_core/math/utils.h>

#ifdef USE_OMP
#include <omp.h>
#endif

int main(int argc, char *argv[]) {
	srand((unsigned int)time(NULL));
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI 
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	mavs::raytracer::SimpleTracer scene;

	mavs::raytracer::Aabb ground;
	ground.SetSize(1.0E6, 1.0E6, 0.01);
	ground.SetColor(0.25, 1.0, 0.25);
	ground.SetPosition(0, 0, 5.0);
	scene.AddPrimitive(ground);
	float range = 60.0f;
	//int nboxes = mavs::math::rand_in_range(3,10);
	int nboxes = 50;
	std::vector<mavs::raytracer::Aabb> box;
	box.resize(nboxes);
	std::vector<glm::vec2> box_locations;
	box_locations.resize(nboxes);
	std::ofstream fout("target_truth.txt");
	for (int i = 0; i < nboxes; i++) {
		float x = mavs::math::rand_in_range(-range, range);
		float y = mavs::math::rand_in_range(-range, range);
		double sx = mavs::math::rand_in_range(0.5, 1.0);
		double sy = mavs::math::rand_in_range(0.5, 1.0);
		box_locations[i].x = x;
		box_locations[i].y = y;
		fout << i << " " << x << " " << y << " " << sx << " " << sy << std::endl;
		box[i].SetPosition(x, y, 2.0f);
		box[i].SetSize(sx, sy, 4.0f);
		float theta = (float)atan2(y, x);
		float red = (float)cos(0.25*theta);
		float blue = 1.0f - red;
		float green = 1.0f;
		box[i].SetColor(red,green,blue);
		//box[i].SetColor(0.7,0.7,0.25);
		scene.AddPrimitive(box[i]);
	}
	fout.close();
	mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	glm::dvec3 position(0.0f, 0.0f, 2.0f);
	glm::dquat orientation(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::sensor::radar::Radar radar;
	mavs::sensor::camera::RgbCamera camera;
	camera.Initialize(512, 512, 0.0035f, 0.0035f, 0.00175f);
#ifdef USE_MPI  
	radar.SetComm(MPI_COMM_WORLD);
#endif
	float dt = 0.1f;
	float time = 0.0f;
	//while (true) {
	while (time <= 10.0f){
#ifdef USE_MPI  
		double t1 = MPI_Wtime();
#elif USE_OMP
		double t1 = omp_get_wtime();
#endif  
		env.AdvanceTime(dt);
		orientation = glm::dquat(cos(0.5*time), 0.0, 0.0, sin(0.5*time));
		camera.SetPose(position, orientation);
		camera.Update(&env, dt);
		radar.SetPose(position, orientation);
		radar.Update(&env, dt);
		if (myid == 0) {
#ifdef USE_MPI    
			std::cout << "Scan time = " << MPI_Wtime() - t1 << std::endl;
#elif USE_OMP
			std::cout << "Scan time = " << omp_get_wtime() - t1 << std::endl;
#endif
		}

		if (myid == 0) {
			//radar.WriteObjectsToText("radar_detections.txt");
			//radar.WriteLobeToText("radar_lobe.txt");
			radar.Display();
			//radar.SaveImage("radar_display.bmp");
			camera.Display();
		}

		time += dt;
	}
#ifdef USE_MPI  
  MPI_Finalize();
#endif
  
  return 0;
}

