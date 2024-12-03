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
* \file actor_example.cpp
*
* An example of an actor (dynamic object) in MAVS
*
* Usage: >./actor_example scene.json actor.json
*
* scene.json examples found in mavs/data/scenes
*
* actor.json examples found in mavs/data/actors
*
* \author Chris Goodin
*
* \date 10/4/2018
*/

#include <sensors/camera/rgb_camera.h>
#include <sensors/camera/simple_camera.h>

#include <iostream>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

int main(int argc, char *argv[]) {
#ifdef USE_EMBREE
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI  
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	mavs::sensor::camera::RgbCamera camera;
	mavs::sensor::camera::SimpleCamera free_camera;
	free_camera.FreePose();

#ifdef USE_MPI  
	camera.SetComm(MPI_COMM_WORLD);
#endif
	mavs::environment::Environment env;


	if (argc<3) {
		std::cerr << "Usage: ./actor_example scenefile.json actorfile.json" 
			<< std::endl;
		return 1;
	}
	std::string scene_file(argv[1]);
	std::string actor_file(argv[2]);
	mavs::raytracer::embree::EmbreeTracer scene;
	if (myid == 0)std::cout << "Loading " << scene_file << std::endl;
	scene.Load(scene_file);
	//std::vector<int> actors = scene.AddActors(actor_file);

	camera.Initialize(512, 512, 0.0035f, 0.0035f, 0.0035f);
	env.SetRaytracer(&scene);
	env.LoadActors(actor_file);
	camera.SetEnvironmentProperties(&env);
	glm::dvec3 position(0.0, 0.0, 1.0);
	glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
	free_camera.SetPose(position, orientation);
#ifdef USE_MPI
	MPI_Barrier(MPI_COMM_WORLD);
#endif

	if (myid == 0)std::cout << "Done loading " << scene.GetNumberTrianglesLoaded() <<
		" triangles, rendering" << std::endl;

	float time = 0.0f;
	float dt = 0.1f;
	//for (int i = 0; i<25; i++) {
	//while (time<12.0){
	while(true) {
#ifdef USE_MPI    
		double t1 = MPI_Wtime();
#endif    
		env.AdvanceTime(dt);
		glm::vec3 actvec = env.GetActorPosition(0);
		glm::vec2 tovec(actvec.x - position.x,actvec.y-position.y);
		double lt_yaw = atan2(tovec.y, tovec.x);
		orientation = glm::dquat(cos(0.5*lt_yaw), 0.0, 0.0, sin(0.5*lt_yaw));
		camera.SetPose(position, orientation);
		camera.Update(&env, dt);
		free_camera.Update(&env, dt);
		if (myid == 0) {
#ifdef USE_MPI
			std::cout << "FPS = " << 1.0 / (MPI_Wtime() - t1) << std::endl;
#endif
			camera.Display();
			free_camera.Display();
		}
		time += dt;
	}

	//if (myid == 0) {
	//	camera.Display();
	//	camera.SaveImage("output.bmp");
	//}
#ifdef USE_MPI
	MPI_Finalize();
#endif

#endif
	return 0;
}


