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
* \file utest_mavs_camera.cpp
*
* Unit test to evaluate the mavs camera model
*
* Usage: >./utest_mavs_camera
*
* Correct out put is a camera frame, rotating around a 
* simple scene with a red sphere, a yellow box, and a 
* green surface.
*
* Ctrl+C to stop the simulation
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <iostream>

#include <raytracers/simple_tracer/simple_tracer.h>
#include "raytracers/material.h"
#include <sensors/camera/rgb_camera.h>
#include "mavs_core/environment/particle_system/particle_system.h"

#ifdef USE_OMP
#include <omp.h>
#endif

int main (int argc, char *argv[]){
  int myid = 0;
  int numprocs = 1;
#ifdef USE_MPI
  int ierr = MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD,&numprocs);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif
  
  mavs::sensor::camera::RgbCamera camera;
#ifdef USE_MPI  
  camera.SetComm(MPI_COMM_WORLD);
#endif
	mavs::raytracer::SimpleTracer scene;
	scene.CreateTestScene();

  if (argc>1){
    int pix = atoi(argv[1]);
    camera.Initialize(pix,pix,0.0035f, 0.0035f, 0.0035f);
		camera.SetElectronics(1.0f, 1.0f);
		camera.SetAntiAliasing("simple");
  }

  mavs::environment::Environment env;


  //glm::vec3 light_pos(0,0,13);
  //glm::vec3 light_col(50,50,25);
  //glm::vec3 light_direction(0,0,-1);
  //float spot_angle = 2.0;
  //env.AddSpotlight(light_col,light_pos,light_direction,spot_angle);

  env.SetRaytracer(&scene);
	camera.SetEnvironmentProperties(&env);

  //add a particle system to the scene
  mavs::environment::ParticleSystem smoke_psys;
  smoke_psys.SetLifetime(3.0f);
  smoke_psys.SetInitialVelocity(0.0f, 0.0f, 5.0f);
  smoke_psys.SetGravityFactor(0.1f);
  smoke_psys.SetExpansionRate(0.5f);
  smoke_psys.SetInitialRadius(0.5f);
  smoke_psys.SetTransparency(0.5f);
  smoke_psys.SetVelocityRandomization(1.5f, 1.5f, 0.5f);
  smoke_psys.SetInitialColor(0.75f,0.75f,0.75f);
  glm::vec3 smoke_center(1.0f,-2.0f,10.0f);
  float smoke_radius = 3.0f;
  float smoke_rate = 100.0f;
  smoke_psys.SetSource(smoke_center,smoke_radius,smoke_rate);
  env.AddParticleSystem(smoke_psys);
	

  double range = -25.0;
  double height = 10.0;
  glm::dvec3 position(range, 0.0, height);
  glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
  float time = 0.0f;
  float dt = 0.1f;
  while(true){
#ifdef USE_MPI
    double t1 = MPI_Wtime();
#elif USE_OMP
    double t1 = omp_get_wtime();
#endif
    env.AdvanceTime(dt);
    //camera.SetEnvironmentProperties(&env);
    position = glm::dvec3(range*cos(time), range*sin(time),height);
    double angle = 0.5*time;
    orientation = glm::dquat(cos(angle),0.0,0.0,sin(angle));
    camera.SetPose(position, orientation);
    camera.Update(&env,dt);
    time += dt;
    if(myid==0){
#ifdef USE_MPI    
      std::cout<<"Render time = "<<MPI_Wtime()-t1<<std::endl;
#elif USE_OMP
      std::cout<<"Render time = "<<omp_get_wtime()-t1<<std::endl;
#endif    
      camera.Display();
    }
  }
  if (myid==0)
    camera.SaveImage("camera_utest_output.bmp");
#ifdef USE_MPI
  MPI_Finalize();
#endif
  return 0;
}

