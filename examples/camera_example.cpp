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
* An example MAVS RGB camera sensor
*
* Usage: >./camera_example scene.json env.json camera.json
*
* scene.json examples found in mavs/data/scenes
* 
* env.json examples found in mavs/data/environments
* 
* camera.json examples found in mavs/data/sensors/cameras
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <sensors/camera/rgb_camera.h>

#include <iostream>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#else
#include <raytracers/simple_tracer/simple_tracer.h>
#endif

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
  //mavs::sensor::camera::SegmentCamera camera;	

#ifdef USE_MPI  
  camera.SetComm(MPI_COMM_WORLD);
#endif
  mavs::environment::Environment env;
  
#ifdef USE_EMBREE
  if (argc<4){
    std::cerr<<"Usage: ./camera_example scenefile.json envfile.json"<<
      " camera_file.json \n";
#ifdef USE_MPI
		MPI_Finalize();
#endif
    return 1;
  }
  std::string scene_file(argv[1]);
  std::string env_file(argv[2]);
  std::string cam_file(argv[3]);
  mavs::raytracer::embree::EmbreeTracer scene;
  if (myid==0)std::cout<<"Loading "<<scene_file<<std::endl;
  scene.Load(scene_file);
  env.Load(env_file);
  camera.Load(cam_file);
  scene.TurnOffLabeling();
  scene.TurnOffSpectral();
  scene.TurnOnSurfaceTextures();
#else
  mavs::raytracer::SimpleTracer scene;
  mavs::raytracer::Sphere sred;
  sred.SetPosition(0,0,10);
  sred.SetColor(1.0,0.25,0.25);
  sred.SetRadius(5.0);
  scene.AddPrimitive(sred);
  mavs::raytracer::Aabb box,box2;
  box.SetSize(1.0E6f, 1.0E6f, 0.01f);
  box.SetColor(0.25f,1.0f,0.25f);
  box.SetPosition(0.0f,0.0f,5.0f);
  scene.AddPrimitive(box);
  box2.SetPosition(0.0f,4.0f, 3.0f);
  box2.SetSize(10.0f,10.0f,2.0f);
  box2.SetColor(0.7f,0.7f,0.25f);
  scene.AddPrimitive(box2);
  if (argc<3){
    std::cerr<<"Usage: ./camera_example cam_res envfile.json \n";
    return 1;
  }
  int pix = atoi(argv[1]);
  camera.Initialize(pix,pix,0.0035f,0.0035f,0.0035f);
  std::string env_file(argv[2]);
  env.Load(env_file);
#endif

  env.SetRaytracer(&scene);
  camera.SetEnvironmentProperties(&env);	
	camera.SetDrawAnnotations();
  glm::dvec3 position(-35.0, 0.0, 2.0);
  glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
#ifdef USE_MPI
  MPI_Barrier(MPI_COMM_WORLD);
#endif
  
  if (myid==0)std::cout<<"Done loading "<<scene.GetNumberTrianglesLoaded()<<
		" triangles, rendering"<<std::endl;
	int nsteps = 25;
  for (int i=0;i<nsteps;i++){
#ifdef USE_MPI    
    double t1 = MPI_Wtime();
#endif    
#ifdef USE_OMP
		double t1 = omp_get_wtime();
#endif
    camera.SetPose(position, orientation);
    camera.Update(&env,0.1);
    if(myid==0){
#ifdef USE_MPI
      std::cout<<"FPS = "<<1.0/(MPI_Wtime()-t1)<<std::endl;
#endif
#ifdef USE_OMP
			std::cout << "FPS = " << 1.0 / (omp_get_wtime() - t1) << std::endl;
#endif
      camera.Display(); 
    }
    position.x += 1.0;
  }
  /*
  if (myid ==0) {
    camera.Display();
    camera.SaveImage("output.bmp");
		//camera.AnnotateFrame(&env,false);
		//camera.SaveSegmentedImage("seg_objects.bmp");
		//camera.SaveAnnotationsCsv("annotations.csv");
		//camera.SaveAnnotationsXmlVoc("annotations.xml");
		camera.AnnotateFrame(&env, true);
		camera.DisplaySegmentedImage();
		camera.SaveSegmentedImage("seg_labels.bmp");
		camera.SaveSemanticAnnotationsCsv("annotations.csv");
		camera.SaveRangeImage("ranges.bmp");
		//camera.AnnotateFull(&env);
		//camera.SaveSegmentedImage("seg_full.bmp");
  }
  */
#ifdef USE_MPI
  MPI_Finalize();
#endif
  return 0;
}

