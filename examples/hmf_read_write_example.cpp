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
* \file hmf_read_write_example.cpp
* 
* Demonstrates how MAVS can be used to load and write out
* applanix heightmap files (.hmf).
*
* Usage: >./hmf_read_write_example heightmap.hmf
*
* heightmap.hmf is a binary input file in the Applanix .hmf format
* an example can be found in mavs/data/hmf_files
*
* The code will load in the hmf file, then write it back out. 
* A window displaying the hmf file will pop up, close the window 
* to finish the program.
*
* \author Chris Goodin
* 
* \date 10/4/2018
*/
#include <iostream>

#include <sensors/lidar/m8.h>
#include <sensors/io/applanix_io.h>
#include <mavs_core/terrain_generator/heightmap.h>
#include <raytracers/simple_tracer/simple_tracer.h>


int main(int argc, char *argv[]) {
	if (argc <= 1) {
		std::cerr << "ERROR, must provide .hmf file as input " << std::endl;
	}

	//input binary heightmap file with .hmf extension
	std::string hmf_file(argv[1]);

	mavs::io::ApplanixHeightMap map1,map2;

	//load the heightmap file
	map1.LoadHmf(hmf_file);

	//Write the file back out to a new file
	map1.WriteHmf("test.hmf");

	//To test, load the newly created .hmf
	map2.LoadHmf("test.hmf");

	//compare the stats of the two to check the read/write
	map1.PrintStats();
	map2.PrintStats();

	//convert to a heightmap and display
	mavs::terraingen::HeightMap converted = map2.ConvertToMavsHeightMap();
	converted.Display(false);

	// Create a simple MAVS scene and scan with an M8
	// The scan will be used to create an applanix hmf
	mavs::raytracer::SimpleTracer scene;
	mavs::raytracer::Aabb floor;
	floor.SetSize(1.0E6, 1.0E6, 0.01);
	floor.SetColor(0.25, 0.25, 0.25);
	floor.SetPosition(0.0, 0.0, 0.0);
	scene.AddPrimitive(floor);
	std::vector<mavs::raytracer::Aabb> boxes;
	int nboxes = 20;
	float theta_step = (float)(360.0 / nboxes);
	boxes.resize(nboxes);
	for (int i = 0; i < nboxes; i++) {
		float theta = (float)(mavs::kDegToRad*theta_step*i);
		float r = 2.0f * (i) + 10.0f;
		boxes[i].SetPosition(r*cos(theta), r*sin(theta), 0.0f);
		boxes[i].SetSize(2.0, 2.0, 2.0*(i+1));
		boxes[i].SetColor(0.25, 0.25, 0.25);
		scene.AddPrimitive(boxes[i]);
	}
	mavs::environment::Environment env;
	env.SetRaytracer(&scene);
	mavs::sensor::lidar::MEight m8;
	glm::dvec3 position(0.0, 0.0, 1.0);
  glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
	m8.SetPose(position, orientation);
	m8.Update(&env, 0.1);
	//m8.WritePointsToImage("scan.bmp");

	//Create an applanix heightmap from the pointcloud
	mavs::PointCloud pc = m8.GetRosPointCloud();
	mavs::io::ApplanixHeightMap lidar_map(pc);
	
	//Convert to a raw heightmap
	mavs::terraingen::HeightMap mavs_hm = lidar_map.ConvertToMavsHeightMap();
	
	// Create a GridSurface - DEM-like structure
	mavs::terraingen::GridSurface surface(mavs_hm);

	//Display a grayscale fo the heightmap
	//surface.Display();

	//save it to a wavefront .obj mesh
	//surface.WriteObj("hmf");

	return 0;
}