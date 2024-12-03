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
* \file top_down_ortho.cpp
*
* Generates a top-down orthographic rendering of a scene
* and saves it to a ppm.
*
* Usage: > top_down_ortho scene_file.json (display)
*
* scene_file.json is a MAVS scene file, examples found
* in mavs/data/scenes, and display tells the program
* to display the result (display=1) or not (display=0).
*
* The program saves two files, "height_orthoview.ppm" and
* "color_orthoview.ppm"
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <iostream>
#include <mavs_core/environment/environment.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <sensors/camera/ortho_camera.h>

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: ./top_down_orth scenefile.json" << std::endl;
		return 1;
	}

	bool display = false;
	if (argc >= 3) {
		int disp = atoi(argv[2]);
		if (disp > 0)display = true;
	}

	std::string scene_file(argv[1]);

	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	glm::vec3 ur = scene.GetUpperRightCorner();
	glm::vec3 ll = scene.GetLowerLeftCorner();
	glm::vec3 center = 0.5f*(ur + ll);
	mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	float pixres = 1.0f;
	float dx = ur.x - ll.x;
	float dy = ur.y - ll.y;
	int nx = (int)ceil(dx / pixres);
	int ny = (int)ceil(dy / pixres);
	glm::vec3 position(center.x, center.y, ur.z + 10.0f);
	glm::quat orientation(0.7071f, 0.0f, 0.7071f, 0.0f);
	std::cout<<"Done loading "<<scene.GetNumberTrianglesLoaded()<<" triangles, rendering"<<std::endl;

	mavs::sensor::camera::OrthoCamera cam;
	cam.Initialize(ny, nx, dy,dx,1.0f);
	cam.SetPose(position, orientation);
	cam.Update(&env,0.03);
	cam.SaveImage("color_orthoview.ppm");
	cam.SaveImage("color_orthoview.bmp");
	cam.SaveRangeImage("height_orthoview.ppm");
	cam.SaveRangeImage("height_orthoview.bmp");
	if (display) {
		cam.Display();
	}

	return 0;
}

