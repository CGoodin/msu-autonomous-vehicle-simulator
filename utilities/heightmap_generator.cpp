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
* \file heightmap_generator.cpp
*
* Generate a square heightmap for ANVEL from a surface
*
* Usage: >./heightmap_generator scene_file.json (resolution)
*
* scene_file.json is a MAVS scene file, examples found
* in mavs/data/scenes.
*
* resolution is an optional parameter that specifies the 
* spatial resolution of the heightmap in meters. If not
* included, a default of 1.0 meters is used.
*
* Save the output with a .raw file extension for use as an 
* ANVEL heightmap
*
* \author Chris Goodin
*
* \date 2/22/2019
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <iostream>
#include <mavs_core/math/utils.h>
#include <CImg.h>
#include <raytracers/embree_tracer/embree_tracer.h>

int main(int argc, char *argv[]) {
	if (argc<2) {
		std::cerr << "Usage: ./scene_viewer scenefile.json " << std::endl;
		return 1;
	}
	float res = 1.0f;
	if (argc>2) {
		res = (float)atof(argv[2]);
	}

	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);

	glm::vec3 ll = scene.GetLowerLeftCorner();
	glm::vec3 ur = scene.GetUpperRightCorner();
	glm::vec3 v = ur - ll;
	std::cout << "Lower left = " << ll.x << " " << ll.y << std::endl;
	float size = std::max(v.x, v.y);
	int n_base = (int)std::ceil(size / res);
	int seed = 1;
	int n = 1;
	while (n < n_base) {
		n = (int)pow(2, seed) + 1;
		seed++;
	}
	std::cout << n_base << " " << n << std::endl;

	cimg_library::CImg<float> image(n, n, 1 ,1);
	image = 0.0;

	for (int i = 0; i < n; i++) {
		float x = ll.x + i * res;
		for (int j = 0; j < n; j++) {
			float y = ll.y + j * res;
			float z = scene.GetSurfaceHeight(x, y);
			if (z < ll.z)z = ll.z;
			z = z - ll.z;
			image.draw_point(n - i - 1, n - j - 1, (float*)&z);
		}
	}
	image.normalize(0.0f, 65535.0);
	cimg_library::CImg<unsigned short int> raw_image(n, n, 1, 1);
	//image.normalize(0.0f, 255.0);
	//cimg_library::CImg<unsigned char> raw_image(n, n, 1, 1);

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			unsigned short int p = (unsigned short int)image(i, j);
			raw_image.draw_point(i, j, (unsigned short int*)&p);
			//unsigned char p = (unsigned char)image(i, j);
			//raw_image.draw_point(i, j, (unsigned char*)&p);
		}
	}
	//raw_image.display();

	std::string ofname = mavs::utils::GetSaveFileName();
	raw_image.save(ofname.c_str());

	//std::cout << "Lower left corner: (" << ll.x << ", " << ll.y << ", " << ll.z << ")" << std::endl;
	//std::cout << "Upper right corner: (" << ur.x << ", " << ur.y << ", " << ur.z << ") " << std::endl;
	std::cout << "heightOffset = " << (0.5*v.z) << std::endl;
	std::cout << "heightScale = " << (0.5*v.z) << std::endl;
	std::cout << "cellSize = " << n * res << std::endl;
	return 0;
}


