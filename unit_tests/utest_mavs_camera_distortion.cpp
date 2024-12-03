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
* \file utest_mavs_camera_distortion.cpp
*
* Unit test to evaluate the mavs camera distortion model
*
* Usage: >./utest_mavs_camera_distortion
*
* Creates a distortion model, then prints the undistorted
* and distorted positions of those pixels. 
*
* Correct output is: 
*
* Undistorted.u Undistorted.v Distorted.u Distorted.c
*
* 0 0 23.3065 18.3965
*
* 0 483 23.6183 466.037
*
* 603 0 581.996 18.3743
*
* 603 483 582.06 466.059
*
* 302 242 303.22 242.83
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <iostream>
#include "sensors/camera/distortion_model.h"


int main(int argc, char *argv[]) {
	
	//create a camera system
	int num_horizontal_pix = 604;
	int num_vertical_pix = 484;
	float focal_array_width = 0.003215f;
	float focal_array_height = 0.002576f;
	float focal_length = 0.0035f;
	float xpixdim = focal_array_width / num_horizontal_pix;
	float ypixdim = focal_array_height / num_vertical_pix;
	float half_horizontal_dim = 0.5f*num_horizontal_pix;
	float half_vertical_dim = 0.5f*num_vertical_pix;

	//create a distortion model
	mavs::sensor::camera::DistortionModel dm;
	glm::vec2 cc, fc;
	std::vector<float> kc;
	kc.resize(5, 0.0f);
	fc.x = 657.30f;
	fc.y = 657.74f;
	cc.x = 302.72f;
	cc.y = 242.33f;
	float alpha_c = 0.00042f;
	kc[0] = -0.25349f;
	kc[1] = 0.11868f;
	kc[2] = -0.00028f;
	kc[3] = 0.00005f;
	dm.SetDistortionParameters(cc, fc, alpha_c, kc);
	dm.SetNominalFocalLength(focal_length);

	//Define a several pixels, in this case the top left corner
	std::vector<glm::vec2> pixels;
	pixels.resize(5);
	pixels[0] = glm::vec2(0, 0);
	pixels[1] = glm::vec2(0, num_vertical_pix - 1);
	pixels[2] = glm::vec2(num_horizontal_pix - 1, 0);
	pixels[3] = glm::vec2(num_horizontal_pix - 1, num_vertical_pix - 1);
	pixels[4] = glm::vec2(half_horizontal_dim, half_vertical_dim);

	std::cout << "Undistorted.u" << " " << "Undistorted.v" << " " <<
		"Distorted.u" << " " << "Distorted.c" << std::endl;
	for (int i = 0; i < (int)pixels.size(); i++) {
		// get the location of the pixel in pixel plane coordinates
		glm::vec2 meters(
			(pixels[i].x - half_horizontal_dim + 0.5f)*xpixdim,
			(pixels[i].y - half_vertical_dim + 0.5f)*ypixdim);
		meters = meters / focal_length;

		//distort it
		glm::vec2 xd = dm.Distort(meters);
		
		//print the results
		std::cout << pixels[i].x << " " << pixels[i].y << " " <<
			xd.x << " " << xd.y << std::endl;
	}
	return 0;
}

