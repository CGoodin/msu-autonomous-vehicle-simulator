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
#include <mavs_core/environment/clouds.h>
#include <mavs_core/math/constants.h>
#include <iostream>
#include <fstream>
#include <time.h>

namespace mavs {
namespace environment {

Clouds::Clouds() {
	SetCloudCoverFraction(0.0f);
	cloud_noise_low_.SetFrequency(0.01);
	cloud_noise_high_.SetFrequency(0.1);
	int curr_time = (int)time(NULL);
	cloud_noise_low_.SetSeed(curr_time);
	cloud_noise_high_.SetSeed(curr_time+1);
	image_.assign(512, 512, 1, 3, 0.0);
	//cloud_color_ = 255.0f*glm::vec3(0.1f, 0.1f, 0.1f);
	cloud_color_ = glm::vec3(2.0f, 2.0f, 2.0f);
}

float Clouds::GetCloudDensity(glm::vec3 direction) {
	float x = 1000.0f*direction.x*direction.z;
	float y = 1000.0f*direction.y*direction.z;
	float alpha = 4.0f;
	float z = 0.0f;
	double theta = mavs::kRadToDeg*(mavs::kPi_2 - acos(direction.z));
	if (theta > 2.0) {
		z = (float)((1.0f + cloud_noise_low_.GetPerlin(x, y)) + 0.15f*(1.0f + cloud_noise_high_.GetPerlin(x, y)));
		z = 0.5f*z;
		if (z > thresh_) {
			float d = (z - thresh_) / intens_range_;
			z = exp(-alpha*d);
		}
		else {
			z = 0.0f;
		}
	}
	return z;
}

glm::vec3 Clouds::GetCloudColor(glm::vec3 direction) {
	float z = GetCloudDensity(direction);
	return z*cloud_color_;
}

void Clouds::WriteHemisphere() {
	float az = 0.0f;
	float zen = 0.0f;
	float az_max = 360.0f;
	float zen_max = 90.0f;
	float step = 1.5f;
	while (az < az_max) {
		zen = 0.0f;
		while (zen <= zen_max) {
			float sa = (float)sin(mavs::kDegToRad*az);
			float ca = (float)cos(mavs::kDegToRad*az);
			float sz = (float)sin(mavs::kDegToRad*zen);
			float cz = (float)cos(mavs::kDegToRad*zen);
			glm::vec3 direction(ca*sz, sa*sz, cz);
			float d = glm::length(GetCloudColor(direction));
			std::cout << direction.x << " " << direction.y << " " << direction.z << " " << d << std::endl;
			zen += step;
		}
		az += step;
	}
}

void Clouds::ShowCloudImage() {
	glm::vec3 blue(0.529f, 0.808f, 0.922f);
	blue = 255.0f*blue*0.1f;
	//glm::vec3 white(1.0f, 1.0f, 1.0f);
	float flen = 0.0035f;
	float imdim = 0.007f;
	float pixdim = imdim/(1.0f*image_.width());
	int x2 = image_.width() / 2;
	int y2 = image_.height() / 2;
	for (int i=0;i<image_.width();i++){
		for (int j=0;j<image_.height();j++){
			glm::vec3 direction((i - x2)*pixdim, (j - y2)*pixdim, flen);
			direction = direction / glm::length(direction);
			//float f = GetCloudDensity(direction);
			//glm::vec3 color = f * white + (1.0f - f)*blue;
			glm::vec3 color = GetCloudColor(direction);
			if (glm::length(color) > 0.0f) {
				image_.draw_point(i, j, (float *)&color);
			}
			else {
				image_.draw_point(i, j, (float *)&blue);
			}
		}
	}
	disp_ = image_;
}


} //namespace environment
} //namespace mavs