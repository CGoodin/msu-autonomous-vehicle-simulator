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
* \file halo_car_lidar_analysis.cpp
*
* Programatically analyzes lidar of the Halo vehicle
*
* Usage: >./halo_car_lidar_analysis
* 
* \author Chris Goodin
*
* \date 11/13/2018
*/
#include <iostream>
#include <vector>
#include "simulation/halo/halo_car.h"
#include <glm/glm.hpp>

int main(int argc, char *argv[]) {

	std::vector<std::string> scenes;
	scenes.push_back("rough_0_allveg_scene.json");
	scenes.push_back("rough_0_halfveg_scene.json");
	scenes.push_back("rough_0_noveg_scene.json");
	scenes.push_back("rough_50_allveg_scene.json");
	scenes.push_back("rough_50_halfveg_scene.json");
	scenes.push_back("rough_50_noveg_scene.json");
	scenes.push_back("rough_25_allveg_scene.json");
	scenes.push_back("rough_25_halfveg_scene.json");
	scenes.push_back("rough_25_noveg_scene.json");

	glm::vec3 init_pos(-20.0f, -20.0f, 0.0f);
	glm::quat init_ori(0.92388f, 0.0f, 0.0f, 0.38268f);

	for (int s = 0; s < (int)scenes.size(); s++) {
		mavs::halo::HaloCar halo_car;
		halo_car.SetInitialVehiclePose(init_pos, init_ori);
		halo_car.SetVehicleGoal(0.0f, 0.0f);
		halo_car.TurnOnSensor(7);
		halo_car.TurnOnSensor(8);
		halo_car.TurnOnSensor(9);
		std::string basename = scenes[s].substr(0, scenes[s].size() - 5);
		halo_car.SetWriteCombinedPointCloud(basename,5.0f);

		halo_car.LoadScene(scenes[s]);
		
		halo_car.Run();
	}
	return 0;
}
