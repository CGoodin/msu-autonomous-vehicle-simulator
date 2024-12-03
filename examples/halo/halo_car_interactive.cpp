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
* \file halo_car_interactive.cpp
*
* Drive the halo car through the scene and record data, etc.
*
* Usage: >./halo_car_example mavs_scene_file.json 
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
* 
* \author Chris Goodin
*
* \date 11/2/2018
*/
#include <iostream>

#include "simulation/halo/halo_car.h"

int main(int argc, char *argv[]) {

	mavs::halo::HaloCar halo_car;
	halo_car.SetGpsFrameRate(20.0f);
	halo_car.SetCanBusRate(20.0f);
	halo_car.SetLidarFrameRate(5.0);

	if (argc < 2) {
		halo_car.LoadScene();
	}
	else {
		std::string scene_file(argv[1]);
		halo_car.LoadScene(scene_file);
	}

	if (argc > 2) {
		std::string chrono_inputs(argv[2]);
		halo_car.UseChronoVehicle(chrono_inputs);
	}

	//halo_car.AddActor("hmmwv");

	halo_car.RunInteractive();
 
	return 0;
}
