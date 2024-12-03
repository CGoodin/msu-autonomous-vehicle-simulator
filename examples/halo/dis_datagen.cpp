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
* \file dis_datagen.cpp
*
* Generate SLAM input data for the DIS project
*
* Usage: >./dis_datagen
*
* \author Chris Goodin
*
* \date 11/26/2018
*/
#include<cstdlib>
#include<ctime>

#include <iostream>
#include <string>

#include <mavs_core/terrain_generator/random_scene.h>
#include <mavs_core/data_path.h>
#include <mavs_core/pose_readers/anvel_vprp_reader.h>
#include <mavs_core/math/utils.h>

#include "simulation/halo/halo_car.h"

int main(int argc, char *argv[]) {
#ifdef USE_EMBREE
	std::vector<std::string> scenefiles,prefixes;
	scenefiles.push_back("/scenes/dis_forest_sparse_scene.json");
	scenefiles.push_back("/scenes/dis_forest_medium_scene.json");
	scenefiles.push_back("/scenes/dis_forest_dense_scene.json");
	prefixes.push_back("sparse_");
	prefixes.push_back("medium_");
	prefixes.push_back("dense_");
	for (int s = 0; s < 3; s++) {
		mavs::MavsDataPath mavs_data_path;
		//std::string scene_file = mavs_data_path.GetPath() + "/scenes/desert_dis_scene.json";
		std::string scene_file = scenefiles[s];
		//std::string scene_file = mavs_data_path.GetPath() + "/scenes/cube_scene.json";
		std::string path_file = mavs_data_path.GetPath() + "/waypoints/Ball_DIS.vprp";
		//create the pose structure and load
		mavs::AnvelVprpReader pose_reader;
		std::vector<mavs::Pose> poses = pose_reader.Load(path_file, 1);

		mavs::halo::HaloCar halo_car;
		halo_car.SetOutfilePrefix(prefixes[s]);
		halo_car.LoadScene((mavs_data_path.GetPath()+scene_file));
		halo_car.LoadPoses(path_file);
		halo_car.SetUsePoses(true);
		halo_car.SetInitialVehiclePose(poses[0].position, poses[0].quaternion);
		halo_car.WriteSlamOutput();
		//halo_car.TurnOnSensor(1);
		//halo_car.SetImageFrameRate(0.5f);
		halo_car.TurnOnLogging("./");
		halo_car.SetDisplay(false);
		halo_car.Run();

	}
#endif
	return 0;
}

