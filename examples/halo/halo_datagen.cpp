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
* \file halo_datagen.cpp
*
* Easy-to-use (hopefully) tool for generating simulation data
* for the Halo car
*
* Usage: >./halo_datagen number_of_frames
*
* where number_of_frames is the number of labeled frames to generate.
*
* \author Chris Goodin
*
* \date 11/20/2018
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
	std::srand((unsigned int)time(0));
	int num_to_generate = 100;
	if (argc>1)num_to_generate = atoi(argv[1]);

	int num_generated = 0;
	int scene_num = 0;
	mavs::MavsDataPath mavs_data_path;
	std::string eco_path = mavs_data_path.GetPath() + "/ecosystem_files/";

	while (num_generated < num_to_generate) {
		//std::vector<mavs::terraingen::RandomSceneInputs> scene_inputs;
		mavs::terraingen::RandomSceneInputs forest_input;
		//mavs::terraingen::RandomSceneInputs desert_input;
		//mavs::terraingen::RandomSceneInputs meadow_input;
		// 5 acres
		forest_input.terrain_width = 142.25f;
		forest_input.terrain_length = 142.25f;
		forest_input.lo_mag = mavs::math::rand_in_range(0.5f,5.0f);
		forest_input.hi_mag = 0.0f;
		forest_input.mesh_resolution = 1.0f;
		forest_input.trail_width = 3.0f;
		forest_input.wheelbase = 1.25f;
		forest_input.track_width = 0.6f;
		forest_input.path_type = "Valleys";
		forest_input.basename = "forest";
		forest_input.plant_density = mavs::math::rand_in_range(0.25f, 0.85f);
		forest_input.output_directory = ".";
		forest_input.eco_file = eco_path + "american_southeast_forest.json";
		//scene_inputs.push_back(forest_input);
		//meadow
		/*
		meadow_input = forest_input;
		meadow_input.basename = "meadow";
		meadow_input.eco_file = eco_path + "american_southeast_meadow.json";
		inputs.push_back(meadow_input);
		//desert
		desert_input = forest_input;
		desert_input.basename = "desert";
		desert_input.eco_file = eco_path + "american_southwest_desert.json";
		//add inputs to list
		inputs.push_back(desert_input);
		*/

		// Create a random scene
		mavs::terraingen::RandomScene random_scene;
		random_scene.SetInputs(forest_input);
		random_scene.Create();

		std::string scene_file(forest_input.basename);
		scene_file = scene_file + "_scene.json";

		std::string path_file(forest_input.basename);
		path_file = path_file + "_path.vprp";

		//mavs::raytracer::embree::EmbreeTracer scene;
		//scene.Load(scene_file);

		//create the pose structure and load
		mavs::AnvelVprpReader pose_reader;
		std::vector<mavs::Pose> poses = pose_reader.Load(path_file, 5);

		mavs::halo::HaloCar halo_car;
		halo_car.GetEnvironment()->SetTurbidity(mavs::math::rand_in_range(2.0f, 10.0f));
		halo_car.GetEnvironment()->SetDateTime(2018, mavs::math::rand_in_range(1, 12), mavs::math::rand_in_range(1, 28), mavs::math::rand_in_range(9, 16), mavs::math::rand_in_range(0, 59), 0, 6);
		halo_car.LoadScene(scene_file);
		halo_car.LoadPoses(path_file);
		halo_car.SetUsePoses(true);
		halo_car.SetInitialVehiclePose(poses[0].position, poses[0].quaternion);
		halo_car.TurnOnSensor(1); //top left camera
		halo_car.TurnOnSensor(4); //rear right camera
		halo_car.TurnOnLogging("./");
		halo_car.SetDisplay(false);
		halo_car.SetSaveLabeled(true);
		halo_car.SetImageFrameRate(2.0f);
		halo_car.SetOutfilePrefix(("scene_" + mavs::utils::ToString(scene_num) + "_"));
		halo_car.Run((num_to_generate-num_generated));
		num_generated += halo_car.GetNumImageFrames();
		std::cout << "Generated = " << num_generated << " of " << num_to_generate 
			<<" images."<<std::endl;
		scene_num++;
	}

#endif
	return 0;
}

