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
* \file batch_convert_obj.cpp
*
* Convert a list of obj files to binary and save in place
*
* Usage: >./batch_obj_convert input_file_list.txt
*
* \author Chris Goodin
*
* \date 3/10/2020
*/
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <raytracers/mesh.h>
#include <mavs_core/math/utils.h>

int main(int argc, char *argv[]) {

	if (argc < 2) {
		std::cerr << "Usage: ./batch_obj_convert file_list.txt" << std::endl;
		exit(1);
	}
	std::vector<std::string> files_to_convert;
	std::ifstream fin(argv[1]);
	std::string infile;
	while (!fin.eof()) {
		fin >> infile;
		files_to_convert.push_back(infile);
	}
	files_to_convert.pop_back();

	std::string inpath = files_to_convert[0];

	for (int i = 1; i < files_to_convert.size(); i++) {
		std::string meshfile = files_to_convert[i];
		if (mavs::utils::file_exists(meshfile)) {
			std::cout << "Converting " << meshfile << std::endl;
			mavs::raytracer::Mesh mesh;
			mesh.Load(inpath, inpath+meshfile);
			std::string outfile_name = meshfile;
			for (int t = 0; t < 3; t++)outfile_name.pop_back();
			outfile_name.append("bin");
			mesh.WriteBinary(inpath,outfile_name);
		}
	}
	return 0;
}
