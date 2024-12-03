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
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <CImg.h>
#ifdef None
#undef None
#endif
#include <glm/glm.hpp>
#include <mavs_core/math/utils.h>
#include "sensors/camera/add_rain_to_existing_image.h"

int main(int argc, char * argv[]) {

	std::string imagefile(argv[1]);
	float rate = (float)atof(argv[2]);
	cimg_library::CImg<float> image;
	image.load(imagefile.c_str());

	mavs::sensor::camera::AddRainToImage(image, rate, true);

	//image.display();
	std::string outfile = "raining_";
	outfile.append(imagefile);
	image.save(outfile.c_str());

	return 0;
}