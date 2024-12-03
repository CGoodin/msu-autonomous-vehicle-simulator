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
* \file utest_mavs_vti.cpp
*
* Unit test to evaluate implementation of mavs vti
*
* Usage: >./utest_mavs_vti rci
*
* RCI is the remold-cone index, int PSI. 
* It is an optional argument, if not passed, default value is used.
*
* \author Chris Goodin
*
* \date 2/7/2019
*/
#include <stdlib.h>
#include <iostream>
#include <fstream>
//#include "vehicles/vti/fine_grained.h"
//#include "vehicles/vti/coarse_grained.h"
//#include "vehicles/vti/brixius.h"
#include <vti/fine_grained.h>
#include <vti/coarse_grained.h>
#include <vti/brixius.h>
#include <mavs_core/math/constants.h>
#include <CImg.h>

int main (int argc, char *argv[]){
	//--- Set up the CImg variables for plotting results ---//
	const unsigned int green[3] = { 0, 255, 0 };
	const unsigned int red[3] = { 255, 0, 0 };
	const unsigned int blue[3] = { 0, 0, 255 };
	const unsigned int yellow[3] = { 255, 255, 0 };
	const unsigned int white[3] = { 255, 255, 255 };
	cimg_library::CImg<unsigned int> traction_image;
	int nx = 512;
	int ny = 512;
	traction_image.assign(nx, ny, 1, 3, 0);
	traction_image.draw_line(0, ny / 2, nx, ny / 2, white, 1);
	traction_image.draw_line(nx / 2, 0, nx / 2, ny, white, 1);

	//--- Set the parameters for the vti models ---//
	//mavs::vti::FineGrained fg;
	mavs::vti::Brixius bx;
	mavs::vti::CoarseGrained cg;
	double load = 3.892; //kilo-Newtons
	double deflection = 0.1; //meters
	double rci = 310; //kilo-Pascals, about 45 PSI
	if (argc > 1) {
		double ci_in = (double)atof(argv[1]); 
		//convert input from PSI to kPa
		rci = 6.89476*ci_in;
	}

	//--- Calculate values over slip range ---//
	double slip = -1.0;
	double steering = 0.0;
	while (slip <= 1.0) {
		// update vti models at different slips
		cg.Update(rci, load, deflection, slip, steering);
		bx.Update(rci, load, deflection, slip, steering);

		//add results to plot
		int i = (int)(0.5*nx * (slip+1.0));
		int j_ct = ny - (int)(0.5*ny*(cg.GetLongitudinalTraction() + 1.0)) - 1;
		int j_cr = ny - (int)(0.5*ny*(cg.GetLongitudinalMotionResistance() + 1.0)) - 1;
		int j_ft = ny - (int)(0.5*ny*(bx.GetLongitudinalTraction() + 1.0)) - 1;
		int j_fr = ny - (int)(0.5*ny*(bx.GetLongitudinalMotionResistance() + 1.0)) - 1;
		traction_image.draw_circle(i, j_ct, 2, green, 1.0, 1);
		traction_image.draw_circle(i, j_cr, 2, red, 1.0, 1);
		traction_image.draw_circle(i, j_ft, 2, blue, 1.0, 1);
		traction_image.draw_circle(i, j_fr, 2, yellow, 1.0, 1);

		//increment slip
		slip += 0.01;
	}

	std::ofstream fout("traction.txt");
	slip = -1.0;
	while (slip <= 1.0) {
		steering = -22.0*mavs::kDegToRad;
		while (steering <= (22.0*mavs::kDegToRad)) {
			bx.Update(rci, load, deflection, slip, steering);
			cg.Update(rci, load, deflection, slip, steering);
			fout << slip << " " << steering << " " << bx.GetLongitudinalTraction() << " " << bx.GetLateralTraction() << " "<<cg.GetLongitudinalTraction()<<" "<<cg.GetLateralTraction()<<std::endl;
			steering += 0.1*mavs::kDegToRad;
		}
		slip += 0.05;
	}
	fout.close();

	//display results
	traction_image.display();
  return 0;
}
