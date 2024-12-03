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
#include <sensors/lidar/os1_16.h>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

OusterOS1_16::OusterOS1_16(){
	SetBeamSpotCircular((float)(0.13*mavs::kDegToRad));
  SetRotationRate(10.0,2048);
	max_range_ = 360.0; // 120.0;
  min_range_ = 0.8;
}

void OusterOS1_16::SetRotationRate(float rot_hz, int mode){
	if (rot_hz < 10.0)rot_hz = 10.0;
	if (rot_hz > 20.0)rot_hz = 20.0;
	if (rot_hz != 10.0 && rot_hz != 20.0) {
		if (rot_hz < 15.0) {
			rot_hz = 10.0;
		}
		else {
			rot_hz = 20.0;
		}
	}
	if (mode == 2048) rot_hz = 10.0;
	if (mode != 512 && mode != 1024 && mode != 2048) {
		std::cerr << "ERROR in setting mode of OS1-16 lidar." << std::endl;;
		std::cerr << "Mode must be 512,1024, or 2048" << std::endl;
		exit(10);
	}
	rotation_rate_ = rot_hz;
	float res = 360.0f / (1.0f*mode);
	//float vres = 33.2f / 64.0f;
	float vres = 33.2f/15.0f;
  //SetScanPattern(-180.0f,180.0f-res,res,-16.6f,16.6f,vres);
  SetScanPattern(-180.0f,180.0f-res,res,-16.6f,16.6f,vres);
  SetTimeStep(1.0/rot_hz);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
