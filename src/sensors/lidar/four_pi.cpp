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
#include <sensors/lidar/four_pi.h>
#include <limits>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

FourPiLidar::FourPiLidar(){
	angle_res_ = 0.25f; //degrees
	InitFourPi(angle_res_);
}

FourPiLidar::FourPiLidar(float res) {
	angle_res_ = res;
	InitFourPi(angle_res_);
}
void FourPiLidar::InitFourPi(float res){
	SetBeamSpotRectangular(0.0f, 0.0f);
	max_range_ = 1000.0f; // std::numeric_limits<float>::max();
	min_range_ = 0.0f;

  SetScanPattern(-180.0f,180.0f-res,res,-90.0f,90.0f,res);
  SetTimeStep(0.1);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
