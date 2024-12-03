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
#include <sensors/lidar/hdl64e_simple.h>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

Hdl64ESimple::Hdl64ESimple() {
	Hdl64ESimple(10.0);
}

Hdl64ESimple::Hdl64ESimple(float rep_rate){
	float delta = 0.0187f*rep_rate - 0.015f;
  SetBeamSpotRectangular(0.0033f, 0.0007f);
  SetScanPattern(-180.0f,180.0f-delta,delta,-24.8f,2.0f,0.425396825f);
  SetTimeStep(0.1f);
  rotation_rate_ = 10.0f;
  max_range_ = 120.0f;
  min_range_ = 1.0f;
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
