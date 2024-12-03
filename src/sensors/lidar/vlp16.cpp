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
#include <sensors/lidar/vlp16.h>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

Vlp16::Vlp16(){
  SetBeamSpotRectangular(0.0033f, 0.0007f);
  SetRotationRate(5.0);
  max_range_ = 100.0;
  min_range_ = 1.0;
}

void Vlp16::SetRotationRate(float rot_hz){
  rot_hz = math::clamp(rot_hz,5.0f,20.0f);
	rotation_rate_ = rot_hz;
  //float res = 0.2f*rot_hz;
  float res = 0.02f*rot_hz;
  SetScanPattern(-180.0f,180.0f-res,res,-15.0f,15.0f,2.0f);
  SetTimeStep(1.0/rot_hz);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
