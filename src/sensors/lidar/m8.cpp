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
#include <sensors/lidar/m8.h>

#include <mavs_core/math/utils.h>
#include <mavs_core/math/constants.h>

namespace mavs{
namespace sensor{
namespace lidar{

MEight::MEight(){
  //M8 spins at a constant rate of 10Hz
  // Lasers fire at 53,828 Hz, 5,383 fires/spin
  // 420,000 points per second in single return mode
  // 42,000 points per rotation at 10 Hz
  // 5,250 * 8 = 42,000
  // 360/5250 = 0.06857....

  // no info about the laser divergence in spec sheet
  // set it to the same as velodyne
  SetBeamSpotRectangular(0.0033f, 0.0007f);
  SetRotationRate(10.0f);
  max_range_ = 150.0f;
  min_range_ = 1.0f;
  SetMode(1);
}

void MEight::SetRotationRate(float rot_hz){
  rot_hz = math::clamp(rot_hz,5.0f,20.0f);
  float res = 0.006857f*rot_hz;
  SetScanPattern(-180.0f,180.0f-res,res,-18.22f,3.2f,3.06f);
  SetTimeStep(1.0f/rot_hz);
	rotation_rate_ = rot_hz;
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
