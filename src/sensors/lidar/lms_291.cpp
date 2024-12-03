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
#include <sensors/lidar/lms_291.h>

namespace mavs{
namespace sensor{
namespace lidar{

static const double spin_time = 1.0/75.0;

Lms291_S05::Lms291_S05(){
  SetMode(1);
  SetBeamSpotCircular(0.012913f);
  OneDegreeResolution();
  SetTimeStep(spin_time);
  min_range_ = 1.0;
  max_range_ = 80.0;
  rotation_rate_ = 75.0; //hz
  repitition_rate_ = 75.0;
  is_planar_ = true;
}

void Lms291_S05::OneDegreeResolution(){
  angle_min_ = (float)(-90*kDegToRad);
  angle_max_ = (float)(90*kDegToRad);
  angle_increment_ = (float)(1.0*kDegToRad);
  SetScanPattern(-90,90,1.0);
}

void Lms291_S05::HalfDegreeResolution(){
  angle_min_ = (float)(-90*kDegToRad);
  angle_max_ = (float)(90*kDegToRad);
  angle_increment_ = (float)(0.5*kDegToRad);
  repitition_rate_ = (float)(0.5*repitition_rate_);
  SetScanPattern(-90,90,0.5);
}

void Lms291_S05::QuarterDegreeResolution(){
  angle_min_ = (float)(-50*kDegToRad);
  angle_max_ = (float)(50*kDegToRad);
  angle_increment_ = (float)(0.25*kDegToRad);
  repitition_rate_ = 0.25f*repitition_rate_;
  SetScanPattern(-50,50,0.25);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
