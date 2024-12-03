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
#include <sensors/lidar/planar_lidar.h>

namespace mavs{
namespace sensor{
namespace lidar{

LaserScan PlanarLidar::GetLaserScan(){
  LaserScan scan;
  scan.angle_min = angle_min_;
  scan.angle_max = angle_max_;
  scan.angle_increment = angle_increment_;
  scan.time_increment = 1.0f/rotation_rate_; //time_increment_;
  scan.scan_time = 1.0f/repitition_rate_; //scan_time_;
  scan.range_min = min_range_;
  scan.range_max = max_range_;
  scan.ranges = distances_;
  scan.intensities = intensities_;
  return scan;
}

void PlanarLidar::SetScanProperties(float ang_min, float ang_max, float ang_res) {
	angle_min_ = (float)(mavs::kDegToRad*ang_min);
	angle_max_ = (float)(mavs::kDegToRad*ang_max);
	angle_increment_ = (float)(mavs::kDegToRad*ang_res);
	SetScanPattern(ang_min, ang_max, ang_res);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
