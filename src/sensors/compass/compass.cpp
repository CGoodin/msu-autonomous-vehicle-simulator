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
#include <sensors/compass/compass.h>

#include <iostream>
#include <fstream>
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <stdlib.h>

#include <glm/glm.hpp>

namespace mavs{
namespace sensor{
namespace compass{

Compass::Compass(){
  rms_error_degrees_ = 1.0;
  rms_error_radians_ = kDegToRad*rms_error_degrees_;
  type_ = "compass";
}

Compass::~Compass(){
}

void Compass::Update(environment::Environment *env, double dt){
  
  current_heading_ = atan2(look_to_.y, look_to_.x) + 
    rms_error_radians_*((double)rand()/(RAND_MAX));

  local_sim_time_ += local_time_step_;
  updated_ = true;
}

double Compass::GetHeading(){
  return current_heading_;
}
#ifdef USE_MPI
void Compass::PublishData(int root,MPI_Comm broadcast_to){
  MPI_Bcast(&current_heading_,1,MPI_DOUBLE,root,broadcast_to);
}
#endif
} //namespace compass
} //namespace sensor
} //namespace mavs

