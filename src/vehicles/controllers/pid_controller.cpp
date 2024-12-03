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
#include <vehicles/controllers/pid_controller.h>

#include <iostream>

namespace mavs {
namespace vehicle {

PidController::PidController(){
  kp_ = 0.3;
  ki_ = 0.0;
  kd_ = 0.05;
  setpoint_ = 0.0;
  previous_error_ = 0.0;
  integral_ = 0.0;
}

// see: https://en.wikipedia.org/wiki/PID_controller
double PidController::GetControlVariable(double measured_value, double dt){
  double error = setpoint_ - measured_value;
  integral_ += error*dt;
  double derivative = (error - previous_error_)/dt;
  double output = kp_*error + ki_*integral_ + kd_*derivative;
  previous_error_ = error;
  return output;
}

} //namespace vehicle
} //namespace mavs