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
* \file utest_mavs_rtk.cpp
*
* Unit test to evaluate the mavs rtk sensor
*
* Usage: >./utest_mavs_rtk (output_type) (max_error) (droput_rate) 
* 
* where 
* output_type can be either "all" or "position"
* max_error is the maximum error in meters and
* droput_rate is the # of gps dropouts/hour
*
* Creates an RTK sensor that uses an empirical error model
* and runs it for four hours. Prints the position error to stdout.
*
* \author Chris Goodin
*
* \date 4/29/2019
*/
#include <stdlib.h>
#include <sensors/rtk/rtk.h>

int main (int argc, char *argv[]){
  
  mavs::environment::Environment env;
  env.SetLocalOrigin(32.6526,-90.8779,73.152); //Vicksburg, MS
  
  float err = 0.35f;
  float drop_rate = 3.0f;
  std::string output_type = "position";
  if (argc>1){
    std::string out_type(argv[1]);
    if (out_type=="all" || out_type=="position"){
      output_type = out_type;
    }
  }
  if (argc>2){
    err = (float)atof(argv[2]);
  }
  if (argc>3){
    drop_rate = (float)atof(argv[3]);
  }


  mavs::VehicleState veh_state;
  veh_state.pose.position.x = 5.0;
  veh_state.pose.position.y = 10.0;
  veh_state.pose.position.z = 2.0;
  
  mavs::sensor::rtk::Rtk rtk;
  rtk.SetError(err);
  rtk.SetDropoutRate(drop_rate);
	rtk.SetWarmupTime(600.0f);

  float t = 0.0f;
  float dt = 0.05f;
  float path_radius = 15.0f;
  while (t<14400.0f){
    veh_state.pose.position.x = path_radius*cos(t/100.0);
    veh_state.pose.position.y = path_radius*sin(2.0*t/100.0);
    rtk.SetPose(veh_state);
	  rtk.Update(&env, dt);
    rtk.PrintCurrentOdometry(output_type);
    t += dt;
  }
  
  return 0;
}
