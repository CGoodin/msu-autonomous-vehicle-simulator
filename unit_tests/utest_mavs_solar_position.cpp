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
* \file utest_mavs_solar_position.cpp
*
* Unit test to evaluate the mavs solar position calculation
*
* Usage: >./utest_mavs_solar_position
*
* Check the solar position calculator, comparing results to values from
* the NOAA solar calculator.
* https://www.esrl.noaa.gov/gmd/grad/solcalc/azel.html
* for 1 Jan 2000 at 1800 UT (1200 CST) , 32 Lat, 90 Long, time zone = 6
*
* The values from the web page are "solar elevation = 35, solar azimuth = 179.03"
* The algorithm in MAVS for solar position is slightly less precise than the NOAA page,
* so there will be some differences.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/

#include "mavs_core/environment/solar_position.h"
#include <iostream>

int main(int argc, char *argv[]){
	mavs::environment::SolarPosition sunpos;

  std::cout.precision(10);
  mavs::environment::DateTime check_day;
  check_day.year = 2000;
  check_day.month = 1;
  check_day.day = 1;
  check_day.hour = 12;
  check_day.time_zone = 6;
  check_day.minute = 0;
  check_day.second = 0;
	sunpos.SetDateTime(check_day);
  //Location check_position;
  float lat = 32.0f;
  float lon = 90.0f;
  float deg2rad = 0.01745329252f;
	sunpos.SetLocalLatLong(lat, lon);
  mavs::environment::HorizontalCoordinate checkpos = sunpos.GetSolarPosition();
  std::cout<<"Solar Elevation (error) = "<<90-checkpos.zenith/deg2rad<<" ("<<
    (90-checkpos.zenith/deg2rad)-35<<")"<<std::endl;
  std::cout<<"Solar Azimuth (error) = "<<checkpos.azimuth/deg2rad<<" ("<<
    checkpos.azimuth/deg2rad-179.03<<")"<<std::endl;
  
  return 0;
}
