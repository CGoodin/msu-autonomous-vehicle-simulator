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
#include <iostream>
#include "sensors/ins/swiftnav_msg.h"

int main(int argc, char *argv[]) {

	mavs::swiftnav::sbpMessagesWithTimeStamp msg_out,msg_in;
	msg_out.pos_llh.lat = 100.0f;
	msg_out.pos_llh.lon = 200.0f;
	msg_out.pos_llh.height = -100.0f;
	msg_out.time = 1000;
	std::cout << msg_out << std::endl;
	msg_out.Write("output.sbp");

	msg_in.Read("output.sbp");
	std::cout << msg_in;

	std::cout << sizeof(msg_out) << " "<<sizeof(mavs::swiftnav::dwTime_t)<<" "<< sizeof(mavs::swiftnav::msg_pos_llh_t) << " " << sizeof(mavs::swiftnav::msg_vel_ned_t) << " " << sizeof(mavs::swiftnav::msg_dops_t) << " " << sizeof(mavs::swiftnav::msg_utc_time_t) << " " << sizeof(mavs::swiftnav::msg_orient_euler_t) << " " << sizeof(mavs::swiftnav::msg_orient_quat_t) << " " <<  sizeof(mavs::swiftnav::msg_angular_rate_t) << " " << sizeof(mavs::swiftnav::msg_imu_raw_t) << " " << sizeof(mavs::swiftnav::msg_mag_raw_t) << " " << sizeof(mavs::swiftnav::swiftnavFLAGS) << " " << std::endl;
	
	return 0;
}