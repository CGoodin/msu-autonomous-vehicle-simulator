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
#include <sensors/camera/rccb_camera.h>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace sensor {
namespace camera {


void RccbCamera::Update(environment::Environment *env, double dt) {
	CheckFreq(dt);
	RgbCamera::RenderFrame(env, dt);

	// see : Karanam, G. "Interfacing Red/Clear Sensors to ADSP-BF609� Blackfin Processors." Analog Devices, Norwood, Engineer-to-Engineer Note EE-358, Rev 1 (2013). 
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = j + i * num_vertical_pix_;
			//glm::vec3 oldcol = image_buffer_[n];
			//float new_green = 0.3f*oldcol.x +  0.59f*oldcol.y + 0.11f*oldcol.z;
			float new_green = 0.3f*image_(i,j,0) + 0.59f*image_(i,j,1) + 0.11f*image_(i,j,2);
			//glm::vec3 newcol(oldcol.x, new_green, oldcol.z);
			//image_buffer_[n] = newcol;
			image_(i, j, 1) = new_green;
		}
	}

	CopyBufferToImage();

	if (log_data_) {
		SaveImage(utils::ToString(local_sim_time_) + log_file_name_);
	}

	local_sim_time_ += local_time_step_;
	updated_ = true;
}


} //namespace camera
} //namespace sensor
} //namespace mavs