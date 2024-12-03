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
// MPI has to be included first or 
// the compiler chokes up
#ifdef USE_MPI
#include <mpi.h>
#endif

#include <sensors/camera/nir_camera.h>

#include <iostream>
#include <ctime>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace camera{

NirCamera::NirCamera(){
  updated_ = false;
  local_sim_time_ = 0.0f;
  Initialize(512,512,0.0035f,0.0035f,0.0035f);
  gamma_ = 0.75f;
	gain_ = 1.0f; 
	background_color_ = 0.0f;
	camera_type_ = "nir";
}

NirCamera::~NirCamera(){

}

void NirCamera::Update(environment::Environment *env, double dt){
	CheckFreq(dt);
	if (disp_is_free_)UpdatePoseKeyboard();
  //define directions
  glm::vec3 look_to_f = focal_length_*look_to_;
  glm::vec3 look_side_d = horizontal_pixdim_*look_side_;
  glm::vec3 look_up_d = vertical_pixdim_*look_up_;

  ZeroBuffer();
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
  for (int i=0;i<num_horizontal_pix_;i++){
    for (int j=0;j<num_vertical_pix_;j++){
			int n = j + i * num_vertical_pix_;
#ifdef USE_MPI
      if ( (n)%comm_size_==comm_rank_){
#endif
				glm::vec3 direction = look_to_f 
					+ (i-half_horizontal_dim_+0.5f)*look_side_d 
					+ (j-half_vertical_dim_+0.5f)*look_up_d;
					direction = glm::normalize(direction);
					glm::vec3 pix_color(background_color_,background_color_,background_color_);  
					raytracer::Intersection inter = 
						env->GetClosestIntersection(position_,direction);
					if (inter.dist > 0.0f) {
						//assume sun is directly overhead
						if (inter.normal.z < 0.0f)inter.normal = -1.0f*inter.normal;
						float brdf = 0.25f + 0.75f*std::max(inter.normal.z,0.0f) / glm::length(inter.normal);
						float rho = 0.1f;
						if (inter.spectrum_name.size()>0) {
							rho = env->GetScene()->GetReflectance(inter.spectrum_name, 0.905f); 
						}
						float intens = brdf * 255.0f*rho;
						pix_color = glm::vec3(intens, intens, intens);
						segment_buffer_[n] = inter.object_id;
						range_buffer_[n] = inter.dist;
					}
					//image_buffer_[n] = pix_color;
					for (int c = 0; c < 3; c++)image_(i, j, c) = pix_color[c];
#ifdef USE_MPI
      }
#endif
    }
  }

  ReduceImageBuffer();

	CopyBufferToImage();

	image_.normalize(0, 255);

  if (log_data_) {
    SaveImage(utils::ToString(local_sim_time_)+log_file_name_); }

  local_sim_time_ += local_time_step_;
  updated_ = true;
}

} //namespace camera
} //namespace sensor
} //namespace mavs
