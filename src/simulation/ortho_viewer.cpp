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
#include <simulation/ortho_viewer.h>
#include <iostream>

namespace mavs{

OrthoViewer::OrthoViewer() {
	initialized_ = false;
	pixres_ = 1.0f;
}

OrthoViewer::~OrthoViewer() {

}

void OrthoViewer::Update(mavs::environment::Environment *env) {
	if (!initialized_) {
		glm::vec3 ur = env->GetScene()->GetUpperRightCorner();
		glm::vec3 ll = env->GetScene()->GetLowerLeftCorner();
		ur.x = std::min(ur.x, 2048.0f);
		ur.y = std::min(ur.y, 2048.0f);
		ll.x = std::max(ll.x, -2048.0f);
		ll.y = std::max(ll.y, -2048.0f);
		ll_ = glm::vec2(ll.x,ll.y);
		float dx = ur.x - ll.x;
		float dy = ur.y - ll.y;
		int nx = (int)ceil(dx / pixres_);
		int ny = (int)ceil(dy / pixres_);
		glm::vec3 center = 0.5f*(ur + ll);
		glm::vec3 position(center.x, center.y, ur.z + 10.0f);
		glm::quat orientation(0.7071f, 0.0f, 0.7071f, 0.0f);
		cam_.Initialize(ny, nx, dy, dx, 1.0f);
		//cam_.Initialize(nx, ny, dx, dy, 1.0f);
		cam_.SetPose(position, orientation);
		cam_.Update(env, 0.03);
		// now loop through and resize the image to remove empty pixels
		int ulo = cam_.GetWidth();
		int uhi = 0;
		int vlo = cam_.GetHeight();
		int vhi = 0;
		for (int i = 0; i < cam_.GetWidth(); i++) {
			for (int j = 0; j < cam_.GetHeight(); j++) {
				glm::vec3 color = cam_.GetPixel(i, j);
				if (glm::length(color) > 0.0f) {
					if (i > uhi)uhi = i;
					if (j > vhi)vhi = j;
					if (i < ulo)ulo = i;
					if (j < vlo)vlo = j;
				}
			}
		}
		cropped_image_ = cam_.GetCurrentImage().get_crop(ulo, vlo, uhi, vhi);
		std::vector<std::vector<glm::vec3> > pmap = cam_.GetPointsFromImage();
		float llx = std::numeric_limits<float>::max();
		float lly = std::numeric_limits<float>::max();
		for (int u = 0; u < pmap.size(); u++) {
			for (int v = 0; v < pmap[u].size(); v++) {
				glm::vec3 p = pmap[u][v];
				float mag = glm::length(p);
				if (mag > 0.0f && mag<10000.0f ) {
					if (p.x < llx)llx = p.x;
					if (p.y < lly)lly = p.y;
				}
			}
		}
		ll_.x = llx;
		ll_.y = lly;
	}
}

void OrthoViewer::Update(mavs::environment::Environment *env, std::vector<glm::vec2> waypoints) {
	if (!initialized_) {
		Update(env);
	}
	glm::vec3 yellow(255.0f, 255.0f, 0.0f);
	glm::vec3 green(0.0f, 255.0f, 0.0f);
	int nx = cropped_image_.width();
	int ny = cropped_image_.height();
	int cu = nx - (int)((0.0f - ll_.y) / pixres_);
	int cv = ny - (int)((0.0f - ll_.x) / pixres_);
	cropped_image_.draw_circle(cu, cv, 2, (float *)&green);
	for (int i = 0; i < waypoints.size()-1; i++) {
		int u0 = nx - (int)((waypoints[i].y - ll_.y) / pixres_);
		int v0 = ny - (int)((waypoints[i].x - ll_.x) / pixres_);
		int u1 = nx - (int)((waypoints[i+1].y - ll_.y) / pixres_);
		int v1 = ny - (int)((waypoints[i+1].x - ll_.x) / pixres_);
		if (!(u0 == u1 && v0 == v1)) {
			cropped_image_.draw_line(u0, v0, u1, v1, (float *)&yellow);
		}
	}

}

void OrthoViewer::SaveImage(std::string fname) {
	cropped_image_.save(fname.c_str());
}

}//namespace mavs

