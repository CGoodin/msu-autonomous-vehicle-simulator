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
#include <sensors/object_detector/object_detector.h>

namespace mavs{
namespace sensor {

ObjectDetector::ObjectDetector() {
	SetScanProperties(-180.0f, 180.0f, 0.25f);
	SetMaxRange(30.0f);
	SetName("Object_Detector");
}

void ObjectDetector::Update(environment::Environment *env, double dt) {
	lidar::PlanarLidar::Update(env, dt);
	AnnotateFrame(env, false);
}

std::vector<Obstacle> ObjectDetector::GetObstacles() {
	std::vector<Obstacle> obstacles;
	std::map<int, mavs::sensor::Annotation> anno = GetObjectAnnotations();
	std::map<int, mavs::sensor::Annotation>::iterator it;
	float thresh = 1.0E12f;
	for (it = anno.begin(); it != anno.end(); it++) {
		glm::vec3 ll = it->second.GetLLCorner();
		glm::vec3 ur = it->second.GetURCorner();
		if (ll.x<thresh && ll.x>-thresh && ll.y<thresh && ll.y>-thresh &&
			ur.x<thresh && ur.x>-thresh && ur.y<thresh && ur.y>-thresh) {
			glm::vec3 center = 0.5f*(ll + ur);
			glm::vec3 size = ur - ll;
			Obstacle obs;
			obs.x = center.x;
			obs.y = center.y;
			obs.height = ur.z;
			obs.radius = 0.5f*(float)sqrt(size.x*size.x + size.y*size.y);
			if (obs.radius>0.0f)obstacles.push_back(obs);
		}
	}
	return obstacles;
}

} //namespace sensor
} //namespace mavs
