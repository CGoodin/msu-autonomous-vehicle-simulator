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
#include <mavs_core/math/polygon.h>

#include <iostream>
#include <limits>
#include <algorithm>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace math{

Polygon::Polygon(){

}

Polygon::Polygon(std::vector<glm::vec2> points){
  polygon_ = points;
  llx_ = std::numeric_limits<float>::max();
  lly_ = llx_;
  urx_ = std::numeric_limits<float>::min();
  ury_ = urx_;
  for (int i=0;i<(int)polygon_.size();i++){
    if (polygon_[i].x<llx_)llx_=polygon_[i].x;
    if (polygon_[i].x>urx_)urx_=polygon_[i].x;
    if (polygon_[i].y<lly_)lly_=polygon_[i].y;
    if (polygon_[i].y>ury_)ury_=polygon_[i].y;
  }
}

Polygon::~Polygon(){

}

bool Polygon::OnSegment(glm::vec2 p, glm::vec2 q, glm::vec2 r){
  if (q.x<=std::max(p.x,r.x) && q.x>= std::min(p.x,r.x) &&
      q.y<=std::max(p.y,r.y) && q.y>= std::min(p.y,r.y) ){
    return true;
  }
  else{
    return false;
  }
}

bool Polygon::SegmentIntersect(glm::vec2 p1, glm::vec2 q1, 
			       glm::vec2 p2, glm::vec2 q2){
  int o1 = Orientation(p1,q1,p2);
  int o2 = Orientation(p1,q1,q2);
  int o3 = Orientation(p2,q2,p1);
  int o4 = Orientation(p2,q2,q1);
  if (o1!=o2 && o3!=o4) return true;

  if (o1==0 && OnSegment(p1,p2,q1)) return true;
  if (o2==0 && OnSegment(p1,q2,q1)) return true;
  if (o3==0 && OnSegment(p2,p1,q2)) return true;
  if (o4==0 && OnSegment(p2,q1,q2)) return true;

  return false;
}

int Polygon::Orientation(glm::vec2 p, glm::vec2 q, glm::vec2 r){
  float val = (q.y-p.y)*(r.x-q.x)-(q.x-p.x)*(r.y-q.y);

  if (val==0) return 0;
  return (val>0)? 1: 2;
}

 bool Polygon::IsInside(glm::vec2 p){
  size_t n = polygon_.size();
  if (n<3) return false;
  float inf = std::numeric_limits<float>::max();
  glm::vec2 extreme(inf,p.y);
  int count = 0;
  int i = 0;
  for (int i=0;i<(int)polygon_.size();i++){
    int next = (i+1)%n;
    if (SegmentIntersect(polygon_[i],polygon_[next],p,extreme)){
      if (Orientation(polygon_[i],p,polygon_[next])==0){
	return OnSegment(polygon_[i],p,polygon_[next]);
      }
      count++;
    }
  }
  return (count%2==1);
}

glm::vec2 Polygon::GetRandomInside(){
  glm::vec2 point;
  int loop_counter = 0;
  while (true) {
    float px = rand_in_range(llx_,urx_);
    float py = rand_in_range(lly_,ury_);
    glm::vec2 testpoint(px,py);
    if (IsInside(testpoint)){
      point = testpoint;
      break;
    }
    loop_counter++;
  }
  return point;
}

// https://www.mathopenref.com/coordpolygonarea.html
float Polygon::GetArea() {
	float area = 0.0f;
	for (int i = 0; i < (polygon_.size()-1); i++) {
		area += (polygon_[i].x*polygon_[i + 1].y - polygon_[i].y*polygon_[i + 1].x);
	}
	area += (polygon_.back().x*polygon_[0].y - polygon_.back().y*polygon_[0].x);
	area = 0.5f*area;
	area = (float)fabs(area);
	return area;
}

} //namespace math
} //namespace mavs
