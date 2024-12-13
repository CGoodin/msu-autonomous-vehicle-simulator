/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <mavs_core/coordinate_systems/ellipsoid.h>

namespace mavs{
namespace coordinate_system{

static std::vector<Ellipsoid> ellipsoid_;

Ellipsoid::Ellipsoid(){
  Init();
};
  
Ellipsoid::Ellipsoid(int Id, std::string name, double radius, double ecc){
  id_ = Id; 
  ellipsoid_name_ = name; 
  equatorial_radius_ = radius; 
  eccentricity_squared_ = ecc;
}

void Ellipsoid::Init(){
  ellipsoid_.push_back(Ellipsoid( -1, "Placeholder", 0, 0)); 
  ellipsoid_.push_back(Ellipsoid( 1, "Airy", 6377563, 0.00667054));
  ellipsoid_.push_back(Ellipsoid( 2, "Australian National", 6378160, 
				  0.006694542));
  ellipsoid_.push_back(Ellipsoid( 3, "Bessel 1841", 6377397, 0.006674372));
  ellipsoid_.push_back(Ellipsoid( 4, "Bessel 1841 (Nambia) ", 6377484, 
				  0.006674372));
  ellipsoid_.push_back(Ellipsoid( 5, "Clarke 1866", 6378206, 0.006768658));
  ellipsoid_.push_back(Ellipsoid( 6, "Clarke 1880", 6378249, 0.006803511));
  ellipsoid_.push_back(Ellipsoid( 7, "Everest", 6377276, 0.006637847));
  ellipsoid_.push_back(Ellipsoid( 8, "Fischer 1960 (Mercury) ", 6378166, 
				  0.006693422));
  ellipsoid_.push_back(Ellipsoid( 9, "Fischer 1968", 6378150, 0.006693422));
  ellipsoid_.push_back(Ellipsoid( 10, "GRS 1967", 6378160, 0.006694605));
  ellipsoid_.push_back(Ellipsoid( 11, "GRS 1980", 6378137, 0.00669438));
  ellipsoid_.push_back(Ellipsoid( 12, "Helmert 1906", 6378200, 0.006693422));
  ellipsoid_.push_back(Ellipsoid( 13, "Hough", 6378270, 0.00672267));
  ellipsoid_.push_back(Ellipsoid( 14, "International", 6378388, 0.00672267));
  ellipsoid_.push_back(Ellipsoid( 15, "Krassovsky", 6378245, 0.006693422));
  ellipsoid_.push_back(Ellipsoid( 16, "Modified Airy", 6377340, 0.00667054));
  ellipsoid_.push_back(Ellipsoid( 17, "Modified Everest", 6377304, 
				  0.006637847));
  ellipsoid_.push_back(Ellipsoid( 18, "Modified Fischer 1960", 6378155, 
				  0.006693422));
  ellipsoid_.push_back(Ellipsoid( 19, "South American 1969", 6378160, 
				  0.006694542));
  ellipsoid_.push_back(Ellipsoid( 20, "WGS 60", 6378165, 0.006693422));
  ellipsoid_.push_back(Ellipsoid( 21, "WGS 66", 6378145, 0.006694542));
  ellipsoid_.push_back(Ellipsoid( 22, "WGS-72", 6378135, 0.006694318));
  ellipsoid_.push_back(Ellipsoid( 23, "WGS-84", 6378137, 0.00669438));
}
  
double Ellipsoid::GetEquatorialRadius(int refnum ){
  if (refnum<1 || refnum>23) refnum=23;
  return ellipsoid_[refnum].GetEquatorialRadius();
}
  
double Ellipsoid::GetEccentrictySquared(int refnum){
  if (refnum<1 || refnum>23) refnum=23;
  return ellipsoid_[refnum].GetEccentricitySquared();
}

} //namespace environment
} //namespace mavs
