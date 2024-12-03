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
* \file mesh_manipulator.cpp
*
* Quickly scale, rotate, and re-center obj meshes
*
* Usage: >./mesh_manipulator path_to_mtl full_path_to_mesh.obj
*
* path_to_mtl is a file path, without the file, to where the .mtl 
* file is located. Full_path_to_mesh.obj is the full path to the 
* obj file to manipulate.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <raytracers/mesh.h>

int main(int argc, char * argv[]){

  if (argc!=3){
    std::cerr<<"ERROR, must provide the path to the mesh file and the mesh "<<
      "file name as command line input "<<std::endl;
    std::cerr<<"Usage: ./mesh_manipulator /path/to/mesh "<<
      "/path/to/mesh/meshfile.obj"<<std::endl;
    return 1;
  }

  std::string path(argv[1]);
  std::string fname(argv[2]);
  mavs::raytracer::Mesh mesh;
  mesh.Load(path,fname);

  glm::vec3 size, center;
  size = mesh.GetSize();
  center = mesh.GetCenter();
  std::cout<<"The mesh size is "<<size.x<<" x "<<size.y<<" x "<<size.z
	   <<std::endl;
  std::cout<<"The mesh center is "<<center.x<<", "<<center.y<<", "<<center.z
	   <<std::endl; 
  std::cout<<"The number of triangles is "<<mesh.GetNumFaces()<<std::endl;
  char y_to_z, scale, translate, new_file;
  std::cout<<"Rotate Y to Z (y or n): ";
  std::cin>>y_to_z;
  if (y_to_z=='y'){
    mesh.RotateYToZ();
  }

  std::cout<<"Scale (y or n): ";
  std::cin>>scale;
  if (scale=='y'){
    float sx, sy, sz;
    std::cout<<"Enter the scaling in the x, y, and z direction: ";
    std::cin>>sx>>sy>>sz;
    mesh.Scale(sx,sy,sz);
  }

  std::cout<<"Translate (y or n): ";
  std::cin>>translate;
  if (translate=='y'){
    glm::vec3 t;
    std::cout<<"Enter the translation vector in x, y, and z direction: ";
    std::cin>>t.x>>t.y>>t.z;
    mesh.Translate(t);
  }

  std::cout<<"Save mesh to new file (y or n): ";
  std::cin>>new_file;
  if (new_file=='y'){
    std::cout<<"Enter new file name, w/o extension: ";
    std::string ofname;
    std::cin>>ofname;
    mesh.Write(ofname);
  }

  return 0;
}
