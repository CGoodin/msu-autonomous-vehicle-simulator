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
* \file fix_texture.cpp
*
* Utility to fix a texture map that is tiled incorrectly
*
* Usage: >./fix_texture path_to_mtl full_path_to_mesh.obj
*
* path_to_mtl is a file path, without the file, to where the .mtl 
* file is located. Full_path_to_mesh.obj is the full path to the 
* obj file to manipulate.
*
* \author Chris Goodin
*
* \date 8/8/2022
*/
#include <raytracers/mesh.h>

int main(int argc, char * argv[]){

  if (argc<3){
    std::cerr<<"ERROR, must provide the path to the mesh file and the mesh "<<
      "file name as command line input "<<std::endl;
    std::cerr<<"Usage: ./fix_texture /path/to/mesh "<<
      "/path/to/mesh/meshfile.obj"<<std::endl;
    return 1;
  }

  std::string path(argv[1]);
  std::string fname(argv[2]);

  std::string outname = "fixed";
  if (argc > 3) {
	  outname = std::string(argv[3]);
  }

  float x0 = 0.0f;
  float x1 = 1.0f;
  float y0 = 0.0f;
  float y1 = 1.0f;
  if (argc > 7) {
	  x0 = atof(argv[4]);
	  x1 = atof(argv[5]);
	  y0 = atof(argv[6]);
	  y1 = atof(argv[7]);
  }

  float theta = 0.0f;
  if (argc > 8) {
	  theta = acosf(-1.0f)*atof(argv[8])/180.0f;
  }


  mavs::raytracer::Mesh mesh;
  mesh.Load(path,fname);

	mavs::raytracer::BoundingBox bb = mesh.GetBoundingBox();
	
	glm::vec3 llc = bb.GetLowerLeft();
	glm::vec3 urc = bb.GetUpperRight();
	
	float dx = urc.x - llc.x;
	float dy = urc.y - llc.y;
	mesh.ClearTexCoords();
	float ct = cosf(theta);
	float st = sinf(theta);
	std::cout << "Looping through " << mesh.GetNumFaces() << " faces, " << dx << " " << dy << " "<<ct<<" "<<st<<std::endl;
	for (int i = 0; i < mesh.GetNumVertices(); i++) {
		glm::vec3 vtx = mesh.GetVertex(i);
		float u0 = (vtx.x - llc.x) / dx;
		float v0 = (vtx.y - llc.y) / dy;
		u0 = x0 + u0 * (x1 - x0);
		v0 = y0 + v0 * (y1 - y0);
		float u = ct * u0 - st * v0;
		float v = st * u0 + ct * v0;
		int t = mesh.AddTexCoord(u, v);
	}
	for (int i = 0; i < mesh.GetNumFaces(); i++) {
		mavs::raytracer::Face face = mesh.GetFace(i);
		mesh.SetFaceTextureIndex(i, face.v1, face.v2, face.v3);
	}
	std::cout << "Done fixing, writing to " << outname << ".obj" << std::endl;

	mesh.Write(outname);

  return 0;
}
