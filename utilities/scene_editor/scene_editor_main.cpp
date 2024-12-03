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
* \file scene_editor.cpp
*
* Add vegetation to a surface using polygons
*
* Usage: > scene_editor
*
* \author Chris Goodin
*
* \date 2/28/2019
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#include "scene_editor.h"
#include <sensors/io/user_io.h>

int main(int argc, char *argv[]) {
	std::string surface_file = mavs::io::GetInputFileName("Select surface mesh", "*.obj");
	if (surface_file.size() <= 4) return 0;

	mavs::utils::SceneEditor scene_editor;
	scene_editor.LoadSurface(surface_file);
	scene_editor.RunEditLoop();
	return 0;
}
