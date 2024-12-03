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
#include <sensors/io/user_io.h>

#include <algorithm>
#include <iostream>

#include <mavs_core/data_path.h>
#include <mavs_core/math/utils.h>
#include <tinyfiledialogs.h>


namespace mavs {
namespace io {

std::string GetInputFileName(std::string message, std::string ftype) {
	mavs::MavsDataPath data_path;
	std::string mavs_data_path = data_path.GetPath();
	char const * lTheOpenFileName;
	char const * lFilterPatterns[1] = { ftype.c_str() };
	lTheOpenFileName = tinyfd_openFileDialog(
		message.c_str(),
		mavs_data_path.c_str(),
		1,
		lFilterPatterns,
		NULL,
		0);
	if (lTheOpenFileName) {
		std::string obj_file(lTheOpenFileName);
		std::replace(obj_file.begin(), obj_file.end(), '\\', '/');
		return obj_file;
	}
	else {
		std::cerr << "File not found" << std::endl;
		std::string obj_file("");
		return obj_file;
	}
}

std::string GetSaveFileName(std::string message, std::string default_type, std::string ftype) {
	std::string save_file;
	char const * lTheSaveFileName;
	char const * lFilterPatterns[1] = { ftype.c_str() };
	lTheSaveFileName = tinyfd_saveFileDialog(
		message.c_str(),
		default_type.c_str(),
		1,
		lFilterPatterns,
		NULL);
	if (lTheSaveFileName) {
		std::string save_file(lTheSaveFileName);
		std::replace(save_file.begin(), save_file.end(), '\\', '/');
		return save_file;
	}
	else {
		std::cerr << "Save file not selected" << std::endl;
		std::string save_file("");
		return save_file;
	}
}

float GetUserNumericInput(std::string title, std::string message) {
	float val = 0.0f;
	char const * lTmp;
	lTmp = tinyfd_inputBox(title.c_str(), message.c_str(), "0.0");
	if (lTmp) {
		std::string tmp(lTmp);
		val = (float)mavs::utils::StringToDouble(tmp);
	}
	return val;
}

bool GetUserBool(std::string question, std::string message) {
	int answer = tinyfd_messageBox(question.c_str(),
		message.c_str(),
		"yesno", "question", 0);
	if (answer == 0) {
		return false;
	}
	else {
		return true;
	}
}

void DisplayUserInfo(std::string title, std::string message) {
	tinyfd_messageBox(title.c_str(), message.c_str(), "ok", "info", 1);
}

} //namespace io
} //namespace mavs