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
#include <sensors/io/json_io.h>
#include <sensors/mavs_sensors.h>

namespace mavs {
namespace io {

sensor::Sensor *LoadSensor(const rapidjson::Value &sens) {
	sensor::Sensor *to_add;

	glm::vec3 offset(0.0, 0.0, 0.0);
	if (sens.HasMember("Offset")) {
		offset.x = sens["Offset"][0].GetFloat();
		offset.y = sens["Offset"][1].GetFloat();
		offset.z = sens["Offset"][2].GetFloat();
	}
	glm::quat relor(1.0, 0.0, 0.0, 0.0);
	if (sens.HasMember("Orientation")) {
		relor.w = sens["Orientation"][0].GetFloat();
		relor.x = sens["Orientation"][1].GetFloat();
		relor.y = sens["Orientation"][2].GetFloat();
		relor.z = sens["Orientation"][3].GetFloat();
	}
	float rep_rate = 10.0f;
	if (sens.HasMember("Repitition Rate (Hz)")) {
		rep_rate = sens["Repitition Rate (Hz)"].GetFloat();
	}
	int nprocs = 1;
	if (sens.HasMember("Number Processors")) {
		nprocs = sens["Number Processors"].GetInt();
	}

	bool draw_anno = false;
	if (sens.HasMember("Draw Annotations")) {
		draw_anno = sens["Draw Annotations"].GetBool();
	}

	std::string name = "Sensor";
	if (sens.HasMember("Name")) {
		name = sens["Name"].GetString();
	}
	std::string type = sens["Type"].GetString();

	if (type == "lidar") {
		if (sens.HasMember("Model")) {
			std::string model = sens["Model"].GetString();
			if (model == "M8") {
				to_add = new sensor::lidar::MEight(rep_rate);
			}
			else if (model == "HDL-64E") {
				to_add = new sensor::lidar::Hdl64E(rep_rate);
			}
			else if (model == "HDL-32E") {
				to_add = new sensor::lidar::Hdl32E;
			}
			else if (model == "VLP-16") {
				to_add = new sensor::lidar::Vlp16(rep_rate);
			}
			else if (model == "LMS-291") {
				to_add = new sensor::lidar::Lms291_S05;
			}
			else {
				std::cerr << model << " is not a valid LIDAR model." << std::endl <<
					"It will not be added to the simulation." << std::endl;
			}
		}
		else {
			to_add = new sensor::lidar::Lidar;
			std::string lidar_file = sens["Input File"].GetString();
			to_add->Load(lidar_file);
		}
	}
	else if (type == "compass") {
		to_add = new sensor::compass::Compass;
		std::string compass_file = sens["Input File"].GetString();
		//to_add->Load(compass_file);
	}
	else if (type == "radar") {
		if (sens.HasMember("Model")) {
			std::string model = sens["Model"].GetString();
			if (model == "Delphi Long Range") {
				to_add = new sensor::radar::DelphiLongRange;
			}
			else if (model == "Delphi Mid Range") {
				to_add = new sensor::radar::DelphiMidRange;
			}
			else {
				std::cerr << model << " is not a valid RADAR model." << std::endl <<
					"It will not be added to the simulation." << std::endl;
			}
		}
		else {
			to_add = new sensor::radar::Radar;
			std::string radar_file = sens["Input File"].GetString();
			to_add->Load(radar_file);
		}
	}
	else if (type == "fisheye") {
		to_add = new sensor::camera::FisheyeCamera;
		sensor::Sensor *fisheye = new sensor::camera::FisheyeCamera;
		std::string fisheye_file = sens["Input File"].GetString();
		to_add->Load(fisheye_file);
	}
	else if (type == "camera") {
		if (sens.HasMember("Model")) {
			std::string model = sens["Model"].GetString();
			if (model == "Flea3-4mm") {
				to_add = new sensor::camera::Flea3_4mm;
			}
			else if (model == "XCD-V60") {
				to_add = new sensor::camera::XCD_V60;
			}
			else {
				std::cerr << model << " is not a valid camera model." << std::endl <<
					"It will not be added to the simulation." << std::endl;
			}
		}
		else {
			to_add = new sensor::camera::RgbCamera;
			std::string cam_file = sens["Input File"].GetString();
			to_add->Load(cam_file);
		}
	}
	else if (type == "gps") {
		to_add = new sensor::gps::Gps;
	} //sensor types
	else if (type == "imu") {
		to_add = new sensor::imu::Imu;
		std::string imu_file = sens["Input File"].GetString();
		to_add->Load(imu_file);
	}
	else {
		//sensor type not recognized
	} // no sensor type
	to_add->SetTimeStep(1.0 / rep_rate);
	to_add->SetRelativePose(offset, relor);
	to_add->SetName(name);
	to_add->SetNumProcs(nprocs);
	if (draw_anno)to_add->SetDrawAnnotations();

	return to_add;
} // Load sensor

void LoadSensorBlock(const rapidjson::Document& d,
	std::vector<sensor::Sensor*> &sensors) {

	if (d.HasMember("Sensors")) {
		int num_sensors = d["Sensors"].Capacity();
		for (int i = 0; i < num_sensors; i++) {
			const rapidjson::Value& sensobj = d["Sensors"][i];
			sensor::Sensor *to_add = LoadSensor(sensobj);
			sensors.push_back(to_add);
		} // for i < num_sensors
	} // if has member sensors
	else if (d.HasMember("Sensor")) {
		const rapidjson::Value& sensobj = d["Sensor"];
		sensor::Sensor *to_add = LoadSensor(sensobj);
		sensors.push_back(to_add);
	}
	else {
		std::cerr << "ERROR: No sensor block was listed." << std::endl;
	}
}

} //namespace mavs
} //namespace io