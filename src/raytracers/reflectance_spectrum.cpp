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
#include <raytracers/reflectance_spectrum.h>
#include <fstream>
#include <algorithm> 
#include <mavs_core/math/utils.h>

namespace mavs{

ReflectanceSpectrum::ReflectanceSpectrum() {
	wavelength_.resize(0);
	reflectance_.resize(0);
}

void ReflectanceSpectrum::Load(std::string infile) {
	if (!mavs::utils::file_exists(infile)) {
		std::cerr << "ERROR: Spectrum file " << infile << " does not exist." << std::endl;
	}
	std::ifstream fin(infile);
	if (fin.is_open()) {
		while (!fin.eof()) {
			float w, r;
			fin >> w >> r;
			wavelength_.push_back(w);
			reflectance_.push_back(r);
		}
		wavelength_.pop_back();
		reflectance_.pop_back();

		// entries may be backwards, try reversing
		if (wavelength_[1] < wavelength_[0]) {
			std::reverse(wavelength_.begin(), wavelength_.end());
			std::reverse(reflectance_.begin(), reflectance_.end());
		}

		for (int i = 0; i < (wavelength_.size()-1); i++) {
			if (wavelength_[i + 1] <= wavelength_[i]) {
				std::cerr << "WARNING: " << infile << " may not be valid." << std::endl;
				std::cerr << "Wavelengths should be ordered from shortest to longest." << std::endl;
				break;
			}
		}
	}
	else {
		std::cerr << "ERROR: Failed to open spectrum file " << infile << std::endl;
	}
	fin.close();
	name_ = infile;
	mavs::utils::EraseSubString(name_, mavs::utils::GetPathFromFile(infile));
	mavs::utils::EraseSubString(name_, mavs::utils::GetFileExtension(infile));
	name_.pop_back();
	name_.erase(0, 1);
}

float ReflectanceSpectrum::GetReflectanceAtWavelength(float wl) {
	//std::cout<<"Wavelengthg = "<<wl<<" "<<repeat_vals_.count(wl)<<std::endl;
	//if (repeat_vals_.count(wl)==0){ // this is not thread safe 
		if (wavelength_.size() <= 0 || wavelength_.size() != reflectance_.size()) {
			return 0.0f;
		}
		else {
			if (wl<=wavelength_[0]){
				return reflectance_[0];
			}
			else if (wl >= wavelength_.back()) {
				return reflectance_.back();
			}
			else {
				for (int i = 0; i < (wavelength_.size() - 1); i++) {
					if (wl < wavelength_[i+1]) {
						float y0 = reflectance_[i];
						float y1 = reflectance_[i + 1];
						float x0 = wavelength_[i];
						float x1 = wavelength_[i + 1];
						float rho = (y0*(x1 - wl) + y1 * (wl - x0)) / (x1 - x0);
						//repeat_vals_[wl] = rho;
						return rho;
					}
				}
			}
		}
	//}
	//else {
	//	return repeat_vals_[wl];
	//}
	return 0.0f;
} // GetReflectanceAtWavelength

} // namespace mavs


