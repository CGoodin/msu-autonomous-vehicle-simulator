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
#include <raytracers/texture_layers/texture_layer.h>

namespace mavs {
namespace raytracer {

TextureLayer::TextureLayer() {

}
//See: 
//https://squircleart.github.io/shading/normal-map-generation.html
void TextureLayer::CreateNormalMap() {
	// create a grayscale version of the image
	cimg_library::CImg<float> grayscale;
	grayscale.assign(image_.width(), image_.height(), 1, 1);
	for (int i = 0; i < image_.width(); i++) {
		for (int j = 0; j < image_.height(); j++) {
			float gray = (image_(i, j, 0, 0) + image_(i, j, 0, 1) + image_(i, j, 0, 2)) / 255.0f;
			gray = gray / 3.0f;
			grayscale(i, j, 0) = gray;
		}
	}

	normals_.assign(image_.width(), image_.height(), 1, 3);
	for (int i = 0; i < image_.width(); i++) {
		for (int j = 0; j < image_.height(); j++) {
			int ilo = i - 1;
			if (ilo == -1)ilo = image_.width() - 1;
			int ihi = i + 1;
			if (ihi == image_.width())ihi = 0;
			int jlo = j - 1;
			if (jlo == -1)jlo = image_.height() - 1;
			int jhi = j + 1;
			if (jhi == image_.height())jhi = 0;
			float dfdx = grayscale(ihi, j, 0) - grayscale(ilo, j, 0);
			float dfdy = grayscale(i, jhi, 0) - grayscale(i, jlo, 0);
			float norm = sqrt(dfdx*dfdx + dfdy * dfdy + 1.0f);
			normals_(i, j, 0, 0) = -dfdx / norm;
			normals_(i, j, 0, 1) = -dfdy / norm;
			normals_(i, j, 0, 2) = 1.0f / norm;
			//normals_(i, j, 0, 0) = 255.0f*0.5f*(normals_(i, j, 0, 0) + 1.0);
			//normals_(i, j, 0, 1) = 255.0f*0.5f*(normals_(i, j, 0, 1) + 1.0);
			//normals_(i, j, 0, 2) = 255.0f*(normals_(i, j, 0, 2));
		}
	}
	//normals_.save("normalmap.bmp");
}

void TextureLayer::LoadImage(std::string imagefile) {
	//load the image
	image_.load(imagefile.c_str());
	CreateNormalMap();
}

} //namespace raytracer 
} //namespace mavs