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
#include <mavs_core/pose_readers/anvel_vprp_reader.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <mavs_core/math/utils.h>

namespace mavs{
  
static std::vector<std::string> GetWords(std::string line){
  std::vector<std::string> words;
  std::istringstream sline(line);
  std::string word;
  while(sline>>word){
    std::istringstream subline(word);
    std::string subword;
    while(std::getline(subline,subword,':')){
      words.push_back(subword);
    }
  }
  return words;
}

std::vector<mavs::Pose> AnvelVprpReader::Load(std::string fname,
						int pose_freq){
  std::ifstream fin(fname.c_str());
  std::string line;
  bool data_read = false;

  int lines_read=0;
  while(std::getline(fin,line)){
    std::vector<std::string> words = GetWords(line);
    if (words[0]=="Vehicle Objects"){
      num_vehicle_objects_ = utils::StringToInt(words[1]);
    }
    if (words[0]=="@Data"){
      data_read = true;
    }
    if (data_read && words.size()==8){
      if (lines_read%pose_freq==0){
	mavs::Pose p;
	for (int i=0;i<(int)words.size();i++){
	  p.position.x = utils::StringToDouble(words[1]);
	  p.position.y = utils::StringToDouble(words[2]);
	  p.position.z = utils::StringToDouble(words[3]);
	  p.quaternion.w = utils::StringToDouble(words[4]);
	  p.quaternion.x = utils::StringToDouble(words[5]);
	  p.quaternion.y = utils::StringToDouble(words[6]);
	  p.quaternion.z = utils::StringToDouble(words[7]);
	}
	poses_.push_back(p);
      }
      lines_read++;
    }
  }
  fin.close();
  return poses_;
}

} //namespace mavs
