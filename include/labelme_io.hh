#ifndef LABELME_IO_HH
#define LABELME_IO_HH

#include <string>
#include <vector>       // std::vector

#include <Eigen/Dense>

namespace labelme
{

struct LabelMeData
{
	std::vector<Eigen::Vector3i> colors; // available annotation colors
	std::vector<Eigen::Vector4i> data;   // each representing: idx of original cloud, color_b, color_g, color_r
	std::vector<Eigen::Matrix4f> poses;  // 
	std::string pcd_file;
};

bool read_data(const std::string json_file, LabelMeData& data);
bool save_data(const std::string json_file, const LabelMeData& data);

}

#endif