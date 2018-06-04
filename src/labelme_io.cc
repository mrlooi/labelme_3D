#include "labelme_io.hh"

#include <iostream>
#include <fstream>

#include <json.hpp> 

using namespace nlohmann;

namespace labelme
{

static inline bool keyExists(json& j, const std::string key) 
{
    return j[key] != nullptr;
}

static bool LoadDataFromJson(json& j, const std::string json_file)
{
    bool rt = false;

    std::ifstream f(json_file);
    if (f.is_open())
    {
        try {
            j << f;
            rt = true;
        }
        catch (const std::invalid_argument& ia)
        {
            std::cerr << "INVALID JSON in " << json_file << std::endl;
        } catch (json::parse_error &e)
        {
            std::cerr << e.what() << std::endl;
            std::cerr << "FAILED to parse " << json_file << std::endl;
        }
        f.close();
    }
    else
    {
        std::cerr << "FAILED TO LOAD JSON CONFIG FILE. " << json_file << " NOT FOUND.\n";
    }

    return rt;
}	

bool read_data(const std::string json_file, LabelMeData& data)
{
	json j;
	bool rt = LoadDataFromJson(j, json_file);
	if (!rt)
	{
		std::cout << "Failed to read " << json_file << std::endl;
		return false;
	}

	if (keyExists(j,"data"))
	{
		json& j_data = j["data"]; // N * 4
		const std::vector<std::vector<int>>& v_data = j_data.get<std::vector<std::vector<int>>>();

		data.data.resize(v_data.size()); 
		for (int i = 0; i < v_data.size(); ++i)
		{
			const auto& d_in = v_data[i];
			if (d_in.size() == 4)
			{
				auto& d_out = data.data[i];
				d_out[0] = d_in[0];
				d_out[1] = d_in[1];
				d_out[2] = d_in[2];
				d_out[3] = d_in[3];
			} else {
				printf("data %d is not size 4, skipping\n", i);
			}
		}
	} else {
		printf("NO data FIELD\n");
	}

	if (keyExists(j,"colors"))
	{
		json& j_colors = j["colors"]; // N * 4
		const std::vector<std::vector<int>>& colors = j_colors.get<std::vector<std::vector<int>>>();

		data.colors.resize(colors.size()); 
		for (int i = 0; i < colors.size(); ++i)
		{
			const auto& c_in = colors[i];
			if (c_in.size() == 3)
			{
				auto& c_out = data.colors[i];
				c_out[0] = c_in[0];
				c_out[1] = c_in[1];
				c_out[2] = c_in[2];
			} else {
				printf("color %d is not size 3, skipping\n", i);
			}
		}
	} else {
		printf("NO colors FIELD\n");
	}

	if (keyExists(j,"poses"))
	{
		json& j_poses = j["poses"]; // N * 4
		const std::vector<std::vector<float>>& poses = j_poses.get<std::vector<std::vector<float>>>();

		data.poses.resize(poses.size()); 
		for (int i = 0; i < poses.size(); ++i)
		{
			const auto& p = poses[i];
			if (p.size() == 16)
			{
				Eigen::Matrix4f& viewer_pose = data.poses[i]; 
				for (int j = 0; j < 4; ++j)
				{
					viewer_pose(j,0) = p[j*4];
					viewer_pose(j,1) = p[j*4+1];
					viewer_pose(j,2) = p[j*4+2];
					viewer_pose(j,3) = p[j*4+3];
				}
			} else {
				printf("pose %d is not size 16, skipping\n", i);
			}
		}
	} else {
		printf("NO poses FIELD\n");
	}
	
	if (keyExists(j,"pcd"))
	{
		data.pcd_file = j["pcd"];
	} else {
		printf("NO pcd FIELD\n");
	}

	return true;
}

bool save_data(const std::string json_file, const LabelMeData& data)
{
	std::ofstream file(json_file);
	if (file.is_open()) 
	{
		nlohmann::json j;
		// save original cloud file name
		j["pcd"] = data.pcd_file;

		// save annotation colors
		size_t total_colors = data.colors.size();
		std::vector<std::vector<int>> colors(total_colors);
		for (size_t i = 0; i < total_colors; ++i)
		{
			const auto& colr = data.colors[i];
			colors[i] = {colr[0], colr[1], colr[2]}; // b g r
		}
		j["colors"] = colors;

		// save all remaining indices + color for each
		size_t total_idx = data.data.size();
		std::vector<std::vector<int>> v_data(total_idx);
		for (size_t i = 0; i < total_idx; ++i)
		{
			const auto& d = data.data[i];
			v_data[i] = {d[0], d[1], d[2], d[3]};  // idx bgr
		}
		j["data"] = v_data;


		// save
		file << j;
		// printf("Saved data to %s\n", json_file.c_str());
		file.close();

		return true;
	} else {
		// printf("FAILED to open %s\n", json_file.c_str());
	}

	return false;
}

}
