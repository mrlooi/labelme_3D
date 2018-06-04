
#ifndef VISION_PCL_VIEWER_UTILS_HH
#define VISION_PCL_VIEWER_UTILS_HH

#include <time.h>

#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui.hpp>


namespace PclViewerUtils
{

const static Eigen::Vector3i BLUE {0,0,255};
const static Eigen::Vector3i GREEN {0,255,0};
const static Eigen::Vector3i RED {255,0,0};

static inline Eigen::Vector3i get_random_color()
{
  srand(time(0));
  cv::RNG cv_rng(rand());

	Eigen::Vector3i color {cv_rng.uniform(0, 255), cv_rng.uniform(0,255), cv_rng.uniform(0,255)};
	return color;
}

template <typename T>
static inline void set_diff(std::vector<T>& result, const std::vector<T>& first, const std::vector<T>& second, bool sorted=true)
{
    if (!sorted)
    {
        auto sorted_first = first;
        auto sorted_second = second;
        std::sort(sorted_first.begin(), sorted_first.end());
        std::sort(sorted_second.begin(), sorted_second.end());
        std::set_difference(sorted_first.begin(),sorted_first.end(),sorted_second.begin(),sorted_second.end(),std::inserter(result, result.end()));
    } else {
        std::set_difference(first.begin(),first.end(),second.begin(),second.end(),std::inserter(result, result.end()));
    }
}

	
static int id_ = 0;
template <typename Container>
static void draw_box_cloud_lines(pcl::visualization::PCLVisualizer::Ptr& viewer, const Container& box_points, const Eigen::Vector3i colori = RED)
{
	int num_pts = box_points.size();
	Eigen::Vector3f color {float(colori[0])/255, float(colori[1])/255, float(colori[2])/255};

	std::string id_str = std::to_string(id_);
	if (num_pts == 8)
	{
	    viewer->addLine(box_points[0], box_points[1], color[0], color[1], color[2], "A" + id_str);
	    viewer->addLine(box_points[1], box_points[2], color[0], color[1], color[2], "B" + id_str);
	    viewer->addLine(box_points[2], box_points[3], color[0], color[1], color[2], "C" + id_str);
	    viewer->addLine(box_points[3], box_points[0], color[0], color[1], color[2], "D" + id_str);
	    viewer->addLine(box_points[4], box_points[5], color[0], color[1], color[2], "E" + id_str);
	    viewer->addLine(box_points[5], box_points[6], color[0], color[1], color[2], "F" + id_str);
	    viewer->addLine(box_points[6], box_points[7], color[0], color[1], color[2], "G" + id_str);
	    viewer->addLine(box_points[7], box_points[4], color[0], color[1], color[2], "H" + id_str);
	    viewer->addLine(box_points[0], box_points[4], color[0], color[1], color[2], "I" + id_str);
	    viewer->addLine(box_points[1], box_points[5], color[0], color[1], color[2], "J" + id_str);
	    viewer->addLine(box_points[2], box_points[6], color[0], color[1], color[2], "K" + id_str);
	    viewer->addLine(box_points[3], box_points[7], color[0], color[1], color[2], "L" + id_str);
	    // printf("Color: %d %d %d\n", color[0], color[1], color[2]);
	} 
	else if (num_pts == 4)
	{
		viewer->addLine(box_points[0], box_points[1], color[0], color[1], color[2], "A" + id_str);
		viewer->addLine(box_points[1], box_points[2], color[0], color[1], color[2], "B" + id_str);
		viewer->addLine(box_points[2], box_points[3], color[0], color[1], color[2], "C" + id_str);
		viewer->addLine(box_points[3], box_points[0], color[0], color[1], color[2], "D" + id_str);
	}
  ++id_;
}

template <typename PointT>
static void color_cloud_points(pcl::PointCloud<PointT>& cloud, const std::vector<int>& indices, const Eigen::Vector3i color)
{
	// Eigen::Vector3f color {float(colori[0])/255, float(colori[1])/255, float(colori[2])/255};
    for (int i = 0; i < indices.size(); ++i)
    {
    	PointT& pt = cloud.points[indices[i]];
    	pt.b = color[0];
    	pt.g = color[1];
    	pt.r = color[2];
    	
    }
}

template <typename PointT>
static std::vector<Eigen::Vector3i> get_indices_colored_cloud(const pcl::PointCloud<PointT>& cloud, const std::vector<std::vector<int>>& indices, pcl::PointCloud<PointT>& final_colored_cloud)
{
  
  std::vector<Eigen::Vector3i> colors;

  size_t total_clusters = indices.size();
  std::vector<int> used_indices;
  final_colored_cloud.resize(cloud.size());
  for (size_t c = 0; c < total_clusters; ++c)
  {
      Eigen::Vector3i color = get_random_color();
      colors.push_back(color);
      const auto& c_indices = indices[c];

      for (size_t i = 0; i < c_indices.size(); ++i)
      {
          int pt_idx = c_indices[i];
          used_indices.push_back(pt_idx);
          PointT pt = cloud.points[pt_idx];

          pt.r = color[0];
          pt.g = color[1];
          pt.b = color[2];

          final_colored_cloud.points[pt_idx] = pt;
      }
  }

  std::vector<int> all_indices(cloud.size());
  for (int i = 0; i < cloud.size(); ++i)
  {
    all_indices[i] = i;
  }
  std::sort(used_indices.begin(), used_indices.end());
  std::vector<int> unused_indices;
  set_diff(unused_indices, all_indices, used_indices, true);

  for (int i = 0; i < unused_indices.size(); ++i)
  {
  	int pt_idx = unused_indices[i];
  	PointT& pt = final_colored_cloud.points[pt_idx];
  	pt.r = 1;
  	pt.g = 1;
  	pt.b = 1;
  }

  return colors;
}


}



#endif
