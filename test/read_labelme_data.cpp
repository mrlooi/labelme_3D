#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include "pinhole_camera.hh"
#include "labelme_io.hh"

#include "json.hpp"

#define PRINT(a) std::cout << #a << ": " << a << std::endl;

typedef pcl::PointXYZRGBA PointT;

static inline bool compareContourAreasDesc(const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2){
    double i = fabs(contourArea(cv::Mat(contour1)));
    double j = fabs(contourArea(cv::Mat(contour2)));
    return (i > j);
}

static inline bool replaceStr(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}


int main(int argc, char *argv[])
{
  std::string pcd_file;
  std::string json_file;
  if (argc <= 2)
  {
  	printf("Usage: %s pcd_file json_file\n", argv[0]);
  	return -1;
  }
  pcd_file = argv[1];
  json_file = argv[2];

  int morph_kernel_size = 9;
  float octree_resolution = 0.015;
  std::string poses_json_file = "../data/poses.json";

  pcl::console::parse (argc, argv, "-ores", octree_resolution);
  pcl::console::parse (argc, argv, "-morph", morph_kernel_size);
  pcl::console::parse (argc, argv, "-poses", poses_json_file);
  bool vis = pcl::console::find_switch (argc, argv, "-vis");

  pcl::PointCloud<PointT>::Ptr raw_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); // same as annotated cloud, but with original raw cloud colors
  pcl::PointCloud<PointT>::Ptr annotated_cloud (new pcl::PointCloud<PointT>);
  if ( pcl::io::loadPCDFile <PointT> (pcd_file, *raw_cloud) == -1)
  {
    std::cout << "Failed to read " << pcd_file << std::endl;
  	return (-1);
  }

  labelme::LabelMeData l_data;
  bool rt = labelme::read_data(json_file, l_data);
  if (!rt)
  {
    std::cout << "Failed to read " << json_file << std::endl;
    // return -1;
  }

  std::string base_file_name = json_file;
  replaceStr(base_file_name, ".json", "");
  PRINT(base_file_name)

  rt = labelme::read_data(poses_json_file, l_data);

  if (l_data.data.size() == 0)
  {
    *cloud += *raw_cloud;
  } else {
    cloud->resize(l_data.data.size());
    annotated_cloud->resize(l_data.data.size());
    for (int i = 0; i < l_data.data.size(); ++i)
    {
      int idx = l_data.data[i][0];
      assert (idx < raw_cloud->points.size());
      PointT pt = raw_cloud->points[idx];
      cloud->points[i] = pt;
      pt.b = l_data.data[i][1];
      pt.g = l_data.data[i][2];
      pt.r = l_data.data[i][3];
      annotated_cloud->points[i] = pt;
    }
  }

  PRINT(l_data.colors.size())

  if (vis)
  {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud"));
    viewer->addCoordinateSystem();
    viewer->addPointCloud(annotated_cloud);
    viewer->spin();
  }

  // pcl::transformPointCloud(*cloud, *cloud, viewer_pose);
  // viewer->removeAllPointClouds();
  // viewer->addPointCloud(cloud);
  // viewer->spin();

  int img_width = 960;
  int img_height = 960;
  int fx = 800;
  int fy = fx;
  float cx = 0.5 * img_width;
  float cy = 0.5 * img_height;

  PinholeCamera<PointT> pinhole_cam(fx, fy, cx, cy);
  pinhole_cam.set_image_width(img_width);
  pinhole_cam.set_image_height(img_height);

  cv::Mat proj_img, proj_img2;
  std::vector<std::vector<std::vector<int>>> uv_idx_map;
  

  for (int v = 0; v < l_data.poses.size(); ++v)
  {
    const auto& viewer_pose = l_data.poses[v];
    std::cout << "Projecting with camera pose: " << viewer_pose << std::endl;
    pinhole_cam.set_camera_pose(viewer_pose);

    pinhole_cam.set_input_cloud(cloud);
    pinhole_cam.project(proj_img, uv_idx_map);

    pinhole_cam.set_input_cloud(annotated_cloud);
    // pinhole_cam.project(proj_img2, uv_idx_map);
    pinhole_cam.project_non_occluded_surface(proj_img2, uv_idx_map, octree_resolution, octree_resolution * 4);

    if (vis)
    {
      cv::imshow("img", proj_img);
      cv::imshow("img2", proj_img2);
      cv::waitKey(0);
    }


    std::string proj_img_file = base_file_name + "__" + std::to_string(v) + ".jpg";
    std::string proj_img_json_file = base_file_name + "__" + std::to_string(v) + ".json";
    
    cv::imwrite(proj_img_file, proj_img);
    std::cout << "Saved to " << proj_img_file << std::endl;

    std::vector<std::vector<int>> annot_pts(l_data.colors.size());

    for (int i = 0; i < l_data.colors.size(); ++i)
    {
      const auto& color = l_data.colors[i];
      cv::Mat mask = cv::Mat::zeros(proj_img2.size(), CV_8UC1);   
      for (int x = 0; x < proj_img2.cols; ++x)
      {
        for (int y = 0; y < proj_img2.rows; ++y)
        {
          const auto& px = proj_img2.at<cv::Vec3b>(y,x);
          if (px[0] == color[0] && px[1] == color[1] && px[2] == color[2])
          {
            mask.at<uchar>(y,x) = 255;
          }
        }
      }

      cv::imshow("mask", mask);

      // close holes arising from sparse points 
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                  cv::Size(morph_kernel_size, morph_kernel_size));
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

      std::vector<cv::Vec4i> hierarchy;
      std::vector<std::vector<cv::Point>> out_contours;
      cv::findContours(mask.clone(), out_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
      if (out_contours.size() == 0)
      {
        printf("No contours found for color[%d]: %d %d %d\n", i, color[0], color[1], color[2]);
        continue;
      }
      std::sort(out_contours.begin(), out_contours.end(), compareContourAreasDesc);
      std::vector<cv::Point> biggest_contour = out_contours[0];

      // cv::Mat proj_img_copy = proj_img.clone();
      cv::drawContours(proj_img, out_contours, 0, {color[0], color[1], color[2]}, 2, 8);

      // cv::imshow("morph", mask);
      // cv::imshow("contours", proj_img);
      // cv::waitKey(0);
      double arclen = cv::arcLength(cv::Mat(biggest_contour), true);
      double eps = 0.005;
      double epsilon = arclen * eps;
      std::vector<cv::Point> approx_out;
      cv::approxPolyDP(biggest_contour, approx_out, epsilon, true);
      if (approx_out.size() > 0)
      {
        cv::putText(proj_img, std::to_string(i), approx_out[0], cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(color[0], color[1], color[2]), 1, 8);
        // printf("%d) [", i);
        for (const auto& apt: approx_out)
        {
          cv::circle(proj_img, apt, 3, cv::Scalar(0,255,0), 1, 8, 0);
          annot_pts[i].push_back(apt.x);
          annot_pts[i].push_back(apt.y);
          // printf("[%d,%d],", apt.x, apt.y);
        }
        // printf("]\n");
      }

    }

    if (vis)
    {
      cv::imshow("contours", proj_img);
      cv::waitKey(0);
    }

    std::ofstream file(proj_img_json_file);
    if (file.is_open()) 
    {
      nlohmann::json j;
      j["data"] = annot_pts;
      file << j;
      file.close();
      std::cout << "Saved to " << proj_img_json_file << std::endl;
    }
      
  }

  return 0;
}
