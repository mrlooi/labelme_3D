#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <json.hpp> 

#include "pinhole_camera.hh"

using namespace nlohmann;

#define PRINT(a) std::cout << #a << ": " << a << std::endl;

typedef pcl::PointXYZRGBA PointT;

static inline bool keyExists(json& j, const std::string key) 
{
    return j[key] != nullptr;
}

static inline bool compareContourAreasDesc(const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2){
    double i = fabs(contourArea(cv::Mat(contour1)));
    double j = fabs(contourArea(cv::Mat(contour2)));
    return (i > j);
}

bool LoadDataFromJson(json& j, const std::string json_file)
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


  pcl::PointCloud<PointT>::Ptr raw_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  if ( pcl::io::loadPCDFile <PointT> (pcd_file, *raw_cloud) == -1)
  {
    std::cout << "Failed to read " << pcd_file << std::endl;
  	return (-1);
  }

  json j;
  bool rt = LoadDataFromJson(j, json_file);
  if (!rt)
  {
    std::cout << "Failed to read " << json_file << std::endl;
    return (-1);
  }

  if (!keyExists(j,"data"))
  {
    printf("NO data FIELD\n");
    return -1;
  }
  if (!keyExists(j,"colors"))
  {
    printf("NO colors FIELD\n");
    return -1;
  }
  json& j_data = j["data"]; // N * 4
  json& j_colors = j["colors"]; // N * 4
  const std::vector<std::vector<int>>& colors = j_colors.get<std::vector<std::vector<int>>>();
  const std::vector<std::vector<int>>& data = j_data.get<std::vector<std::vector<int>>>();

  cloud->resize(data.size());
  for (int i = 0; i < data.size(); ++i)
  {
    int idx = data[i][0];
    assert (idx < raw_cloud->points.size());
    PointT pt = raw_cloud->points[idx];
    pt.b = data[i][1];
    pt.g = data[i][2];
    pt.r = data[i][3];
    cloud->points[i] = pt;
  }

  PRINT(colors.size())

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud"));
  viewer->addCoordinateSystem();
  viewer->addPointCloud(cloud);
  viewer->spin();

  Eigen::Matrix4f viewer_pose; 
  viewer_pose << 0.118397,0.988376,0.0953627,-0.829201,0.759221,-0.0282105,-0.650221,-0.00693691,-0.639973,0.149386,-0.753736,2.37129,0,0,0,1;

  // pcl::transformPointCloud(*cloud, *cloud, viewer_pose);
  // viewer->removeAllPointClouds();
  // viewer->addPointCloud(cloud);
  // viewer->spin();

  int img_width = 960;
  int img_height = 960;
  int focal = 800;
  float cx = 0.5 * img_width;
  float cy = 0.5 * img_height;

  PinholeCamera<PointT> pinhole_cam(focal, cx, cy);
  pinhole_cam.set_image_width(img_width);
  pinhole_cam.set_image_height(img_height);

  pinhole_cam.set_camera_pose(viewer_pose);
  std::cout << "Projecting with camera pose: " << viewer_pose << std::endl;

  cv::Mat proj_img, proj_img2;
  std::vector<std::vector<std::vector<int>>> uv_idx_map;
  
  pinhole_cam.set_input_cloud(raw_cloud);
  pinhole_cam.project(proj_img, uv_idx_map);

  pinhole_cam.set_input_cloud(cloud);
  pinhole_cam.project(proj_img2, uv_idx_map);

  cv::imshow("img", proj_img);
  cv::imshow("img2", proj_img2);
  cv::waitKey(0);

  for (int i = 0; i < colors.size(); ++i)
  {
    auto color = colors[i];
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

    // close holes arising from sparse points 
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                cv::Size(13, 13));
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

    cv::imshow("mask", mask);
    cv::imshow("contours", proj_img);
    cv::waitKey(0);
  }


  return 0;
}
