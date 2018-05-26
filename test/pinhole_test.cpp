#include <iostream>

#include <pcl/common/transforms.h>

#include <pcl/octree/octree_search.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include "pinhole_camera.hh"

#define PRINT(a) std::cout << #a << ": " << a << std::endl;

typedef pcl::PointXYZRGBA PointT;  // The point type used for input


int main(int argc, char *argv[])
{
	/* code */
	// std::string json_file = "../data/sm.json";
	std::string pcd_file = "../data/rot_cloud_colored.pcd";

	int img_width = 960;
	int img_height = 960;
	int focal = 800;
	pcl::console::parse (argc, argv, "-width", img_width);
	pcl::console::parse (argc, argv, "-height", img_height);
	pcl::console::parse (argc, argv, "-focal", focal);

	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

	if ( pcl::io::loadPCDFile <PointT> (pcd_file, *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	int N = cloud->size();
	PRINT(cloud->size())

	Eigen::Matrix4f viewer_pose;
	viewer_pose << -0.012, -0.861, -0.509, 2.317,
		-0.989, -0.066, 0.134, 0.719,
		-0.149, 0.505, -0.850, 3.117,
		0.000, 0.000, 0.000, 1.000;
	Eigen::Matrix4f camera_pose = viewer_pose.inverse();

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer1"));
	viewer->addCoordinateSystem();

	pcl::PointCloud<PointT>::Ptr rot_cloud (new pcl::PointCloud<PointT>);
	pcl::transformPointCloud(*cloud, *rot_cloud, camera_pose);

	viewer->addPointCloud(rot_cloud);
	viewer->spin();	

	float cx = 0.5 * img_width;
	float cy = 0.5 * img_height;

	PinholeCamera<PointT> pinhole_cam(focal, cx, cy);
	pinhole_cam.set_camera_pose(camera_pose);
	pinhole_cam.set_image_width(img_width);
	pinhole_cam.set_image_height(img_height);
	pinhole_cam.set_input_cloud(cloud);

	Eigen::MatrixXi uv_idx_map;
	cv::Mat proj_img;
	pinhole_cam.project(proj_img, uv_idx_map);


	cv::imshow("proj", proj_img);
	// cv::imwrite("../data/proj_seg.png", proj_img);
	cv::waitKey(0);

	cv::Vec3b sample_color {203, 143, 254};	
	cv::Mat mask = cv::Mat::zeros(proj_img.size(), CV_8UC1);
	std::vector<int> cloud_color_indices;
	for (int y = 0; y < proj_img.rows; ++y)
	{
		for (int x = 0; x < proj_img.cols; ++x)
		{
			const auto& px = proj_img.at<cv::Vec3b>(y,x);
			if (px == sample_color)
			{
				mask.at<uchar>(y,x) = 255;

				int idx = uv_idx_map(y,x);
				if (idx != -1)
				{
					cloud_color_indices.push_back(idx);
				}
			}
		}
	}
	cv::imshow("mask", mask);
	cv::waitKey(0);

	pcl::PointCloud<PointT>::Ptr out_cloud (new pcl::PointCloud<PointT>);	
	for (int i = 0; i < cloud_color_indices.size(); ++i)
	{
		int idx = cloud_color_indices[i];
		out_cloud->push_back(cloud->points[idx]);
	}
	viewer->removeAllPointClouds();
	viewer->addPointCloud(out_cloud);
	viewer->spin();	

	return 0;
}