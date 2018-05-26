#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>

#include "pinhole_camera.hh"
#include "pcl_viewer_utils.hh"

#define PRINT(a) std::cout << #a << ": " << a << std::endl;

typedef pcl::PointXYZRGBA PointT;


const cv::Scalar RED {0,0,255};
const cv::Scalar GREEN {0,255,0};
const cv::Scalar BLUE {255,0,0};

int img_width = 960;
int img_height = 960;


pcl::PointCloud<PointT>::Ptr raw_cloud (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

// Eigen::MatrixXi uv_idx_map;
std::vector<std::vector<std::vector<int>>> uv_idx_map;

cv::Mat proj_img, proj_img_copy;

std::vector<cv::Point> img_pts;

Eigen::Matrix4f viewer_pose;

void project()
{

  int focal = 800;

  float cx = 0.5 * img_width;
  float cy = 0.5 * img_height;

  PinholeCamera<PointT> pinhole_cam(focal, cx, cy);
  pinhole_cam.set_image_width(img_width);
  pinhole_cam.set_image_height(img_height);

  pinhole_cam.set_camera_pose(viewer_pose);
  pinhole_cam.set_input_cloud(cloud);

  pinhole_cam.project_surface(proj_img, uv_idx_map, 0.1);
  proj_img_copy = proj_img.clone();
  
}

void drawPoints(cv::Mat& img_out, const std::vector<cv::Point>& pts)
{
    // img_out = img.clone();
    float radius = 2;
    const size_t total_pts = pts.size();
    for (int i = 0; i < total_pts; ++i)
    {
        const auto& pt = pts[i];
        cv::circle( img_out, pt, radius, RED, 3, 8, 0 );
        if (i > 0)
        	cv::line( img_out, pts[i-1], pt, cv::Scalar(0,255,0), 1, 8 );	
        // cv::putText(img_out, std::to_string(i), {bbox.x,bbox.y}, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, color, 2, 8);
    }
    // if (total_pts >= 3)
    // {
	   //  for (int i = 0; i < total_pts; ++i)
    // 	{
    // 		cv::line( img_out, pts[i], pts[(i+1)%total_pts], cv::Scalar(0,255,0), 1, 8 );	
    // 	}
    // }
}


static inline void get_viewer_pose(pcl::visualization::PCLVisualizer::Ptr& viewer, Eigen::Matrix4f& view_pose)
{
  Eigen::Affine3f pose_af = viewer->getViewerPose();
  Eigen::Affine3f pose_af_inv = pose_af.inverse();
  for (int i = 0; i < 4; ++i)
  {
    view_pose(i,0) = pose_af_inv(i,0);
    view_pose(i,1) = pose_af_inv(i,1);
    view_pose(i,2) = pose_af_inv(i,2);
    view_pose(i,3) = pose_af_inv(i,3);
  }
  // // rotate along z-axis by 180
  Eigen::Matrix4f z_180_m = Eigen::Matrix4f::Identity();
  z_180_m(0,0) = -1;
  z_180_m(1,1) = -1;
  viewer_pose = z_180_m * viewer_pose;

  // viewer_pose(0,0) = -viewer_pose(0,0);  // z axis is flipped
  // viewer_pose(1,1) = -viewer_pose(1,1);  // z axis is flipped
  // // viewer_pose(2,2) = -viewer_pose(2,2);  // z axis is flipped
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          // cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          cv::Point pt {x,y};
          img_pts.push_back(pt);
          proj_img_copy = proj_img.clone();
          drawPoints(proj_img_copy, img_pts);
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
      else if  ( event == 'x' )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }

     cv::imshow("My Window", proj_img_copy);
}


void trigger_cv_window()
{
  project();

  // cv::flip(proj_img, proj_img, -1); // rotate by 180

  //Create a window
  cv::namedWindow("My Window", 1);

  //set the callback function for any mouse event
  cv::setMouseCallback("My Window", CallBackFunc, NULL);

  //show the image
  while (1)
  {
    // SEE https://progtpoint.blogspot.com/2017/06/key-board-ascii-key-code.html

    cv::imshow("My Window", proj_img_copy);

    char key = (char) cv::waitKey(10); 
    
    if (key == 'c') // backspace
    {
      cout << "C PRESSED\n";
      img_pts.clear();

      proj_img_copy = proj_img.clone();
      drawPoints(proj_img_copy, img_pts);
    }
    else if (key == 8) // backspace
    {
      cout << "BACKSPACE PRESSED\n";
      if (img_pts.size() == 0)
        continue;
      img_pts.pop_back();
      // show again
      proj_img_copy = proj_img.clone();
      drawPoints(proj_img_copy, img_pts);
    }
    else if (key == 13) // enter
    {
      cout << "ENTER PRESSED\n";
      break;
    }
    else if (key == 'q')
    {
      break;
    }
  }

  cv::destroyWindow("My Window");
}

cv::Mat get_mask()
{
	cv::Mat mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
	const cv::Point* elementPoints[1] = { &img_pts[0] };
	int num_pts = img_pts.size();
	cv::fillPoly(mask, elementPoints, &num_pts, 1, 255);
	// cv::imshow("mask", mask);
	// cv::waitKey(0);

	return mask;
}

void get_mask_3d_points(const cv::Mat& mask, std::vector<int>& cloud_indices)
{
	assert(mask.rows >= 1 && mask.rows == uv_idx_map.size() && mask.cols == uv_idx_map[0].size());
	for (int x = 0; x < mask.cols; ++x)
	{
		for (int y = 0; y < mask.rows; ++y)
		{
			if (mask.at<uchar>(y,x) == 255)
			{
				const auto& indices = uv_idx_map[y][x];
				if (indices.size() > 0)
				{
					cloud_indices.insert(cloud_indices.end(), indices.begin(), indices.end());
				}
			}
		}
	}
}

pcl::PointCloud<PointT>::Ptr extract_cloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr indices, bool negative=false)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr extract_cloud(pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> indices, bool negative=false)
{
	pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
	indices_ptr->indices = indices;
	return extract_cloud(cloud, indices_ptr, negative);
}


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

  std::string key = event.getKeySym();
  if (event.keyDown())
  {
  	if (key == "e" || key == "d")
  	{
	    get_viewer_pose(viewer, viewer_pose);
	 	if (key == "e")
	 	{
	 		printf("Operation: EDIT\n");
	 	} else 
	 	{
	 		printf("Operation: DELETE\n");
	 	}
	    trigger_cv_window();
	    cv::Mat mask = get_mask();
	    std::vector<int> cloud_indices;
	    get_mask_3d_points(mask, cloud_indices);
	    if (cloud_indices.size() == 0)
	    {
	    	return;
	    }
	 	if (key == "e")
	 	{
		    auto color = PclViewerUtils::get_random_color();
		    PclViewerUtils::color_cloud_points(*cloud, cloud_indices, color);
	 	} else {
	 		auto& pts = cloud->points;
	 		cloud = extract_cloud(cloud, cloud_indices, true);
	 		// std::sort(cloud_indices.begin(), cloud_indices.end()); // sort so that erase can be ordered
	 		// for (int i = 0; i < cloud_indices.size(); ++i)
	 		// {
	 		// 	int idx = cloud_indices[i] - i;
	 		// 	pts.erase(pts.begin() + idx);
	 		// }
	 	}
    	img_pts.clear();

	    viewer->removeAllPointClouds();
	    viewer->addPointCloud(cloud);
	    viewer->spin();
  	}
  }
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

  if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
      event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    // pcl::visualization::Camera cam;
    // viewer->getCameraParameters(cam);
  }
}


int main(int argc, char *argv[])
{
  std::string pcd_file = "../data/3.pcd";

  if (argc > 1)
  {
    pcd_file = argv[1];
  }

  if ( pcl::io::loadPCDFile <PointT> (pcd_file, *raw_cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  cloud = raw_cloud;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);
  viewer->addCoordinateSystem();
  viewer->addPointCloud(cloud);
  viewer->spin();
}
