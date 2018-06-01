#include <iostream>
// #include <stack>

#include <pcl/io/pcd_io.h>

// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>

#include <opencv2/opencv.hpp>

#include "pinhole_camera.hh"
#include "pcl_viewer_custom.hh"
#include "pcl_viewer_utils.hh"

#define PRINT(a) std::cout << #a << ": " << a << std::endl;

typedef pcl::PointXYZRGBA PointT;


const cv::Scalar RED {0,0,255};
const cv::Scalar GREEN {0,255,0};
const cv::Scalar BLUE {255,0,0};

float octree_resolution = 0.02f;

int img_width = 960;
int img_height = 960;
int focal = 800;


pcl::PointCloud<PointT>::Ptr raw_cloud (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

std::string save_file = "./labelme_3d_out.pcd";

// Eigen::MatrixXi uv_idx_map;
std::vector<std::vector<std::vector<int>>> uv_idx_map;

cv::Mat proj_img, proj_img_copy;

std::vector<cv::Point> img_pts;

Eigen::Matrix4f viewer_pose;

Eigen::Vector3i anchor_color {-1,-1,-1};
std::vector<int> current_indices;

std::vector<Eigen::Vector3i> saved_colors;

struct OpData {
	pcl::PointCloud<PointT>::Ptr cloud;
	std::vector<int> parent_indices;
};

std::vector<OpData> undo_data;
std::vector<OpData> redo_data;



void pt_to_rect(const cv::Point& pt, cv::Rect& rect, int width, int height)
{
	rect.x = pt.x - width / 2;
	rect.y = pt.y - height / 2;
	rect.width = width;
	rect.height = height;
}

inline void extractParentIndices(std::vector<int>& indices, const std::vector<int>& parent_indices, const std::vector<int>& child_indices)
{
    assert(child_indices.size() <= parent_indices.size());
    indices.clear();
    indices.reserve(child_indices.size());
    for (size_t i = 0; i < child_indices.size(); ++i)
    {
        indices.push_back(parent_indices[child_indices[i]]);
    }
}

template <typename T>
inline void set_diff(std::vector<T>& result, const std::vector<T>& first, const std::vector<T>& second, bool sorted=true)
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

inline void get_outliers(std::vector<int>& outliers, const std::vector<int>& inliers, const int cloud_sz)
{
	std::vector<int> all_indices(cloud_sz);
	for (int i = 0; i < cloud_sz; ++i)
	{
		all_indices[i] = i;
	}
	std::vector<int> sorted_inliers = inliers;
	std::sort(sorted_inliers.begin(), sorted_inliers.end());
	set_diff(outliers, all_indices, sorted_inliers);
}

void run_undo_redo(std::vector<OpData>& stack1, std::vector<OpData>& stack2)
{
	if (stack1.size() == 0)
		return;
	pcl::PointCloud<PointT>::Ptr cur_cloud (new pcl::PointCloud<PointT>);
	*cur_cloud += *cloud;
	OpData d2;
	d2.cloud = cur_cloud;
	d2.parent_indices = current_indices;
	stack2.push_back(d2);

	const OpData& d = stack1.back();	
	cloud = d.cloud;
	current_indices = d.parent_indices;
	stack1.pop_back();
}
void undo()
{
	run_undo_redo(undo_data, redo_data);
}
void redo()
{
	run_undo_redo(redo_data, undo_data);
}


void project()
{


  float cx = 0.5 * img_width;
  float cy = 0.5 * img_height;

  PinholeCamera<PointT> pinhole_cam(focal, cx, cy);
  pinhole_cam.set_image_width(img_width);
  pinhole_cam.set_image_height(img_height);

  pinhole_cam.set_camera_pose(viewer_pose);
  pinhole_cam.set_input_cloud(cloud);

  // pinhole_cam.project_surface(proj_img, uv_idx_map, 0.03);
  pinhole_cam.project(proj_img, uv_idx_map);
  // pinhole_cam.project_non_occluded_surface(proj_img, uv_idx_map, octree_resolution);
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

void PolygonCallbackFunc(int event, int x, int y, int flags, void* userdata)
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

void SinglePointCallbackFunc(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == cv::EVENT_LBUTTONDOWN )
	{
		// cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

		cv::Rect anchor_rect;
		cv::Point anchor_pt {x,y};
		pt_to_rect(anchor_pt, anchor_rect, 5, 5);

		size_t total_colors = saved_colors.size();
		std::vector<int> saved_colors_count(total_colors);
		// printf("Anchor Rect: %d %d %d %d\n", anchor_rect.x, anchor_rect.y, anchor_rect.x + anchor_rect.width, anchor_rect.y + anchor_rect.height);
		for (int x = anchor_rect.x; x < anchor_rect.x + anchor_rect.width; ++x)
		{	
	    	for (int y = anchor_rect.y; y < anchor_rect.y + anchor_rect.height; ++y)
	    	{
				const auto& px = proj_img_copy.at<cv::Vec3b>(y,x);
		    	for (int c = 0; c < total_colors; ++c)
		    	{
		    		const auto& s_color = saved_colors[c];
					if (px[0] == s_color[0] && px[1] == s_color[1] && px[2] == s_color[2])
					{
						saved_colors_count[c] += 1;
						break;
					}
		    	}
	    	}
		}

		// check if color matches a saved color
		int max_cnt = 0;
		int max_color_idx = 0;
		for (int i = 0; i < total_colors; ++i)
		{
			int cnt = saved_colors_count[i];
			if (cnt > max_cnt)
			{
				max_cnt = cnt;
				max_color_idx = i;
			}
		}
		if (max_cnt == 0)
		{
			printf("No matching color detected, please select a valid anchor point!\n");
			return;
		}

		anchor_color = saved_colors[max_color_idx];
		printf("Selected anchor color: %d %d %d\n", anchor_color[0], anchor_color[1], anchor_color[2]);

		proj_img_copy = proj_img.clone();
		float radius = 3;
		cv::circle( proj_img_copy, {x,y}, radius, BLUE, 3, 8, 0 );
		
		// // draw
		// cv::Mat mask = cv::Mat::zeros(proj_img.size(), CV_8UC1);		
		// for (int x = 0; x < proj_img.cols; ++x)
		// {
		// 	for (int y = 0; y < proj_img.rows; ++y)
		// 	{
		// 		const auto& px = proj_img_copy.at<cv::Vec3b>(y,x);
		// 		if (px[0] == anchor_color[0] && px[1] == anchor_color[1] && px[2] == anchor_color[2])
		// 		{
		// 			mask.at<uchar>(y,x) = 255;
		// 		}
		// 	}
		// }
  //       std::vector<cv::Vec4i> hierarchy;
  //       std::vector<std::vector<cv::Point>> out_contours;
  //       cv::findContours(mask, out_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  //       cv::drawContours(proj_img_copy, out_contours, -1, cv::Scalar(255, 0, 0), 2, 8);
	}
}

void trigger_cv_window()
{
  // cv::flip(proj_img, proj_img, -1); // rotate by 180

  //Create a window
  cv::namedWindow("My Window", 1);

  //set the callback function for any mouse event
  cv::setMouseCallback("My Window", PolygonCallbackFunc, NULL);

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


void trigger_cv_window_merge()
{
	//Create a window
	cv::namedWindow("My Window", 1);

	//set the callback function for any mouse event
	cv::setMouseCallback("My Window", SinglePointCallbackFunc, NULL);
	while (1)
	{
		cv::imshow("My Window", proj_img_copy);

		char key = (char) cv::waitKey(10); 
		if (key == 'q')
		{
		  cv::destroyWindow("My Window");
		  return;
		}
		if (anchor_color[0] != -1 && anchor_color[1] != -1 && anchor_color[2] != -1)
		{
			proj_img = proj_img_copy;
			break;
		}
	}

	trigger_cv_window();
}


cv::Mat get_mask()
{
	cv::Mat mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
	if (img_pts.size() > 0)
	{
		const cv::Point* elementPoints[1] = { &img_pts[0] };
		int num_pts = img_pts.size();
		cv::fillPoly(mask, elementPoints, &num_pts, 1, 255);
		// cv::imshow("mask", mask);
		// cv::waitKey(0);
	}

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

  bool refresh_cloud = false;
  if (event.keyDown())
  {
	get_viewer_pose(viewer, viewer_pose);

	bool is_ctrl = event.isCtrlPressed();
	if (key == "a" || key == "u" || key == "d" || key == "x" || key == "m")
	{
		if (key == "a")
		{
			printf("Operation: ANNOTATE\n");
		} else if (key == "u")
		{
			printf("Operation: UNDO ANNOTATION\n");
			if (saved_colors.size() == 0)
			{
				printf("There are currently no valid annotation colors\n");	
				return;	
			} 
		} else if (key == "d")
		{
			printf("Operation: DELETE\n");
		} else if (key == "x")
		{
			printf("Operation: EXTRACT\n");
		} else if (key == "m")
		{
			printf("Operation: MERGE\n");
			if (saved_colors.size() == 0)
			{
				printf("There are currently no valid annotation colors\n");	
				return;
			}
		} 

    	project();

		if (key == "m")
		{
			printf("Select an anchor point, then draw an extra polygon \n");
	    	trigger_cv_window_merge();
		} else {
			trigger_cv_window();			
		}

		cv::Mat mask = get_mask();
		std::vector<int> cloud_indices;
		get_mask_3d_points(mask, cloud_indices);
		if (cloud_indices.size() == 0)
		{
			img_pts.clear();
			return;
		}

		// add current cloud to undo stack
		pcl::PointCloud<PointT>::Ptr cur_cloud (new pcl::PointCloud<PointT>);
		*cur_cloud += *cloud;
		OpData op_data;
		op_data.cloud = cur_cloud;
		op_data.parent_indices = current_indices;
		undo_data.push_back(op_data);
		redo_data.clear();

		if (key == "a" || key == "m")
		{
	    	Eigen::Vector3i color;
			if (key == "a")
			{
				color = PclViewerUtils::get_random_color();
				printf("Added new annotation color: %d %d %d\n", color[0], color[1], color[2]);
				saved_colors.push_back(color);
			} else {
		    	color = anchor_color;
	    		anchor_color = {-1,-1,-1}; // reset
			}
			PclViewerUtils::color_cloud_points(*cloud, cloud_indices, color);
			printf("Annotated %d points with color: %d %d %d\n", cloud_indices.size(), color[0], color[1], color[2]);
		} else {
			auto& pts = cloud->points;
			if (key == "d")
			{
				std::vector<int> remain_indices;
				get_outliers(remain_indices, cloud_indices, cloud->size());
				std::vector<int> child_indices;
				extractParentIndices(child_indices, current_indices, remain_indices);
				current_indices = child_indices;
				cloud = extract_cloud(cloud, cloud_indices, true);
				printf("Deleted %d points\n", cloud_indices.size());
			} else if (key == "x")
			{
				std::vector<int> child_indices;
				extractParentIndices(child_indices, current_indices, cloud_indices);
				current_indices = child_indices;
				cloud = extract_cloud(cloud, cloud_indices, false);
				printf("Extracted %d points\n", cloud_indices.size());
			} else if (key == "u")
			{
				for (int i = 0; i < cloud_indices.size(); ++i)
				{
					int ix = cloud_indices[i];
					int parent_ix = current_indices[ix];
					cloud->points[ix] = raw_cloud->points[parent_ix];
				}
				printf("Undo annotation for %d points\n", cloud_indices.size());
			}
			// std::sort(cloud_indices.begin(), cloud_indices.end()); // sort so that erase can be ordered
			// for (int i = 0; i < cloud_indices.size(); ++i)
			// {
			// 	int idx = cloud_indices[i] - i;
			// 	pts.erase(pts.begin() + idx);
			// }
		}
		img_pts.clear();

		refresh_cloud = true;
	} 
	if (is_ctrl)
	{
		if (key == "s")
		{
			if (save_file.size() > 0)
			{
				pcl::io::savePCDFileBinary(save_file, *cloud);
				printf("Saved to %s\n", save_file.c_str());
			} else {
				printf("Please pass a valid file to save_file [-s] argument\n");
			}
		}
		else if (key == "z")
		{
			printf("Ctrl+z PRESSED\n");		
			undo();
			refresh_cloud = true;
		}
		else if (key == "y")
		{
			printf("Ctrl+y PRESSED\n");		
			redo();
			refresh_cloud = true;
		}
	}

	if (refresh_cloud)
	{
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
  std::string pcd_file;
  if (argc <= 1)
  {
  	printf("Usage: %s pcd_file\n", argv[0]);
  	return -1;
  }
  pcd_file = argv[1];

  if ( pcl::io::loadPCDFile <PointT> (pcd_file, *raw_cloud) == -1)
  {
	std::cout << "Cloud reading failed." << std::endl;
	return (-1);
  }

  pcl::console::parse (argc, argv, "-ores", octree_resolution);
  pcl::console::parse (argc, argv, "-width", img_width);
  pcl::console::parse (argc, argv, "-height", img_height);
  pcl::console::parse (argc, argv, "-focal", focal);
  pcl::console::parse (argc, argv, "-s", save_file);

  *cloud += *raw_cloud;

  current_indices.resize(raw_cloud->size());
  for (int i = 0; i < current_indices.size(); ++i)
  {
  	current_indices[i] = i;
  }

  PCLInteractorCustom* style = PCLInteractorCustom::New(); 
  pcl::visualization::PCLVisualizer::Ptr viewer(new PCLVisCustom(style));
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);
  viewer->addCoordinateSystem();
  viewer->addPointCloud(cloud);
  viewer->spin();
}
