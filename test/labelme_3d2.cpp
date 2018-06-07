/*
 A tool for labelling 3D pointclouds

 Copyright (c) 2018 Shi Looi <vincent.sj.looi@gmail.com>
 Licensed under the MIT license.
*/

#include <iostream>
#include <fstream>
#include <unordered_map>  
#include <algorithm>


#include <future>         // std::async, std::future

#include <pcl/io/pcd_io.h>

// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>

#include <opencv2/opencv.hpp>


#define CVUI_IMPLEMENTATION
#include "cvui.h"

#include "pinhole_camera.hh"
#include "pcl_viewer_custom.hh"
#include "pcl_viewer_utils.hh"

#include "labelme_io.hh"


#define PRINT(a) std::cout << #a << ": " << a << std::endl;

typedef pcl::PointXYZRGBA PointT;
typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;


const cv::Scalar RED {0,0,255};
const cv::Scalar GREEN {0,255,0};
const cv::Scalar BLUE {255,0,0};

float octree_resolution = 0.02f;

int img_width = 960;
int img_height = 960;
int focal = 800;

pcl::visualization::PCLVisualizer::Ptr viewer;

pcl::PointCloud<PointT>::Ptr raw_cloud (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

std::string pcd_file;
std::string out_pcd_file = "./labelme_3d_out.pcd";
std::string out_json_file = "./labelme_3d_out.json";
std::string in_json_file = "";

// Eigen::MatrixXi uv_idx_map;
std::vector<std::vector<std::vector<int>>> uv_idx_map;

cv::Mat proj_img, proj_img_copy;

std::vector<cv::Point> img_pts;

Eigen::Matrix4f viewer_pose;

Eigen::Vector3i anchor_color {-1,-1,-1};
std::vector<int> current_indices;

std::vector<Eigen::Vector3i> saved_colors;
// std::unordered_map<int, Eigen::Vector3i> point_color_map;

struct OpData {
	pcl::PointCloud<PointT>::Ptr cloud;
	std::vector<int> parent_indices;
};

std::vector<OpData> undo_data;
std::vector<OpData> redo_data;

bool check_color_exists(const Eigen::Vector3i& color)
{
	for (int i = 0; i < saved_colors.size(); ++i)  // there are obviously much faster ways to do this e.g. hash, too lazy
	{
		const auto& c = saved_colors[i];
		if (color[0] == c[0] && color[1] == c[1] && color[2] == c[2])
		{
			return true;
		}
	}
	return false;
}

bool check_pt_color_exists(const PointT& pt)
{
	Eigen::Vector3i color {pt.b, pt.g, pt.r};
	return check_color_exists(color);
}

void annotate_pt(PointT& pt, const Eigen::Vector3i& color)
{
	// assert (pt_idx < cloud->size());
	// PointT& pt = cloud->points[pt_idx];
	pt.b = color[0];
	pt.g = color[1];
	pt.r = color[2];
	// point_color_map[pt_idx] = color;
}

int color_cloud_points(pcl::PointCloud<PointT>& cloud, const std::vector<int>& indices, const Eigen::Vector3i color, bool override_annots = true)
{
	// Eigen::Vector3f color {float(colori[0])/255, float(colori[1])/255, float(colori[2])/255};
	int colored_cnt = 0;
	if (override_annots)
	{
		for (int i = 0; i < indices.size(); ++i)
		{
			PointT& pt = cloud.points[indices[i]];
			pt.b = color[0];
			pt.g = color[1];
			pt.r = color[2];	
		}
		colored_cnt = indices.size();
	} else {
		for (int i = 0; i < indices.size(); ++i)
		{
			int pt_idx = indices[i];
			PointT& pt = cloud.points[pt_idx];
			if (!check_pt_color_exists(pt))
			{
				annotate_pt(pt, color);
				++colored_cnt;
			}
		}
	}

	return colored_cnt;
}

Eigen::Vector3i generate_random_color()
{
	Eigen::Vector3i color;
	while (true)
	{
		color = PclViewerUtils::get_random_color();
		bool is_used = check_color_exists(color);
		if (!is_used)
			break;
		// int hash = color[0] + color[1] << 1 + color[2] << 2;
		// if (color_hash_map.find(hash) == color_hash_map.end())
		// {

		// 	break;
		// }
	}
	return color;
}

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
	std::cout << "Projecting with camera pose: " << viewer_pose << std::endl;

	pinhole_cam.set_input_cloud(cloud);

	// pinhole_cam.project_surface(proj_img, uv_idx_map, 0.03);
	pinhole_cam.project(proj_img, uv_idx_map);
	// pinhole_cam.project_non_occluded_surface(proj_img, uv_idx_map, octree_resolution, octree_resolution * 4);
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


void print_help()
{
	printf(
"\n===================HELP===================\n"
"## Commands \n"
"### PCL Viewer\n"
"- Annotate: `a` (annotate points in a polygon - currently assigns a random, unique annotation color to the points)\n"
"- Merge Annotation: `m` (annotate points in a polygon - this time the annotation color is defined by a selected color, obtained from 'Annotate' action)\n"
"- Undo-Annotation: `u`  (converts points in the polygon back to their original colors)\n"
"- Delete: `d`   (removes all points in the polygon)\n"
"- Extract: `x`  (extracts all points in the polygon -> opposite of 'delete')\n"
"- Undo: `Ctrl + z`\n"
"- Redo: `Ctrl + y`\n"
"- Save: `Ctrl + s`  (saves a pcd_file containing the final pointcloud and a json_file containing the annotation colors, respective point indices (relative to original cloud) and point colors. See Usage for setting out_pcd_file and out_json_file argument)\n"
"- Quit: `Esc`  (quits the program)\n"
"\n"
"### OpenCV pop-up window\n"
"- Draw a polygon point: `Left-click`\n"
"- Undo previous polygon point: `backspace`\n"
"- Clear: `c` (clears all existing polygon points)\n"
"- Quit: `q`  (closes the pop-up window (but not the PCL viewer))\n"
"===================  ===================\n\n"
);

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

bool save_data()
{
	printf("Saving data...\n");

	labelme::LabelMeData l_data;

	l_data.pcd_file = pcd_file;
	l_data.colors = saved_colors;

	// save all remaining indices + color for each
	size_t total_idx = current_indices.size();
	l_data.data.resize(total_idx);
	for (size_t i = 0; i < total_idx; ++i)
	{
		int idx = current_indices[i];
		const PointT& pt = cloud->points[i];
		l_data.data[i] = {idx, pt.b, pt.g, pt.r};  // idx bgr
	}

	// save
	bool rt = labelme::save_data(out_json_file, l_data);
	if (rt)
	{
		printf("Saved data to %s\n", out_json_file.c_str());
	} else {
		printf("FAILED to open %s\n", out_json_file.c_str());
	}

	return rt;
}

void add_current_cloud_to_stack()
{
	// add current cloud to undo stack
	pcl::PointCloud<PointT>::Ptr cur_cloud (new pcl::PointCloud<PointT>);
	*cur_cloud += *cloud;
	OpData op_data;
	op_data.cloud = cur_cloud;
	op_data.parent_indices = current_indices;
	undo_data.push_back(op_data);
	redo_data.clear();
}

inline void to_lower_case(std::string& s)
{
	std::transform(s.begin(), s.end(), s.begin(), ::tolower);
}

void refresh_cloud_func(bool save=true, bool im_project=true)
{
	viewer->removeAllPointClouds();
	viewer->addPointCloud(cloud);

	if (im_project)
	{
		project();
		cv::imshow("My Window", proj_img);
		cv::waitKey(10);
	}

	if (save)
	{
		std::future<bool> fut = std::async (std::launch::async, save_data); // auto save
		std::future_status fut_status; 

		// save_data(); // auto save
		do {
			viewer->spinOnce(10);
			fut_status = fut.wait_for(std::chrono::milliseconds(20));
			// printf("FUT wait %d\n", fut_status);
		} while (fut_status != std::future_status::ready);
		fut.get();
	} else {
		viewer->spinOnce(10);
	}
}



void process(std::string key, bool is_ctrl=false)
{
	cv::Mat mask = get_mask();
	std::vector<int> cloud_indices;
	get_mask_3d_points(mask, cloud_indices);
	if (cloud_indices.size() == 0)
	{
		img_pts.clear();
		return;
	}

	// add current cloud to undo stack
	add_current_cloud_to_stack();

	if (key == "a" || key == "m")
	{
		Eigen::Vector3i color;
		if (key == "a")
		{
			color = generate_random_color();
			printf("Added new annotation color: %d %d %d\n", color[0], color[1], color[2]);
			saved_colors.push_back(color);
		} else {
			color = anchor_color;
			anchor_color = {-1,-1,-1}; // reset
		}

		bool override_annots = !is_ctrl;
		int colored_cnt = color_cloud_points(*cloud, cloud_indices, color, override_annots); // if control is pressed, don't override previous annotations
		printf("Annotated %d points with color: %d %d %d\n", colored_cnt, color[0], color[1], color[2]);
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
				// point_color_map.erase(ix);
			}
			printf("Undo annotation for %d points\n", cloud_indices.size());
		}
	}
	img_pts.clear();

	refresh_cloud_func(false, true);
}

bool get_anchor_color(const cv::Point& anchor_pt)
{
	cv::Rect anchor_rect;
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
		return false;
	}

	anchor_color = saved_colors[max_color_idx];
	printf("Selected anchor color: %d %d %d\n", anchor_color[0], anchor_color[1], anchor_color[2]);

	proj_img_copy = proj_img.clone();
	float radius = 3;
	cv::circle( proj_img_copy, anchor_pt, radius, BLUE, 3, 8, 0 );

	return true;
}

void trigger_cv_window()
{
	// cv::flip(proj_img, proj_img, -1); // rotate by 180
	project();
	//Create a window
	cv::namedWindow("My Window", 1);

    cvui::init("My Window");

    ArrayXb modes = ArrayXb::Constant(6,false);

    bool& ANNOTATE = modes(0);
    bool& DELETE = modes(1);
    bool& MERGE = modes(2);
    bool& UNDO = modes(3);
    bool& CTRL_ANNOTATE = modes(4);
    bool& CTRL_MERGE = modes(5);

    ANNOTATE = true;

    cv::Rect mode_window_rect {10,50,240,240};

	//show the image
	while (1)
	{
		// SEE https://progtpoint.blogspot.com/2017/06/key-board-ascii-key-code.html
        bool a_mode = ANNOTATE;
        bool d_mode = DELETE;
        bool m_mode = MERGE;
        bool u_mode = UNDO;
        bool ca_mode = CTRL_ANNOTATE;
        bool cm_mode = CTRL_MERGE;


        bool undo_flag = false, redo_flag = false;

        cvui::window(proj_img_copy, mode_window_rect.x, mode_window_rect.y, mode_window_rect.width, mode_window_rect.height, "Mode");
        cvui::checkbox(proj_img_copy, 15, 80, "Annotate", &a_mode);
        cvui::checkbox(proj_img_copy, 100, 80, "Ctrl Annotate", &ca_mode);
        cvui::checkbox(proj_img_copy, 15, 160, "Merge", &m_mode);
        cvui::checkbox(proj_img_copy, 100, 160, "Ctrl Merge", &cm_mode);
        cvui::checkbox(proj_img_copy, 15, 120, "Delete", &d_mode);
        cvui::checkbox(proj_img_copy, 15, 180, "UndoAnnotation", &u_mode);

        cvui::printf(proj_img_copy, 15, 200, 0.4, 0x000000, "Revert");
        cvui::checkbox(proj_img_copy, 15, 240, "Undo", &undo_flag);
        cvui::checkbox(proj_img_copy, 80, 240, "Redo", &redo_flag);

        cvui::printf(proj_img_copy, 15, 260, 0.4, 0xff0000, "Undo size: %d, Redo size: %d", undo_data.size(), redo_data.size());

		cv::imshow("My Window", proj_img_copy);
        cvui::update();

        if (undo_flag)
        {
            printf("UNDO!\n");
            if (undo_data.size() > 0)
            {
				undo();
				refresh_cloud_func(false);
            }
            continue;
        } else if (redo_flag)
        {
            printf("REDO!\n");
            if (redo_data.size() > 0)
            {
				redo();
				refresh_cloud_func(false);
            }
            continue;
        } else if (a_mode && !ANNOTATE)
        {
            modes = ArrayXb::Constant(6,false);
            ANNOTATE = true;
            continue;
        }
        else if (d_mode && !DELETE)
        {
            modes = ArrayXb::Constant(6,false);
            DELETE = true;
            continue;
        }
        else if (m_mode && !MERGE)
        {
            modes = ArrayXb::Constant(6,false);
            MERGE = true;
            continue;
        }
        else if (u_mode && !UNDO)
        {
            modes = ArrayXb::Constant(6,false);
            UNDO = true;
            continue;
        }
        else if (ca_mode && !CTRL_ANNOTATE)
        {
            modes = ArrayXb::Constant(6,false);
            CTRL_ANNOTATE = true;
            continue;
        }
        else if (cm_mode && !CTRL_MERGE)
        {
            modes = ArrayXb::Constant(6,false);
            CTRL_MERGE = true;
            continue;
        }

		char key = (char) cv::waitKey(10);

        if (cvui::mouse(cvui::DOWN)) 
        {
            // Position the rectangle at the mouse pointer.
            cv::Point pt;
            pt.x = cvui::mouse().x;
            pt.y = cvui::mouse().y;

            if (!(pt.x > mode_window_rect.x && pt.x < mode_window_rect.x + mode_window_rect.width &&
            	pt.y > mode_window_rect.y && pt.y < mode_window_rect.y + mode_window_rect.height))
            {
            	if ((MERGE || CTRL_MERGE) && (anchor_color[0] == -1 || anchor_color[1] == -1 || anchor_color[2] == -1))
            	{
            		bool rt = get_anchor_color(pt);
            		if (rt)
            		{
            			proj_img = proj_img_copy;
            		}
            	} else {
		    		img_pts.push_back(pt);
		    		proj_img_copy = proj_img.clone();
					drawPoints(proj_img_copy, img_pts);
            	}
            }
        }
		
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
			std::string key = "a";
			bool is_ctrl = false;
			if (DELETE)
				key = "d";
			else if (MERGE)
				key = "m";
			else if (UNDO)
				key = "u";
			else if (CTRL_ANNOTATE)
			{
				is_ctrl = true;
				key = "a";
			}
			else if (CTRL_MERGE)
			{
				is_ctrl = true;
				key = "m";
			}
			process(key, is_ctrl);
		}
		else if (key == 'q')
		{
			cv::destroyWindow("My Window");
			break;
		}
	}

}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

	std::string key = event.getKeySym();
	to_lower_case(key);

	bool refresh_cloud = false;
	if (event.keyDown())
	{
		if (key == "h")
		{
			print_help();
			return;
		}

		get_viewer_pose(viewer, viewer_pose);

		bool is_ctrl = event.isCtrlPressed();
		if (key == "a")
		{
			// project();
			trigger_cv_window();			
			viewer->spin();
			// refresh_cloud = true;
		} 
		if (is_ctrl)
		{
			if (key == "s")
			{
				if (out_pcd_file.size() > 0)
				{
					pcl::io::savePCDFileBinary(out_pcd_file, *cloud);
					printf("Saved to %s\n", out_pcd_file.c_str());
				} else {
					printf("Please pass a valid file to out_pcd_file [-s] argument\n");
				}
				if (out_json_file.size() > 0)
				{
					save_data();
				} else {
					printf("Please pass a json file\n");
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
			refresh_cloud_func(true, false);
			viewer->spin();
		}
	}
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
	  event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		// pcl::visualization::Camera cam;
		// viewer->getCameraParameters(cam);
	}
}

void read_data()
{
	bool json_is_valid = false;

	labelme::LabelMeData l_data;

	if (in_json_file.size() > 0)
	{
		json_is_valid = labelme::read_data(in_json_file, l_data);
	}

	if (json_is_valid)
	{
		// add_current_cloud_to_stack();

		// populate annotated colors
		saved_colors = l_data.colors;

		// populate current indices & cloud
		size_t d_size = l_data.data.size();
		assert (d_size <= raw_cloud->size());

		current_indices.resize(d_size);
		cloud->resize(d_size);
		for (int i = 0; i < d_size; ++i)
		{
			const auto& d = l_data.data[i];
			int idx = d[0];
			current_indices[i] = idx;
			PointT pt = raw_cloud->points[idx];
			pt.b = d[1];
			pt.g = d[2];
			pt.r = d[3];
			cloud->points[i] = pt;

			// Eigen::Vector3i color {pt.b, pt.g, pt.r};
			// if (check_color_exists(color))
			// {
			// 	// point_color_map[i] = color;
			// }
		}

		printf("Loaded data from %s\n", in_json_file.c_str());
	} 
}


int main(int argc, char *argv[])
{
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
	print_help();

	pcl::console::parse (argc, argv, "-ores", octree_resolution);
	pcl::console::parse (argc, argv, "-width", img_width);
	pcl::console::parse (argc, argv, "-height", img_height);
	pcl::console::parse (argc, argv, "-focal", focal);
	pcl::console::parse (argc, argv, "-s", out_pcd_file);
	pcl::console::parse (argc, argv, "-ij", in_json_file);
	pcl::console::parse (argc, argv, "-oj", out_json_file);


	*cloud += *raw_cloud;

	current_indices.resize(raw_cloud->size());
	for (int i = 0; i < current_indices.size(); ++i)
	{
		current_indices[i] = i;
	}

	read_data();


	PCLInteractorCustom* style = PCLInteractorCustom::New(); 
	viewer = pcl::visualization::PCLVisualizer::Ptr(new PCLVisCustom(argc, argv, style));
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);
	viewer->addCoordinateSystem();
	viewer->addPointCloud(cloud);
	viewer->spin();

	return 0;
}
