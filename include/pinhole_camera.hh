#ifndef SM_READER_HH
#define SM_READER_HH

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/octree/octree_search.h>  // for occlusion detection
#include <pcl/surface/texture_mapping.h>  // for occlusion detection

/* 
You need first 3 parameters that define your camera: the focal length f, and the center of the projection plane: cx, cy. With this you create a 3x3 matrix (I will use matlab syntax):

A = [ f 0 cx;
	  0 f cy;
	  0 0  1 ];
You can use something like cx = 0.5 * image_width, cy = 0.5 * image_height, and some value as f = 800 (try some of them to check how the image looks better).

Then, a 3x4 matrix with the transformation from the camera frame to the point cloud frame:

T = [ r11 r12 r13 tx;
	  r21 r22 r23 ty;
	  r31 r32 r33 tz ];
And finally, your point cloud in homogeneous coordinates, i.e. in a 4xN matrix for a point cloud with N points:

P = [ x1 x2 ... xN;
	  y1 y2 ... yN;
	  z1 z2 ... zN;
	   1  1 ...  1 ];
Now you can project the points:

S = A * T * P;
S is a 3xN matrix where the pixel coordinates of each i-th 3D point are:

x = S(1, i) / S(3, i);
y = S(2, i) / S(3, i);
*/

template <typename PointT>
class PinholeCamera
{
	typedef typename pcl::PCLBase<PointT>::PointCloudConstPtr PointCloudConstPtr;
	typedef typename pcl::octree::OctreePointCloudSearch<PointT> Octree;
	typedef typename Octree::Ptr OctreePtr;

public:
	PinholeCamera(float focal, float cx, float cy, const Eigen::Matrix4f& camera_pose = Eigen::Matrix4f::Identity()):
		T_(3,4)
	{
		f_ = focal;
		cx_ = cx;
		cy_ = cy;

		set_camera_pose(camera_pose);
	}

	// setter methods
	inline void set_focal(float focal)
	{	
		f_ = focal;
	}
	inline void set_cx(float cx)
	{	
		cx_ = cx;
	}
	inline void set_cy(float cy)
	{	
		cy_ = cy;
	}

	inline void set_image_width(int width)
	{
		image_width_ = width;
	}
	inline void set_image_height(int height)
	{
		image_height_ = height;
	}

	inline void set_camera_pose(const Eigen::Matrix4f& camera_pose)
	{
		camera_pose_ = camera_pose;
	}

	inline void set_input_cloud(const PointCloudConstPtr &cloud)
	{
		cloud_ = cloud;

		has_cloud = true;
	}

	// getter methods
	inline float get_focal() const
	{
		return f_;
	}
	inline float get_cx() const
	{
		return cx_;
	}
	inline float get_cy() const
	{
		return cy_;
	}
	inline int get_image_width() const
	{
		return image_width_;
	}
	inline int get_image_height() const
	{
		return image_height_;
	}
	inline size_t get_cloud_size() const
	{
		return cloud_->size();
	}
	inline Eigen::Matrix4f get_camera_pose() const
	{
		return camera_pose_;
	}


	void project(cv::Mat& proj_img, Eigen::MatrixXi& uv_idx_map)
	{
		if (!has_cloud)
		{
			printf("Cloud has not yet been added!\n");
			return;
		}
		compute_A();
		compute_T();
		compute_P();

		Eigen::MatrixXf S = A_ * T_ * P_;

		proj_img = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
		uv_idx_map = Eigen::MatrixXi::Constant(image_height_, image_width_, -1);

		typename pcl::PointCloud<PointT>::Ptr rot_cloud (new pcl::PointCloud<PointT>);
		pcl::transformPointCloud(*cloud_, *rot_cloud, camera_pose_);

		const int N = cloud_->size();
		// store a cache of the distance of all the points to origin
		std::vector<float> pts_distance_cache(N);
		for (int i = 0; i < N; ++i)
		{
			pts_distance_cache[i] = get_pt_distance(rot_cloud->points[i]);
		}

		for (int i = 0; i < N; ++i)
		{
			// std::cout << S(0, i) << std::endl;
			float x = S(0, i) / S(2, i);
			float y = S(1, i) / S(2, i);

			if (x >= image_width_ || x < 0 || y >= image_height_ || y < 0)
				continue;

			const PointT& pt = cloud_->points[i];
			if (!pt_has_nan(pt))
			{
				// printf("%.3f %.3f,", y, x);
				bool updated = false;
				int prev_map_idx = uv_idx_map(y, x);
				if (prev_map_idx == -1)
				{
					uv_idx_map(y, x) = i;
					updated = true;
				} else {
					// if another point has the same uv mapping pixel, compare distance from camera to point and use the closest 
					if (pts_distance_cache[i] < pts_distance_cache[prev_map_idx])
					{
						uv_idx_map(y, x) = i;	
						updated = true;
					}
				}

				if (updated)
				{			
					auto& px = proj_img.at<cv::Vec3b>(y,x);	
					px[0] = pt.b;
					px[1] = pt.g;
					px[2] = pt.r;
				}
			}
		}
	}

	void project(cv::Mat& proj_img, std::vector<std::vector<std::vector<int>>>& uv_idx_map)
	{
		if (!has_cloud)
		{
			printf("Cloud has not yet been added!\n");
			return;
		}
		compute_A();
		compute_T();
		compute_P();

		Eigen::MatrixXf S = A_ * T_ * P_;

		proj_img = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
		uv_idx_map.resize(image_height_);
		for (int y = 0; y < image_height_; ++y)
		{
			uv_idx_map[y].resize(image_width_);
			for (int x = 0; x < image_width_; ++x)
			{
				uv_idx_map[y][x] = {};
			}
		}

		typename pcl::PointCloud<PointT>::Ptr rot_cloud (new pcl::PointCloud<PointT>);
		pcl::transformPointCloud(*cloud_, *rot_cloud, camera_pose_);

		const int N = cloud_->size();
		// store a cache of the distance of all the points to origin
		std::vector<float> pts_distance_cache(N);
		for (int i = 0; i < N; ++i)
		{
			pts_distance_cache[i] = get_pt_distance(rot_cloud->points[i]);
		}

		// Store a cache (as Matrix) of the minimum point distance for each pixel point
		Eigen::MatrixXf uv_min_distance_cache = Eigen::MatrixXf::Constant(image_height_, image_width_, -1); 

		for (int i = 0; i < N; ++i)
		{
			// std::cout << S(0, i) << std::endl;
			float x = S(0, i) / S(2, i);
			float y = S(1, i) / S(2, i);

			if (x >= image_width_ || x < 0 || y >= image_height_ || y < 0)
				continue;

			const PointT& pt = cloud_->points[i];
			if (!pt_has_nan(pt))
			{
				uv_idx_map[y][x].push_back(i);

				float pt_dist = pts_distance_cache[i];
				float min_cache_dist = uv_min_distance_cache(y,x);

				if (pt_dist < min_cache_dist || min_cache_dist == -1)
				{
					auto& px = proj_img.at<cv::Vec3b>(y,x);
					px[0] = pt.b;
					px[1] = pt.g;
					px[2] = pt.r;

					uv_min_distance_cache(y,x) = pt_dist;
				}
			}
		}
	}

	/* 

	*/
	void project_surface(cv::Mat& proj_img, std::vector<std::vector<std::vector<int>>>& uv_idx_map, float dist_tolerance=0)
	{
		if (!has_cloud)
		{
			printf("Cloud has not yet been added!\n");
			return;
		}
		compute_A();
		compute_T();
		compute_P();

		Eigen::MatrixXf S = A_ * T_ * P_;

		proj_img = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
		uv_idx_map.resize(image_height_);
		for (int y = 0; y < image_height_; ++y)
		{
			uv_idx_map[y].resize(image_width_);
			for (int x = 0; x < image_width_; ++x)
			{
				uv_idx_map[y][x] = {};
			}
		}

		typename pcl::PointCloud<PointT>::Ptr rot_cloud (new pcl::PointCloud<PointT>);
		pcl::transformPointCloud(*cloud_, *rot_cloud, camera_pose_);

		const int N = cloud_->size();
		// store a cache of the distance of all the points to origin
		std::vector<float> pts_distance_cache(N);
		for (int i = 0; i < N; ++i)
		{
			pts_distance_cache[i] = get_pt_distance(rot_cloud->points[i]);
		}

		// Store a cache (as Matrix) of the minimum point distance for each pixel point
		Eigen::MatrixXf uv_min_distance_cache = Eigen::MatrixXf::Constant(image_height_, image_width_, -1); 

		for (int i = 0; i < N; ++i)
		{
			// std::cout << S(0, i) << std::endl;
			float x = S(0, i) / S(2, i);
			float y = S(1, i) / S(2, i);

			if (x >= image_width_ || x < 0 || y >= image_height_ || y < 0)
				continue;

			const PointT& pt = cloud_->points[i];
			if (!pt_has_nan(pt))
			{
				float pt_dist = pts_distance_cache[i];
				float min_cache_dist = uv_min_distance_cache(y,x);

				bool updated = false;
				if (min_cache_dist == -1 || pt_dist < min_cache_dist)
				{
					uv_min_distance_cache(y,x) = pt_dist;
					// update (check previous added points and remove if exceeds dist tolerance) then add this
					auto& cur_indices = uv_idx_map[y][x]; // naturally sorted
					int r = 0;
					for (int ix = 0; ix < cur_indices.size(); ++ix)
					{
						int idx = ix - r;
						if (pts_distance_cache[cur_indices[idx]] > pt_dist + dist_tolerance)
						{
							cur_indices.erase(cur_indices.begin() + idx);
							++r;
						}
					}
					cur_indices.push_back(i);

					auto& px = proj_img.at<cv::Vec3b>(y,x);	
					px[0] = pt.b;
					px[1] = pt.g;
					px[2] = pt.r;
				} else if (pt_dist < min_cache_dist + dist_tolerance)
				{
					// add 
					uv_idx_map[y][x].push_back(i);
				}
			}
		}
	}

	void project_non_occluded_surface(cv::Mat& proj_img, std::vector<std::vector<std::vector<int>>>& uv_idx_map, float octree_resolution=0.02)
	{
		if (!has_cloud)
		{
			printf("Cloud has not yet been added!\n");
			return;
		}
		compute_A();
		compute_T();
		compute_P();

		Eigen::MatrixXf S = A_ * T_ * P_;

		proj_img = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
		uv_idx_map.resize(image_height_);
		for (int y = 0; y < image_height_; ++y)
		{
			uv_idx_map[y].resize(image_width_);
			for (int x = 0; x < image_width_; ++x)
			{
				uv_idx_map[y][x] = {};
			}
		}

		typename pcl::PointCloud<PointT>::Ptr rot_cloud (new pcl::PointCloud<PointT>);
		pcl::transformPointCloud(*cloud_, *rot_cloud, camera_pose_);

		OctreePtr octree (new Octree(octree_resolution));
		octree->setInputCloud (rot_cloud);
		octree->addPointsFromInputCloud ();

		const int N = cloud_->size();

		for (int i = 0; i < N; ++i)
		{
			// std::cout << S(0, i) << std::endl;
			float x = S(0, i) / S(2, i);
			float y = S(1, i) / S(2, i);

			if (x >= image_width_ || x < 0 || y >= image_height_ || y < 0)
				continue;

			const PointT& pt = cloud_->points[i];
			if (!pt_has_nan(pt))
			{
				// check occlusion
				bool is_occ = tm_.isPointOccluded(rot_cloud->points[i], octree);
				if (!is_occ)
				{
					uv_idx_map[y][x].push_back(i);

					// TODO: only update pixel to closest point 
					auto& px = proj_img.at<cv::Vec3b>(y,x);
					px[0] = pt.b;
					px[1] = pt.g;
					px[2] = pt.r;
				}
			}
		}
	}

private:
	template <typename PT>
	static inline bool pt_has_nan(const PT& pt)
	{
		return std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z);
	}

	inline void compute_A()
	{
		A_ << f_, 0, cx_, 0, f_, cy_, 0, 0, 1;
	}
	inline void compute_T()
	{
		for (int i = 0; i < 3; ++i)
		{
			T_(i,0) = camera_pose_(i,0);
			T_(i,1) = camera_pose_(i,1);
			T_(i,2) = camera_pose_(i,2);
			T_(i,3) = camera_pose_(i,3);
		}
	}
	inline void compute_P()
	{
		const int N = cloud_->size();

		P_.resize(4, N);
		for (int i = 0; i < N; ++i)
		{
			const PointT& pt = cloud_->points[i];
			P_(0,i) = pt.x;
			P_(1,i) = pt.y;
			P_(2,i) = pt.z;
			P_(3,i) = 1;
		}
	}

	template <typename Pt>
	inline double get_pt_distance(const Pt& pt)
	{
		return Eigen::Vector3f(pt.x, pt.y, pt.z).squaredNorm();
	}

	float f_; 
	float cx_;
	float cy_;	

	int image_height_ = 800;
	int image_width_ = 800;

	Eigen::Matrix4f camera_pose_;
	PointCloudConstPtr cloud_;

	// computed 
	Eigen::Matrix3f A_;
	Eigen::MatrixXf T_;
	Eigen::MatrixXf P_;

	// state
	bool has_cloud = false;

	// algorithms
	typename pcl::TextureMapping<PointT> tm_;
};

#endif
