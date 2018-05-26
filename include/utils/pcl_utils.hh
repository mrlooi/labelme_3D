/**
 * Written by:
 * Vincent, Keefe, Aditya
 */

#ifndef VISION_PCL_UTILS_HH
#define VISION_PCL_UTILS_HH


#include <Eigen/Eigenvalues> 

// pcl core
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

// passthrough
#include <pcl/filters/passthrough.h>

// clipper
#include <pcl/filters/plane_clipper3D.h>

// extract indices
#include <pcl/filters/extract_indices.h>

// normal est
#include <pcl/features/normal_3d_omp.h>

// PCL downsampling
#include <pcl/filters/voxel_grid.h>

namespace PclUtils
{
// public:
    // PclUtils()
    // {}
    // ~PclUtils()
    // {}

/**
 * @brief      Removes the pointcloud data which have NaN values.
 *
 * @param      cloud      The input cloud
 * @param      out_cloud  The output cloud
 * @return     The indices of valid points in the pointcloud data.
 */
template <typename PointT>
static std::vector<int> FilterCloudNan(const pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>& out_cloud)
{
    const int cloud_sz = cloud.points.size();

    auto& out_pts = out_cloud.points;
    out_pts.clear();
    out_pts.reserve(cloud_sz);

    std::vector<int> good_indices;

    for (unsigned int i = 0; i < cloud_sz; i++)
    {
        const PointT& pt = cloud.points[i];

        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
        {
            out_pts.push_back(pt);
            good_indices.push_back(i);
        }
    }
    return good_indices;
}

/**
 * @brief      Computes the average (centroid) 3D point value in a container data structure
 *
 * @param      points      Container of 3D points xyz data e.g. vector, list, set, etc
 * @return     The average xyz values as pcl::PointXYZ
 */
template <typename Container>
static pcl::PointXYZ ComputePointsAverage(const Container& points)
{
    const size_t sz = points.size();
    if (sz == 0)
        return {0,0,0};
    
    float x = 0;
    float y = 0;
    float z = 0;

    for (size_t i = 0; i < sz; ++i)
    {
        const auto& pt = points[i];
        x += pt.x;
        y += pt.y;
        z += pt.z;
    }
    x /= sz;
    y /= sz;
    z /= sz;
    return {x,y,z};
}

/**
 * @brief      Computes the average point value in a cloud
 *
 * @param      cloud      The input cloud
 * @return     The average xyz values as pcl::PointXYZ
 */
template <typename PointT>
static pcl::PointXYZ ComputeCloudAverage(const pcl::PointCloud<PointT>& cloud)
{
    return ComputePointsAverage(cloud.points);
}

/**
 * @brief      Transforms a point based on a transform
 *
 */   
template <typename PointT>
inline static void TransformPoint(const PointT& in_pt, PointT& out_pt, const Eigen::Matrix4f &transform)
{
    Eigen::Matrix<float, 3, 1> pt (in_pt.x, in_pt.y, in_pt.z);
    out_pt.x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
    out_pt.y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
    out_pt.z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
}

template <typename Container>
static void TransformPoints(const Container& in_data, Container& out_data, const Eigen::Matrix4f &transform)
{
    if (&in_data != &out_data)
    {
        out_data.resize(in_data.size());
    }
    for (size_t i = 0; i < in_data.size(); ++i)
    {
        TransformPoint(in_data[i], out_data[i], transform);
    }
}

/**
 * @brief      Transforms a normal based on a transform
 *
 */    
static void TransformNormal(const pcl::Normal& in_n, pcl::Normal& out_n, const Eigen::Matrix4f &transform)
{
    pcl::PointXYZ in_pt {in_n.normal_x, in_n.normal_y, in_n.normal_z};
    pcl::PointXYZ out_pt;

    TransformPoint(in_pt, out_pt, transform);

    out_n.normal_x = out_pt.x;
    out_n.normal_y = out_pt.y;
    out_n.normal_z = out_pt.z;
}

template <typename Container>
static void TransformNormals(const Container& in_data, Container& out_data, const Eigen::Matrix4f &transform)
{
    if (&in_data != &out_data)
    {
        out_data.resize(in_data.size());
    }
    for (size_t i = 0; i < in_data.size(); ++i)
    {
        TransformNormal(in_data[i], out_data[i], transform);
    }
}

/**
 * @brief      Computes the 3D min max in a container data structure
 *
 * @param      points      Container of 3D points xyz data e.g. vector, list, set, etc
 * @param      min_pt      output min 3D xyz of data
 * @param      max_pt      output max 3D xyz of data
 */
template <typename T, typename Container>
static void ComputePointsMinMax(const Container& pts, T& min_pt, T& max_pt)
{
    if (pts.size() == 0)
        return;

    max_pt.x = pts[0].x; 
    min_pt.x = max_pt.x;
    max_pt.y = pts[0].y; 
    min_pt.y = max_pt.y;
    max_pt.z = pts[0].z; 
    min_pt.z = max_pt.z;

    for(int i = 1; i < pts.size(); ++i)
    {
        if (pts[i].x > max_pt.x) max_pt.x = pts[i].x;
        else if (pts[i].x < min_pt.x) min_pt.x = pts[i].x;
        if (pts[i].y > max_pt.y) max_pt.y = pts[i].y;
        else if (pts[i].y < min_pt.y) min_pt.y = pts[i].y;
        if (pts[i].z > max_pt.z) max_pt.z = pts[i].z;
        else if (pts[i].z < min_pt.z) min_pt.z = pts[i].z;
    }
}


/**
 * @brief      returns the indices of the input array of pcl cloud ptr sorted by cloud size
 *
 * @param      v            vector of pcl::PointCloud::Ptr
 * @param      ascending    sort by size in ascending order (default false)
 * @return     the subset cloud relating to the indices of the input cloud 
 */
template <typename CloudPtrContainer>
static std::vector<size_t> SortIndicesByCloudsSize(const std::vector<CloudPtrContainer> &v, bool ascending=false) 
{
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  if (ascending)
  {
      std::sort(idx.begin(), idx.end(),
           [&v](size_t i1, size_t i2) {return v[i1]->size() < v[i2]->size();});
  } else 
  {
      std::sort(idx.begin(), idx.end(),
           [&v](size_t i1, size_t i2) {return v[i1]->size() > v[i2]->size();});
  }

  return idx;
}


/**
 * @brief      get the subset cloud relating to the indices of the input cloud 
 *
 * @param      extractor  pcl::ExtractIndices<PointT> type
 * @param      cloud      input cloud
 * @param      indices    input indices (pcl::PointIndices::Ptr)
 * @param      negative   input bool: if true, gets the cloud outside the indices. if false (default), gets cloud inside the indices
 * @return     the subset cloud relating to the indices of the input cloud 
 */
template <typename PointT>
static typename pcl::PointCloud<PointT>::Ptr GetCloudByIndices(pcl::ExtractIndices<PointT>& extractor, typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::PointIndices::ConstPtr indices, bool negative=false)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);    

    extractor.setInputCloud (cloud);
    extractor.setIndices (indices);
    extractor.setNegative (negative);
    extractor.filter(*cloud_filtered);
    return cloud_filtered;
}

/**
 * @brief      get the subset cloud relating to the indices of the input cloud 
 *
 * @param      cloud      input cloud
 * @param      indices    input indices (pcl::PointIndices::Ptr)
 * @param      negative   input bool: if true, gets the cloud outside the indices. if false (default), gets cloud inside the indices
 * @return     the subset cloud relating to the indices of the input cloud 
 */
template <typename PointT>
static typename pcl::PointCloud<PointT>::Ptr GetCloudByIndices(typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::PointIndices::ConstPtr indices, bool negative=false)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);    
    pcl::ExtractIndices<PointT> extractor;
    return GetCloudByIndices<PointT>(extractor, cloud, indices, negative);
}

/**
 * @brief      get the subset cloud relating to the indices of the input cloud 
 *
 * @param      extractor  pcl::ExtractIndices<PointT> type
 * @param      cloud      input cloud
 * @param      indices    input indices (pcl::IndicesPtr)
 * @param      negative   input bool: if true, gets the cloud outside the indices. if false (default), gets cloud inside the indices
 * @return     the subset cloud relating to the indices of the input cloud 
 */
template <typename PointT>
static typename pcl::PointCloud<PointT>::Ptr GetCloudByIndices(pcl::ExtractIndices<PointT>& extractor, typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::IndicesPtr indices, bool negative=false)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);    

    extractor.setInputCloud (cloud);
    extractor.setIndices (indices);
    extractor.setNegative (negative);
    extractor.filter(*cloud_filtered);
    return cloud_filtered;
}

/**
 * @brief      get the subset cloud relating to the indices of the input cloud 
 *
 * @param      cloud      input cloud
 * @param      indices    input indices (pcl::IndicesPtr)
 * @param      negative   input bool: if true, gets the cloud outside the indices. if false (default), gets cloud inside the indices
 * @return     the subset cloud relating to the indices of the input cloud 
 */
template <typename PointT>
static typename pcl::PointCloud<PointT>::Ptr GetCloudByIndices(typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::IndicesPtr indices, bool negative=false)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);    
    pcl::ExtractIndices<PointT> extractor;
    return GetCloudByIndices<PointT>(extractor, cloud, indices, negative);
}

template <typename PointT>
static void GetCloudByIndices(typename pcl::PointCloud<PointT>::Ptr out_cloud, typename pcl::PointCloud<PointT>::ConstPtr cloud, const pcl::IndicesPtr indices, bool negative=false)
{
    if (cloud->size() == 0)
        return;
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud (cloud);
    extractor.setIndices (indices);
    extractor.setNegative (negative);
    extractor.filter(*out_cloud);
}

/**
 * @brief      get the subset cloud relating to the indices of the input cloud 
 *
 * @param      cloud      input cloud
 * @param      indices    container of data storing the indices (e.g. vector, set, etc)
 * @param      out_cloud  output cloud (indices cloud)
 */
template <typename PointT, typename Container>
static void GetCloudByIndices(const pcl::PointCloud<PointT>& cloud, const Container& indices, pcl::PointCloud<PointT>& out_cloud)
{
    size_t sz = indices.size();
    out_cloud.points.reserve(sz);
    for (size_t i = 0; i < sz; ++i)
    {
        out_cloud.points.push_back(cloud.points[indices[i]]);
    }
}


template <typename PointT>
static void CopyCloudToVector(const pcl::PointCloud<PointT>& cloud, std::vector<PointT>& v)
{
    v.reserve(cloud.size());
    for (int i = 0; i < cloud.size(); ++i)
    {
        v.push_back(cloud.points[i]);
    }
}


/**
 * @brief      get indices of points in the cloud that are within a 3D bounding box
 *
 * @param      cloud      The input cloud
 * @param      params     Parameters of the 3D Bounding box (min_x,max_x,min_y,max_y,min_z,max_z) as a vector of size 6
 * @return     The indices of points within the 3D bounding box.
 */
template <typename PointT, typename T>
static pcl::IndicesPtr GetROIIndices(typename pcl::PointCloud<PointT>::ConstPtr cloud, const std::vector<T>& params)
{
    assert(params.size() == 6);

    pcl::IndicesPtr indices (new std::vector <int>);
    // pcl::IndicesPtr indices_x (new std::vector <int>);
    // pcl::IndicesPtr indices_xy (new std::vector <int>);
    pcl::PassThrough<PointT> pass;

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (params[0], params[1]);
    pass.filter (*indices);

    pass.setIndices (indices);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (params[2],params[3]);
    pass.filter (*indices);

    pass.setIndices (indices);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (params[4], params[5]);
    pass.filter (*indices);

    return indices;
}

/**
 * @brief      get points in the cloud that are within a 3D bounding box
 *
 * @param      cloud      The input cloud
 * @param      params     Parameters of the 3D Bounding box (min_x,max_x,min_y,max_y,min_z,max_z) as a vector of size 6
 * @return     The points within the 3D bounding box.
 */
template <typename PointT, typename T>
static typename pcl::PointCloud<PointT>::Ptr GetROICloud(typename pcl::PointCloud<PointT>::ConstPtr cloud, const std::vector<T>& params)
{
    auto roi_indices = GetROIIndices<PointT>(cloud, params);
    return GetCloudByIndices<PointT>(cloud, roi_indices);
}

/**
 * @brief      convert euler angles to quaternion
 *
 * @param      x axis angle
 * @param      y axis angle
 * @param      z axis angle
 * @return     A vector of size 4 representing the quaternion (x,y,z,w)
 */
template <typename T>
inline static std::vector<T> Euler2Quaternion(const T& nx, const T& ny, const T& nz)
{
    auto x_angle = Eigen::AngleAxisf(nx, Eigen::Vector3f::UnitX());
    auto y_angle = Eigen::AngleAxisf(ny, Eigen::Vector3f::UnitY());
    auto z_angle = Eigen::AngleAxisf(nz, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q = x_angle * y_angle * z_angle;
    return {q.x(), q.y(), q.z(), q.w()};
}

/**
 * @brief      convert quaternion to euler
 *
 * @param      x
 * @param      y
 * @param      z
 * @param      w
 * @return     A vector of size 3 representing euler angles (x,y,z)
 */
inline static std::vector<float> Quaternion2Euler(const float& x, const float& y, const float& z, const float& w)
{
    Eigen::Quaternionf q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    return {euler.x(),euler.y(),euler.z()};
}

/**
 * @brief      transform the input cloud based on the input transformation matrix
 *
 * @param      cloud       input cloud
 * @param      transform   transformation matrix (Eigen::Matrix4f)
 */
template <typename PointT>
static inline void TransformPointCloud(pcl::PointCloud<PointT>& cloud, const Eigen::Matrix4f& transform)
{
    pcl::transformPointCloud (cloud, cloud, transform);
}

/**
 * @brief      find the pose (translation, quaternion) of a pointcloud
 *
 * @param      cloud      input cloud
 * @param      out_transform     output translation
 * @param      out_quarternion   output quaternion
 * @param      min point (in terms of xyz) of the cloud 
 * @param      max point (in terms of xyz) of the cloud
*/
template <typename PointT>
static void FindCloudPose(const pcl::PointCloud<PointT>& cloud, Eigen::Vector3f& out_transform, Eigen::Quaternionf& out_quarternion, PointT& out_min_pt, PointT& out_max_pt)
{
    // //calculate smallest bounding box
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    //
    //        // Transform the original cloud to the origin where the principal components correspond to the axes.
    const Eigen::Matrix4f mat4f_identity = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projectionTransform(mat4f_identity);
    //
    projectionTransform.block(0,0,3,3) = eigenVectorsPCA.transpose();
    projectionTransform.block(0,3,3,1) = -1.f * (projectionTransform.block(0,0,3,3) * pcaCentroid.head(3));

    pcl::PointCloud<PointT> cloudPointsProjected;

    pcl::transformPointCloud(cloud, cloudPointsProjected, projectionTransform);
    //
    //        // Get the minimum and maximum points of the transformed cloud.
    pcl::getMinMax3D(cloudPointsProjected, out_min_pt, out_max_pt);
    const Eigen::Vector3f meanDiagonal = 0.5f*(out_max_pt.getVector3fMap() + out_min_pt.getVector3fMap());
    //
    //        // Final transform & quarternion
    out_transform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head(3);
    out_quarternion = Eigen::Quaternionf(eigenVectorsPCA);

}

/**
 * @brief      find the pose (translation, quaternion) of a pointcloud
 *
 * @param      cloud      input cloud
 * @param      out_transform     output translation
 * @param      out_quarternion   output quaternion
 * @return     A vector of size 2 consisting of: min and max points (xyz) of the cloud 
 */
template <typename PointT>
static std::vector<PointT> FindCloudPose(const pcl::PointCloud<PointT>& cloud, Eigen::Vector3f& out_transform, Eigen::Quaternionf& out_quarternion)
{
    std::vector<PointT> min_max_pts(2);
    FindCloudPose(cloud, out_transform, out_quarternion, min_max_pts[0], min_max_pts[1]);

    return min_max_pts;
}

/**
 * @brief      calculate the median normal of a pointcloud of normals
 *
 * @param      normals      input normal cloud
 * @return     median of normals (pcl::Normal)
 */
static pcl::Normal CalculateMedianNormal(pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
    std::vector<float> normal_x, normal_y, normal_z;
    for (int it = 0; it < normals->points.size(); it++)
    {
        if (std::isnan(normals->points[it].normal_x) ||
            std::isnan(normals->points[it].normal_y) ||
            std::isnan(normals->points[it].normal_z))
        {
            continue;
        }
        normal_x.push_back(normals->points[it].normal_x);
        normal_y.push_back(normals->points[it].normal_y);
        normal_z.push_back(normals->points[it].normal_z);
    }

    std::nth_element(normal_x.begin(), normal_x.begin() + (int)(normal_x.size() / 2), normal_x.end());
    std::nth_element(normal_y.begin(), normal_y.begin() + (int)(normal_y.size() / 2), normal_y.end());
    std::nth_element(normal_z.begin(), normal_z.begin() + (int)(normal_z.size() / 2), normal_z.end());

    pcl::Normal ret;
    if(!normal_x.empty()){
        ret.normal_x = normal_x[(int)(normal_x.size() / 2)];
        ret.normal_y = normal_y[(int)(normal_y.size() / 2)];
        ret.normal_z = normal_z[(int)(normal_z.size() / 2)];
    }

    return ret;
}

/**
 * @brief      calculate the median normal of a pointcloud
 *
 * @param      cloud      input cloud
 * @return     median of normals (pcl::Normal) of the input pointcloud
 */
template <typename PointT>
static typename pcl::Normal CalculateMedianNormal(typename pcl::PointCloud<PointT>::ConstPtr cloud, float radius_search=0.2, bool downsample=true)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    typename pcl::PointCloud<PointT>::Ptr p_cloud(new pcl::PointCloud<PointT>);
    *p_cloud += *cloud;

    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;

    if (downsample)
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (p_cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*p_cloud);            
    }

    ne.setInputCloud(p_cloud);
    typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(kdtree);
    ne.setRadiusSearch(radius_search);
    ne.compute(*normals);

    return CalculateMedianNormal(normals);
}

/**
 * @brief      calculate the hyperplane coefficients of a point and normal vector
 *
 * @param      normal      The normal struct (pcl::Normal)
 * @param      point       The point struct (e.g. pcl::PointXYZ)
 * @return     The model coefficients of the hyperplane 
 */
template <typename PointT>
static pcl::ModelCoefficients::Ptr CalculatePlaneCoefficients(const pcl::Normal normal, const PointT point)
{
    float normal_vector_size = std::sqrt(normal.normal_x * normal.normal_x + 
                                         normal.normal_y * normal.normal_y + 
                                         normal.normal_z * normal.normal_z);
    pcl::ModelCoefficients::Ptr ret(new pcl::ModelCoefficients);
    ret->values.resize(4);
    ret->values[0] = normal.normal_x / normal_vector_size;
    ret->values[1] = normal.normal_y / normal_vector_size;
    ret->values[2] = normal.normal_z / normal_vector_size;
    ret->values[3] = -(ret->values[0] * point.x + ret->values[1] * point.y + ret->values[2] * point.z);
    return ret;
}

/**
 * @brief      calculate the hyperplane coefficients of a pointnormal vector
 *
 * @param      pt_normal      pcl::PointNormal struct
 * @return     The model coefficients of the hyperplane 
 */
template <typename PointNormal>
static pcl::ModelCoefficients::Ptr CalculatePlaneCoefficients(const PointNormal& pt_normal)
{
    return CalculatePlaneCoefficients({pt_normal.normal_x,pt_normal.normal_y,pt_normal.normal_z}, {pt_normal.x,pt_normal.y,pt_normal.z});
}


/**
 * @brief      get the indices of the points in the cloud that are not affected by the plane clipping operation 
 *
 * @param      cloud      input cloud
 * @param      plane_vector    input vector of the plane used for clipping the cloud
 * @return     the indices of the points of the cloud not affected by the plane clipping operation 
 */
template <typename PointT>
static pcl::PointIndices::Ptr GetClippedCloudIndices(typename pcl::PointCloud<PointT>::ConstPtr cloud, const Eigen::Vector4f& plane_vector)
{
    pcl::PlaneClipper3D<PointT> clipper(plane_vector);
    std::vector<int> clip_indices;
    clipper.clipPointCloud3D(*cloud, clip_indices);
    pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
    indices_ptr->indices = clip_indices;

    return indices_ptr;
}

/**
 * @brief      get the points in the cloud that are not affected by the plane clipping operation 
 *
 * @param      cloud      input cloud
 * @param      plane_vector    input vector of the plane used for clipping the cloud
 * @return     the points of the cloud not affected by the plane clipping operation 
 */
template <typename PointT>
static typename pcl::PointCloud<PointT>::Ptr GetClippedCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud, const Eigen::Vector4f& plane_vector)
{
    auto indices_ptr = GetClippedCloudIndices<PointT>(cloud, plane_vector);

    return GetCloudByIndices<PointT>(cloud, indices_ptr, false);
}

/**
 * @brief      get the points in the cloud that are not affected by the plane clipping operation 
 *
 * @param      cloud          input cloud
 * @param      plane_coeff    input plane coefficients (e.g. generated by CalculatePlaneCoefficients)
 * @return     the points of the cloud not affected by the plane clipping operation 
 */
template <typename PointT>
static typename pcl::PointCloud<PointT>::Ptr GetClippedCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::ModelCoefficients::ConstPtr plane_coeff)
{
    Eigen::Vector4f plane_vector;
    for(int i = 0; i < 4; i++) 
    {
        plane_vector(i) = plane_coeff->values[i];
    }

    return GetClippedCloud<PointT>(cloud, plane_vector);
}

/**
 * @brief      get the 8 box vertex points from a min point and a max point
 *
 * @param      min_pt         input min point
 * @param      max_pt         input max point
 * @param      box_vertices   output array type that will contain the 8 box vertices from the min max points
 */
template <typename PointT, typename Container>
static void GetBoxVerticesFromMinMax(const PointT& min_pt, const PointT& max_pt, Container& box_vertices)
{
    PointT final_pt1, final_pt2, final_pt3, final_pt4;
    PointT final_pt5, final_pt6, final_pt7, final_pt8;
    final_pt1 = min_pt;
    final_pt7 = max_pt;
    final_pt2 = min_pt;
    final_pt2.x = max_pt.x;
    final_pt3 = min_pt;
    final_pt3.y = max_pt.y;
    final_pt3.x = max_pt.x;
    final_pt4 = min_pt;
    final_pt4.y = max_pt.y;

    final_pt5 = min_pt;
    final_pt5.z = max_pt.z;
    final_pt6 = min_pt;
    final_pt6.x = max_pt.x;
    final_pt6.z = max_pt.z;
    final_pt8 = min_pt;
    final_pt8.y = max_pt.y;
    final_pt8.z = max_pt.z;

    box_vertices.reserve(8);
    box_vertices.push_back(final_pt1);
    box_vertices.push_back(final_pt2);
    box_vertices.push_back(final_pt3);
    box_vertices.push_back(final_pt4);
    box_vertices.push_back(final_pt5);
    box_vertices.push_back(final_pt6);
    box_vertices.push_back(final_pt7);
    box_vertices.push_back(final_pt8);
}

};


#endif //VISION_PCL_UTILS_HH
