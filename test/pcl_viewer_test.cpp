#include <iostream>

#include <pcl/io/pcd_io.h>

#include "pcl_viewer_custom.hh"


#define PRINT(a) std::cout << #a << ": " << a << std::endl;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::visualization::PCLVisualizerInteractorStyle PVIStyle;


int main(int argc, char *argv[])
{
	std::string pcd_file;
	if (argc <= 1)
	{
		printf("Usage: %s pcd_file\n", argv[0]);
		return -1;
	}
	pcd_file = argv[1];

	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

	if ( pcl::io::loadPCDFile <PointT> (pcd_file, *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	// PCLInteractorCustom style; // * style = PCLInteractorCustom::New(); 
	// vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle> style = pcl::visualization::PCLVisualizerInteractorStyle::New();
	vtkSmartPointer<PCLInteractorCustom> style = vtkSmartPointer<PCLInteractorCustom>::New();
	// pcl::visualization::PCLVisualizerInteractorStyle::Ptr style 
	pcl::visualization::PCLVisualizer::Ptr viewer(new PCLVisCustom(style));
	viewer->addCoordinateSystem();
	viewer->addPointCloud(cloud);
	viewer->spin();
	return 0;
}