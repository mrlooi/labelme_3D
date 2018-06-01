#ifndef PCL_VIEWER_CUSTOM_HH
#define PCL_VIEWER_CUSTOM_HH

#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/pcl_visualizer.h>

class PCLInteractorCustom: public pcl::visualization::PCLVisualizerInteractorStyle
{
public:
	static PCLInteractorCustom *New ();
	// static PCLInteractorCustom

	PCLInteractorCustom(): pcl::visualization::PCLVisualizerInteractorStyle()
	{
			
	}

protected:
	void OnKeyDown () override;
	void OnChar () override;
};

class PCLVisCustom: public pcl::visualization::PCLVisualizer
{
public:
	PCLVisCustom(pcl::visualization::PCLVisualizerInteractorStyle* style): 
		pcl::visualization::PCLVisualizer(d_argc_, d_argv_, "", style, true)
	{

	}
	PCLVisCustom(int argc, char** argv, pcl::visualization::PCLVisualizerInteractorStyle* style): 
		pcl::visualization::PCLVisualizer(argc, argv, "", style, true)
	{
	}

private:
	int d_argc_; 
	char** d_argv_;
};


#endif