#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//Filter
#include <pcl/filters/passthrough.h>
//Transformation & Rotation
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

//Global Data initialization
using namespace std;

//pointer to save cloud data
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

/*-----------------------------------------------------------------------------------------*/

//Helper functions

//1. To load PCD file
int loadPCDFile(const string &filePath){
	//checks for presence of PCD file
	if(pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, *cloud) == -1){
		PCL_ERROR ("Couldn't read file pcd file in the given path \n File Path \t: %s",filePath);
		return (-1);
	}
	//cloud points are loaded
	return (1);
}

//2. To apply pass through filter
pcl::PointCloud<pcl::PointXYZ> passThroughFilter(string axis, float min, float max){
	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName (axis);
	pass.setFilterLimits (min, max);
	//pass.setFilterLimitsNegative (true);
	pass.filter (cloud_filtered);
	return (cloud_filtered);
}

//3. To perform translation and rotation 
int transformation (){
	float theta = M_PI;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	*source_cloud= *cloud;
	
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.translation() << 0.0, 80.0, 0.0;
	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
	
	printf ("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;
	
	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
	
	// Visualization
	printf(  "\nPoint cloud colors :  white  = original point cloud\n"
			"                        red  = transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
	
	 // Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
	viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}
	
	return 0;
}

/*------------------------------------------------------------------------------------------*/


int
main ()
{
	loadPCDFile("../cloud-1000.pcd");
	
	std::cout << "Loaded "
			<< cloud->width * cloud->height
            << " data points from input file with the following fields: "
            << std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile<pcl::PointXYZI> ("../cloud-1000.pcd", *cloud1);
	pcl::io::savePCDFileASCII ("../op.pcd",*cloud1);
	//pcl::io::savePCDFileASCII ("../op.pcd", passThroughFilter("x",0.0,30));
	//transformation();
	return (0);
}



