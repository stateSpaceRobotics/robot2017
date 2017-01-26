#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>

int main(int argc, char **argv)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2::Ptr blob (new pcl::PCLPointCloud2);

	pcl::PCDReader reader;

	reader.read("/home/lee/Downloads/table_scene.pcd", *blob);

  	/*cloud->width  = 0;
  	cloud->height = 0;
  	cloud->points.resize (cloud->width * cloud->height);*/

	ros::init(argc,argv, "objectTester");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("cloud",1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while(ros::ok())
	{
		sensor_msgs::PointCloud2 output;
		pcl_conversions::fromPCL(*blob,output);
		pub.publish(output);
	}
	return 0;
}
