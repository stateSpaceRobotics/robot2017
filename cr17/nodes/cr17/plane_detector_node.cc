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

ros::Publisher pub;
void cloudInput(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3d (new pcl::PointCloud<pcl::PointXYZ>),
						projCloud(new pcl::PointCloud<pcl::PointXYZ>);*/

	pcl_conversions::toPCL(*input, *cloud);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize(0.1,0.1,0.1);
	sor.filter(cloud_filtered);

	/*pcl::fromPCLPointCloud2(cloud_filtered,*cloud3d);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(1.0);

	seg.setInputCloud(cloud3d);
	seg.segment(*inliers, *coefficients);

	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModeltype(pcl::SACMODEL_PLANE);
	proj.setIndices(inliers);
	proj.setInputCloud(cloud3d);
	proj.setModelCoefficients(coefficients);
	proj.filter(*projCloud);*/

	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered,output);

	pub.publish(output);

}

int main(int argc, char ** argv)
{

	ros::init(argc,argv,"plane_detector_node");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("cloud", 1000, cloudInput);

	pub = n.advertise<sensor_msgs::PointCloud2> ("obstacles", 1000);

	while(ros::ok())
	{
		ros::spin();
	}

	return 0;
}
