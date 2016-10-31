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
#include <pcl/common/impl/eigen.hpp>
#include <pcl/segmentation/extract_clusters.h>

pcl::PointCloud<pcl::PointXYZ> processData();

bool newData = false;
pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

ros::Publisher pub;
void cloud_input(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl_conversions::toPCL(*input, *cloud);
	newData = true;
	//pcl::PointCloud<pcl::PointXYZ> data = processData();
	//sensor_msgs::PointCloud2 output;
	//pcl::toROSMsg(data,output);
	//pub.publish(output);
}

pcl::PointCloud<pcl::PointXYZ> processData()
{
	pcl::PCLPointCloud2* tmpCloud = new pcl::PCLPointCloud2(*cloud);
	pcl::PCLPointCloud2ConstPtr cloudPtr(tmpCloud);
	pcl::PCLPointCloud2 cloud_filtered;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3d (new pcl::PointCloud<pcl::PointXYZ>),
						projCloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize(0.1,0.1,0.1);
	sor.filter(cloud_filtered);

	pcl::fromPCLPointCloud2(cloud_filtered,*cloud3d);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(1.0);
	seg.setAxis(Eigen::Vector3f(0,0,1));

	seg.setInputCloud(cloud3d);
	seg.segment(*inliers, *coefficients);

	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setIndices(inliers);
	proj.setInputCloud(cloud3d);
	proj.setModelCoefficients(coefficients);
	proj.filter(*projCloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(projCloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(projCloud);
	ec.extract(cluster_indices);
	
	int j = 0;
	pcl::PointCloud<pcl::PointXYZ> objects;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster->points.push_back(projCloud->points[*pit]);
		}
		pcl::PointXYZ *median;
		if(cloud_cluster->points.size()%2==0)
		{
			int index = floor(cloud_cluster->points.size()/2);
			pcl::PointXYZ temp = cloud_cluster->points[index], temp2 = cloud_cluster->points[index+1];
			int x,y,z;
			x = (temp.x + temp2.x)/2;
			y = (temp.y + temp2.y)/2;
			z = (temp.z + temp2.z)/2;
			median = new pcl::PointXYZ(x,y,z);
			objects.points.push_back(*median);

		} else {
			*median = cloud_cluster->points[cloud_cluster->points.size()/2];
			objects.points.push_back(*median);
		}
	}
	return objects;
}

int main(int argc, char ** argv)
{

	ros::init(argc,argv,"plane_detector");
	ros::NodeHandle nh;

	pub = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);

	ros::Subscriber sub = nh.subscribe("cloud", 1, cloud_input);

	while(ros::ok())
	{
		if(newData)
		{
			newData = false;
			pcl::PointCloud<pcl::PointXYZ> data = processData();
			sensor_msgs::PointCloud2 output;
			pcl_conversions::fromPCL(*cloud,output);
			pcl::toROSMsg(data,output);
			pub.publish(output);
		}
		ros::spin();
	}

	return 0;
}
