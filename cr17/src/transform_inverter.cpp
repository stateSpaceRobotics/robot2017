// Inverts stamped transform message and republishes

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"

#include <string>

class TransformInverter{

public:

  ros::NodeHandle n_;
  ros::Publisher inverseTransform_pub_;
  ros::Subscriber transform_sub_;
  std::string subscriber_topic;
  std::string publisher_topic;

  TransformInverter(ros::NodeHandle n) : 
    n_(n)
  {
    if (!n_.getParam("transform_topic", subscriber_topic))
    {
        ROS_ERROR("Failed to get param 'transform_topic'");
    }
    if (!n_.getParam("inverseTransform_topic", publisher_topic))
    {
        ROS_ERROR("Failed to get param 'inverseTransform_topic'");
    }

    transform_sub_ = n_.subscribe(subscriber_topic, 1, &TransformInverter::transformCallback, this);
    inverseTransform_pub_ = n_.advertise<geometry_msgs::TransformStamped>(publisher_topic, 1);
  }

  void transformCallback (const geometry_msgs::TransformStamped::ConstPtr& msg)
  {
    geometry_msgs::TransformStamped new_msg;
    tf::StampedTransform sT;
    
    tf::transformStampedMsgToTF(*msg, sT);
    sT = tf::StampedTransform(sT.inverse(), sT.stamp_, sT.child_frame_id_, sT.frame_id_);
    tf::transformStampedTFToMsg(sT, new_msg);

    inverseTransform_pub_.publish(new_msg);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "transform_inverter");
  ros::NodeHandle n;
  TransformInverter inverter(n);
  
  ros::spin();
  
  return 0;
}
