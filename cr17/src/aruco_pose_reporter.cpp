// Inverts and republishes the pose output by aruco
// Inversion is needed as aruco outputs pose of the board
// with respect to the camera.
// Pose is published in the camera frame

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"

#include <tf/transform_listener.h>

#include <string>

class PoseReporter{

public:

  ros::NodeHandle n_;
  ros::NodeHandle n;
  ros::Publisher pose_pub_;
  ros::Subscriber transform_sub_;
  
  tf::TransformListener listener;

  std::string subscriber_topic;
  std::string publisher_topic;
  std::string target_frame;
  std::string world_frame;

  PoseReporter() : 
  n_("~"),
  n()
  {
    if (!n_.getParam("transform_topic", subscriber_topic)){
        ROS_ERROR("Failed to get param 'transform_topic'");
    } else if (!n_.getParam("pose_topic", publisher_topic)){
        ROS_ERROR("Failed to get param 'pose_topic'");
    } else if (!n_.getParam("target_frame", target_frame)){
        ROS_ERROR("Failed to get param 'target_frame'");
    } else if (!n_.getParam("world_frame", world_frame)){
        ROS_ERROR("Failed to get param 'world_frame'");
    }else {

      // publisher for pose with covariance stamped
      pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(publisher_topic, 1);
      // subscriber to the transform from aruco
      transform_sub_ = n.subscribe(subscriber_topic, 1, &PoseReporter::transformCallback, this);
    }
  }

  void transformCallback (const geometry_msgs::TransformStamped::ConstPtr& msg)
  {
    // msg: camera->board
    tf::StampedTransform sT;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseWithCovarianceStamped new_msg;
    tf::StampedTransform camera_to_target_transform;
    tf::StampedTransform world_to_board_transform;

    try{
      // world->board
      listener.lookupTransform(world_frame, msg->child_frame_id, ros::Time(0), world_to_board_transform);
      
      // camera->target_frame
      listener.lookupTransform(msg->header.frame_id, target_frame, ros::Time(0), camera_to_target_transform);
      
    } catch (tf::TransformException ex) {
      ROS_ERROR("Transforms unavailable!\n%s", ex.what());
      ROS_INFO("world: %s", world_frame.c_str());
      ROS_INFO("target: %s", target_frame.c_str());

    }

    // Convert to a transform: board->camera
    tf::transformStampedMsgToTF(*msg, sT);
    // world->board * board->camera * camera->target_frame
    tf::Transform transform = world_to_board_transform * sT.inverse() * camera_to_target_transform;
    // Invert to board->camera
    tf::Stamped<tf::Transform> tf_pose(transform, sT.stamp_, world_frame);
    // Convert to pose
    tf::poseStampedTFToMsg (tf_pose, pose);

    new_msg.pose.covariance[0] = -1;
    new_msg.header = pose.header;
    new_msg.pose.pose = pose.pose;

    pose_pub_.publish(new_msg);

  }

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "transform_inverter");
  ROS_INFO("Started node");
  PoseReporter inverter;
  ROS_INFO("Started class");
  ros::spin();
  ROS_INFO("Exiting");
  
  return 0;
}
