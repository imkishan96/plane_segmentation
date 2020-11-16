#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_imu_base");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  ros::Rate rate(100.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    q.setRPY(-1.57,0.0,-1.57);
    transform.setRotation( q );
    br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "camera_imu_optical_frame", "base_link"));
    rate.sleep();
  }
  return 0;
};