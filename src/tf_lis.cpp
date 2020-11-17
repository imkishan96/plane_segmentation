#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle nh;

  tf::TransformListener listener;
  ros::Publisher pub;
  visualization_msgs::Marker marker;
  pub = nh.advertise<visualization_msgs::Marker>("raw_normal_marker", 1);
  tf::TransformBroadcaster br;

  ros::Rate rate(400.0);
  while (nh.ok()){
    tf::StampedTransform transform, transform_imu;
    try{
      listener.lookupTransform("/camera_imu_optical_frame", "/plane_normal_tf",
                               ros::Time(0), transform);
      listener.lookupTransform("/odom", "/camera_imu_optical_frame",
                               ros::Time(0), transform_imu);    
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    tf::Quaternion q_imu, q_imu_odom, q_marker;
    tf::Transform tf_imu_odom ;

    q_imu = transform_imu.getRotation();

    double roll, pitch, yaw;
    tf::Matrix3x3(q_imu).getRPY(roll, pitch, yaw);

    q_imu_odom.setRPY(roll,pitch,0.0);

    tf_imu_odom.setOrigin(transform_imu.inverse().getOrigin());
    tf_imu_odom.setRotation(q_imu_odom.inverse());
    br.sendTransform(tf::StampedTransform(tf_imu_odom, ros::Time::now(), "camera_imu_optical_frame", "new_odom"));


    tf::Transform tf_marker_new_odom;
    tf_marker_new_odom = transform.inverse() * tf_imu_odom ;
    q_marker = tf_marker_new_odom.inverse().getRotation();

    marker.header.frame_id = "/new_odom";
    marker.header.stamp = ros::Time();
    marker.ns = "normal_for_new_odom";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = (double) q_marker.getX() ;
    marker.pose.orientation.y = (double) q_marker.getY() ;
    marker.pose.orientation.z = (double) q_marker.getZ() ;
    marker.pose.orientation.w = (double) q_marker.getW() ;


    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    pub.publish(marker);

    tf::Vector3 start_vector, end_vector;
    start_vector = {1.0, 0.0, 0.0};
    tf::Transform new_tf;
    new_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
    new_tf.setRotation(q_marker);
    end_vector = new_tf * start_vector ;
    double x,y,z;

    x = (double) end_vector.getX();
    y = (double) end_vector.getY();
    z = (double) end_vector.getZ();
    double ax = atan2(sqrt(y * y + z * z ),x);
    double ay = atan2(sqrt(z * z + x * x ),y);
    double az = atan2(sqrt(x * x + y * y ),z);
    ROS_INFO("X:%f, Y:%f, Z:%f, AX:%f, AZ:%f ", x ,y, z, ax,az);

    rate.sleep();
  }
  return 0;
};
