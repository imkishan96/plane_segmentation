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

  ros::Rate rate(30.0);
  while (nh.ok()){
    tf::StampedTransform transform, transform_imu;
    try{
      listener.lookupTransform("/odom", "/plane_normal_tf",
                               ros::Time(0), transform);
      listener.lookupTransform("/odom", "/camera_depth_optical_frame",
                               ros::Time(0), transform_imu);    
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    // tf::Vector3 start_vector, end_vector;
    // start_vector = {1.0, 0.0, 0.0};
    // tf::Quaternion q;
    // tf::Transform new_tf;
    // q = transform.getRotation();
    // new_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
    // new_tf.setRotation(q);
    // end_vector = new_tf * start_vector ;
    // double x,y,z;
    // double ax = atan2(sqrt(y * y + z * z ),x);
    // double ay = atan2(sqrt(z * z + x * x ),y);
    // x = (double) end_vector.getX();
    // y = (double) end_vector.getY();
    // z = (double) end_vector.getZ();
    //ROS_INFO("X:%f, Y:%f, Z:%f, AX:%f, AY:%f ", x ,y, z, ax,ay);

    // tf::Quaternion q_imu;
    // q_imu = transform_imu.getRotation();
    // double roll, pitch, yaw;
    // tf::Matrix3x3(q_imu).getRPY(roll, pitch, yaw);
    // tf::Quaternion q_new;
    // q_new.setRPY(roll,pitch, yaw);
    // q_new.normalize();
    // ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", roll, pitch, yaw);

    // tf::Transform tf_odom_new_imu;
    // tf_odom_new_imu.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    // double inv_roll, inv_pitch, inv_yaw;
    // tf::Matrix3x3(q_imu).getRPY(inv_roll, inv_pitch, inv_yaw);
    // tf::Quaternion q_imu_inv;
    // q_imu_inv.setRPY(0.0, 0.0, -inv_yaw);
    // q_imu_inv.normalize();
    //tf_odom_new_imu.setRotation(q_imu_inv);
    //br.sendTransform(tf::StampedTransform(tf_odom_new_imu, ros::Time::now(), "odom", "new_ori"));
    
    tf::Quaternion q_imu = transform_imu.getRotation();
    tf_odom_new_imu.setRotation(q_imu.inverse());
    tf::Transform odom_new_imu_inv;
    odom_new_imu_inv.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q_undo_yaw;
    q_undo_yaw.setRPY(0.0,0.0,0.0);
    odom_new_imu_inv.setRotation(q_undo_yaw);
    br.sendTransform(tf::StampedTransform(tf_odom_new_imu * odom_new_imu_inv, ros::Time::now(), "camera_depth_optical_frame", "new_ori"));

    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time();
    marker.ns = "normal_raw";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = (double) q_new.getX() ;
    marker.pose.orientation.y = (double) q_new.getY() ;
    marker.pose.orientation.z = (double) q_new.getZ() ;
    marker.pose.orientation.w = (double) q_new.getW() ;


    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    pub.publish(marker);
  }
  return 0;
};
