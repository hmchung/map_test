#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& _scan);
// void scan_head_status_cb(const std_msgs::Int8::ConstPtr& _flag);

bool reset_cloud_flag = false;
int signal_value = 1;
int signal_count = 0;
int signal_trigger_count = 2;

double min_angle = -1.0472;
double max_angle = 1.0472;

bool line_vertical = false;
double temp_slope = 0.0;
double temp_yaw_angle = 0.0;

std::string node_name = "object_orientation_pub";
std::string topic_name_scan = "/scan_head";
std::string topic_name_obj_pose = "/object_orientation";
std::string topic_name_head_status = "/ugv_1/laser_scan_state";

int main(int argc, char** argv)
{
  ros::init (argc, argv, node_name);
  ros::NodeHandle nh;

  ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>
    (topic_name_scan, 10, laser_cb);
  // ros::Subscriber scan_head_status_sub = nh.subscribe<std_msgs::Int8>
  //   (topic_name_head_status, 10, scan_head_status_cb);

  ros::Publisher object_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
    (topic_name_obj_pose, 10);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO("CURRENT YAW: %.3f - SLOPE: %.3f", temp_yaw_angle, temp_slope);

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, temp_yaw_angle);

    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.header.frame_id = "base_frame";

    temp_pose.pose.position.x = 0;
    temp_pose.pose.position.y = 0;
    temp_pose.pose.position.z = 0;
    temp_pose.pose.orientation.x = q.getX();
    temp_pose.pose.orientation.y = q.getY();
    temp_pose.pose.orientation.z = q.getZ();
    temp_pose.pose.orientation.w = q.getW();

    object_pose_pub.publish(temp_pose);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


void laser_cb(const sensor_msgs::LaserScan::ConstPtr& _scan){
  //Get the number of original laser reading
  int originalCount = _scan->ranges.size();

  double pan_angle_min = _scan->angle_min;
  double pan_angle_max = _scan->angle_max;
  double pan_angle_increment = _scan->angle_increment;

  double pan_range_min = _scan->range_min;
  double pan_range_max = _scan->range_max;

  //Regression variables
  double temp_sum_x = 0.0;
  double temp_sum_y = 0.0;
  double temp_sum_xy = 0.0;
  double temp_sum_xx = 0.0;  
  int actual_point_count = 0;

  for (int i = 0; i < originalCount; i++){
    int current_pan_index = i;
    double current_pan_angle = pan_angle_min + current_pan_index * pan_angle_increment;

    //ROS_INFO("Pan: %.3lf rad, %.3lf deg", current_pan_angle, current_pan_angle / 3.14 * 180);
    //Laser frame
    double theta = current_pan_angle;
    double l = _scan->ranges[i];

    if (theta > min_angle && theta < max_angle){
      double temp_y = l * cos(theta);
      double temp_x = l * sin(theta);
      double temp_z = 0.0;
      if (temp_x < 20.0 && temp_x > -20.0 && temp_y < 20.0 && temp_y > -20.0){
        temp_sum_x = temp_sum_x + temp_x;
        temp_sum_y = temp_sum_y + temp_y;
        temp_sum_xy = temp_sum_xy + temp_x * temp_y;
        temp_sum_xx = temp_sum_xx + temp_x * temp_x;
        actual_point_count += 1;
      }

    }
  }

  double temp_x_mean = temp_sum_x / actual_point_count;
  double temp_y_mean = temp_sum_y / actual_point_count;
  double denominator = temp_sum_xx - temp_sum_x * temp_x_mean;

      // ROS_INFO("sum x: %.3f", temp_x);
      // ROS_INFO("sum y: %.3f", temp_y);
      // ROS_INFO("sum xy: %.3f", temp_sum_xy);
      // ROS_INFO("sum xx: %.3f", temp_sum_xx);
  
  // ROS_INFO("original: %d", originalCount);
  // ROS_INFO("count: %d", actual_point_count);
  // ROS_INFO("denominator: %.3f", denominator);

  temp_slope = (temp_sum_xy - temp_sum_x * temp_y_mean) / denominator;
  temp_yaw_angle = atan(temp_slope);
}