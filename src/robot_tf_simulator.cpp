#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "my_robot_sim");
  ros::NodeHandle nh;

  ros::Rate loop_rate(40);

  while (ros::ok()){
    double temp_x = 0.0 + 0.1 * (double)(rand() % 100) / 100.0;
    double temp_y = 0.0 + 0.2 * (double)(rand() % 100) / 100.0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(temp_x, temp_y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "scanmatcher_frame"));
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}