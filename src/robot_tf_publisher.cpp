//Listen to PoseWithCovarianceStamped
//Publish as tf

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

std::string node_name = "robot_tf_publisher";
std::string robot_frame = "/base_frame";
std::string map_frame = "/map";

std::string topic_name_pose = "/pose_in";

geometry_msgs::PoseWithCovarianceStamped currentPose;
double t_x, t_y, t_z, r_x, r_y, r_z, r_w;
double r_rol, r_pit, r_yaw;

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _pose);

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);

	ros::NodeHandle nh;

	ros::NodeHandle nh_param("~");
	nh_param.param<std::string>("pose_in", topic_name_pose, topic_name_pose);
	nh_param.param<std::string>("robot_frame", robot_frame, robot_frame);
	nh_param.param<std::string>("map_frame", map_frame, map_frame);

	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
		(topic_name_pose, 10, pose_cb);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	
	ros::Rate loop_rate(30);

	while (ros::ok()){
		q.setRPY(r_rol, r_pit, r_yaw);
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, robot_frame));

		loop_rate.sleep();
		ros::spinOnce();
	}
}

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _pose){
	currentPose = *_pose;

	t_x = currentPose.pose.pose.position.x;
	t_y = currentPose.pose.pose.position.y;
	t_z = currentPose.pose.pose.position.z;
	
	r_x = currentPose.pose.pose.orientation.x;
	r_y = currentPose.pose.pose.orientation.y;
	r_z = currentPose.pose.pose.orientation.z;
	r_w = currentPose.pose.pose.orientation.w;

	tf::Quaternion q(r_x, r_y, r_z, r_w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	r_rol = roll;
	r_pit = pitch;
	r_yaw = yaw;
}
