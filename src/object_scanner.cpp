#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

using namespace std;

std::string node_name = "object_scanner";

std::string topic_name_scan = "/scan";
std::string topic_name_angle = "/head_tilt_joint/command";
sensor_msgs::LaserScan current_scan_data;

double tilt_angle_max = 0.785398;
double tilt_angle_min = -0.785398;
double tilt_angle_increment = 0.005;

double current_tilt_angle = 0.0;
int current_tilt_index = 0;

double pan_angle_min;
double pan_angle_max;
double pan_angle_increment;

double current_pan_angle = 0.0;
int current_pan_index = 0;

double pan_range_min = 0.0;
double pan_range_max = 60.0;



//Polar point cloud
const int ppc_width = 720; //depends on laser data
const int ppc_height = 320; //depends on tilting

double l = 0.1; //m    - cam_to_laser_distance

class PolarVector {
public:
	double alpha;
	double theta;
	double r;

	PolarVector(){};
	PolarVector(double _alpha, double _theta, double _r){
		alpha = _alpha;
		theta = _theta;
		r = _r;
	};
};

PolarVector ppc[ppc_height * ppc_width];
PolarVector ppc_cam_r[ppc_height * ppc_width];
PolarVector ppc_cam_l[ppc_height * ppc_width];

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& _scan);
void tilt_cb(const std_msgs::Float64::ConstPtr& _angle);

int loop_count = 0;

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	//Publisher

	//Subscriber
	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>(topic_name_scan, 10, laser_cb);
	ros::Subscriber angle_sub = nh.subscribe<std_msgs::Float64>(topic_name_angle, 10, tilt_cb);

	//Initialize polar point cloud
	for (int i = 0; i < ppc_height; i++){
        for (int j = 0; j < ppc_width; j++){
        }
    }

	ros::Rate loop_rate(30);
	while (ros::ok()){

		//Find dummy distance by angle range
		if (loop_count > 150){

			double pan_max = 0.174533;//0.349066;//20deg
			double pan_min = -0.174533;//10deg
			double tilt_max = 0.174533;//0.349066; //20deg
			double tilt_min = -0.174533; //10deg
			double distance = 0.0;
			int point_count = 0;
			
			for (int i = 0; i <= (ppc_width-1) * (ppc_height-1); i++){
				PolarVector temp_vector = ppc[i];
				double temp_alpha = temp_vector.alpha;
				double temp_theta = temp_vector.theta;
				double temp_r = temp_vector.r;

				if (temp_alpha >= tilt_min && temp_alpha <= tilt_max &&
					temp_theta >= pan_min && temp_theta <= pan_max && 
					temp_r > 0.01)
				{
					//distance = ((distance * point_count) + temp_r) / (point_count + 1);
					distance = distance + temp_r;
					point_count = point_count + 1;
				}
			}
			distance = distance / point_count;

			ROS_INFO("Points: %d Obstacle distance: %lf", point_count, distance);
		}

		loop_count++;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& _scan){
    //Get the number of original laser reading
    int originalCount = _scan->ranges.size();

	pan_angle_min = _scan->angle_min;
	pan_angle_max = _scan->angle_max;
	pan_angle_increment = _scan->angle_increment;

	pan_range_min = _scan->range_min;
	pan_range_max = _scan->range_max;

	//ROS_INFO("Tilt: %.3lf rad, %.3lf deg", current_tilt_angle, current_tilt_angle / 3.14 * 180);

    for (int i = 0; i < originalCount; i++){
        current_pan_index = i;
        current_pan_angle = pan_angle_min + current_pan_index * pan_angle_increment;
        //ROS_INFO("Pan: %.3lf rad, %.3lf deg", current_pan_angle, current_pan_angle / 3.14 * 180);
        //Laser frame
        double alpha = current_tilt_angle;
        double theta = current_pan_angle;
        double d = _scan->ranges[i];
        //ROS_INFO("Pan: %.3lf deg, Tilt %.3lf deg, R: %.3lf", current_pan_angle / 3.14 * 180, current_tilt_angle / 3.14 * 180, d);
        
        PolarVector temp_vector = PolarVector(alpha, theta, d);
    	ppc[current_tilt_index * ppc_width + current_pan_index] = temp_vector;

    	//Right camera frame
    	double tan_theta_c = (d * sin(theta) - l) / (d * cos(theta));
    	double theta_c = atan(tan_theta_c);
    	double d_c = d * cos(theta) / cos(theta_c);

    	temp_vector = PolarVector(alpha, theta_c, d_c);
    	ppc_cam_r[current_tilt_index * ppc_width + current_pan_index] = temp_vector;
    	
    	//Left camera frame
    	tan_theta_c = (d * sin(theta) + l) / (d * cos(theta));
    	theta_c = atan(tan_theta_c);
    	d_c = d * cos(theta) / cos(theta_c);

    	temp_vector = PolarVector(alpha, theta_c, d_c);
    	ppc_cam_l[current_tilt_index * ppc_width + current_pan_index] = temp_vector;
    	
    	if (current_pan_index >= (ppc_width - 1)) break;
    }
}

void tilt_cb(const std_msgs::Float64::ConstPtr& _angle){
	current_tilt_angle = _angle->data;
	current_tilt_index = (current_tilt_angle - tilt_angle_min) / tilt_angle_increment;
	
	if (current_tilt_index >= ppc_height - 1) current_tilt_index = ppc_height - 1;
	if (current_tilt_index < 0) current_tilt_index = 0;
}

