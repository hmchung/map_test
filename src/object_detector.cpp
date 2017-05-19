//subscribe to point cloud
//subscribe to cam object recognition
//send to object manager

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

std::string node_name = "object_detector";

#define image_width 640
#define image_height 480
#define image_diagonal 800
#define cam_view_angle 1.3962634

double angle_per_pixel = 0.001745329;
double constraint_distance = 10.0; //theoretical distance where the object can be seen

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
void obj_1_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi);
void obj_2_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi);
void obj_3_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi);
void obj_4_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi);
void obj_5_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi);
void obj_6_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi);

geometry_msgs::Pose getPoseFromROI(sensor_msgs::RegionOfInterest _roi, 
	double true_width, double *estimate_certainty);
std_msgs::ColorRGBA getColorRGBAFromColorCode(int color_code);
int getColorCodeFromColorStr(std::string color_str);
int getObjCodeFromObjStr(std::string obj_str);
double getTrueWidthFromObjType(int obj_type_code);
double getDistanceFromROI(int ROI_width, double true_width);

PointCloud::Ptr temp_cloud;
PointCloud::Ptr current_cloud;
PointCloud::Ptr current_filtered_cloud;

std::string cam_frame_name = "/fixed_cam";
std::string cam_namespace = "fixed_cam";
tf::TransformListener *tf_listener;
ros::Publisher pub_3D;

std::string object_1 = "";
std::string object_2 = "";
std::string object_3 = "";
std::string object_4 = "";
std::string object_5 = "";
std::string object_6 = "";

std::string color_1 = "";
std::string color_2 = "";
std::string color_3 = "";
std::string color_4 = "";
std::string color_5 = "";
std::string color_6 = "";


int color_code_1 = 0;
int color_code_2 = 1;
int color_code_3 = 2;
int color_code_4 = 0;
int color_code_5 = 1;
int color_code_6 = 2;

int obj_code_1 = 0;
int obj_code_2 = 1;
int obj_code_3 = 2;
int obj_code_4 = 0;
int obj_code_5 = 1;
int obj_code_6 = 2;

double obj_true_width_1 = 0.04;
double obj_true_width_2 = 0.04;
double obj_true_width_3 = 0.04;
double obj_true_width_4 = 0.4572;
double obj_true_width_5 = 0.4572;
double obj_true_width_6 = 0.4572;

std::string topic_detector_1 = "/fixed_cam/circle/red/roi";
std::string topic_detector_2 = "/fixed_cam/triangle/green/roi";
std::string topic_detector_3 = "/fixed_cam/cruciform/blue/roi";
std::string topic_detector_4 = "/fixed_cam/circle/blue/roi";
std::string topic_detector_5 = "/fixed_cam/triangle/red/roi";
std::string topic_detector_6 = "/fixed_cam/cruciform/green/roi";

std::string topic_object_publish = "/mixed_object_marker";
std::string topic_cloud_publish = "/fixed_cam/point_cloud";

ros::Publisher obj_detector_pub;

sensor_msgs::RegionOfInterest temp_roi_1, temp_roi_2, temp_roi_3, temp_roi_4, temp_roi_5, temp_roi_6;
bool has_roi_1 = false, has_roi_2 = false, has_roi_3 = false, has_roi_4 = false, has_roi_5 = false, has_roi_6 = false;
bool has_rois[6];

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	nh_param.param<std::string>("cam_frame_name", cam_frame_name, cam_frame_name);
	nh_param.param<std::string>("cam_namespace", cam_namespace, cam_namespace);

	nh_param.param<std::string>("object_1", object_1, object_1);
	nh_param.param<std::string>("object_2", object_2, object_2);
	nh_param.param<std::string>("object_3", object_3, object_3);
	nh_param.param<std::string>("object_4", object_4, object_4);
	nh_param.param<std::string>("object_5", object_5, object_5);
	nh_param.param<std::string>("object_6", object_6, object_6);

	nh_param.param<std::string>("color_1", color_1, color_1);
	nh_param.param<std::string>("color_2", color_2, color_2);
	nh_param.param<std::string>("color_3", color_3, color_3);
	nh_param.param<std::string>("color_4", color_4, color_4);
	nh_param.param<std::string>("color_5", color_5, color_5);
	nh_param.param<std::string>("color_6", color_6, color_6);

	color_code_1 = getColorCodeFromColorStr(color_1);
	color_code_2 = getColorCodeFromColorStr(color_2);
	color_code_3 = getColorCodeFromColorStr(color_3);
	color_code_4 = getColorCodeFromColorStr(color_4);
	color_code_5 = getColorCodeFromColorStr(color_5);
	color_code_6 = getColorCodeFromColorStr(color_6);

	obj_code_1 = getObjCodeFromObjStr(object_1);
	obj_code_2 = getObjCodeFromObjStr(object_2);
	obj_code_3 = getObjCodeFromObjStr(object_3);
	obj_code_4 = getObjCodeFromObjStr(object_4);
	obj_code_5 = getObjCodeFromObjStr(object_5);
	obj_code_6 = getObjCodeFromObjStr(object_6);		

	topic_detector_1 = "/" + cam_namespace + "/" + object_1 + "/" + color_1 + "/roi";
	topic_detector_2 = "/" + cam_namespace + "/" + object_2 + "/" + color_2 + "/roi";
	topic_detector_3 = "/" + cam_namespace + "/" + object_3 + "/" + color_3 + "/roi";
	topic_detector_4 = "/" + cam_namespace + "/" + object_4 + "/" + color_4 + "/roi";
	topic_detector_5 = "/" + cam_namespace + "/" + object_5 + "/" + color_5 + "/roi";
	topic_detector_6 = "/" + cam_namespace + "/" + object_6 + "/" + color_6 + "/roi";

	//topic_object_publish = "/" + cam_namespace + "/mixed_object_marker";
	topic_object_publish = "/mixed_object_marker";
	topic_cloud_publish = "/" + cam_namespace + "/point_cloud";

	tf_listener = new tf::TransformListener();

	//Subscriber
	ros::Subscriber sub_2d_cloud = nh.subscribe("/point_cloud_3D_last", 1, cloud_cb);


	ros::Subscriber sub_obj_detector_1;
	ros::Subscriber sub_obj_detector_2;
	ros::Subscriber sub_obj_detector_3;
	ros::Subscriber sub_obj_detector_4;
	ros::Subscriber sub_obj_detector_5;
	ros::Subscriber sub_obj_detector_6;

	if (object_1 != "") {
		sub_obj_detector_1 = nh.subscribe<sensor_msgs::RegionOfInterest>
		(topic_detector_1, 10, obj_1_cb);
		ROS_WARN("[Object detector] Detector 1. Topic: %s", topic_detector_1.c_str());
	}
	if (object_2 != "") {
		sub_obj_detector_2 = nh.subscribe<sensor_msgs::RegionOfInterest>
			(topic_detector_2, 10, obj_2_cb);
		ROS_WARN("[Object detector] Detector 2. Topic: %s", topic_detector_2.c_str());
	}
	if (object_3 != "") {
		sub_obj_detector_3 = nh.subscribe<sensor_msgs::RegionOfInterest>
			(topic_detector_3, 10, obj_3_cb);
		ROS_WARN("[Object detector] Detector 3. Topic: %s", topic_detector_3.c_str());
	}	
	if (object_4 != "") {
		sub_obj_detector_4 = nh.subscribe<sensor_msgs::RegionOfInterest>
			(topic_detector_4, 10, obj_4_cb);
		ROS_WARN("[Object detector] Detector 4. Topic: %s", topic_detector_4.c_str());
	}
	if (object_5 != "") {
		sub_obj_detector_5 = nh.subscribe<sensor_msgs::RegionOfInterest>
			(topic_detector_5, 10, obj_5_cb);
		ROS_WARN("[Object detector] Detector 5. Topic: %s", topic_detector_5.c_str());
	}
	if (object_6 != "") {
		sub_obj_detector_6 = nh.subscribe<sensor_msgs::RegionOfInterest>
			(topic_detector_6, 10, obj_6_cb);
		ROS_WARN("[Object detector] Detector 6. Topic: %s", topic_detector_6.c_str());
	}
	//Publisher
	pub_3D = nh.advertise<PointCloud>(topic_cloud_publish, 1);	
	obj_detector_pub = nh.advertise<visualization_msgs::Marker>(topic_object_publish, 1);

	ros::Rate loop_rate(10);
	while (ros::ok()){
		for (int i = 1; i <= 6; i++){
			sensor_msgs::RegionOfInterest my_roi;
			int my_obj_code = 1;
			int my_color_code = 1;
			bool temp_has_roi = has_rois[i - 1];

			switch (i){
				case 1:
					my_roi = temp_roi_1;				
					my_obj_code = obj_code_1;
					my_color_code = color_code_1;
					//temp_has_roi = has_roi_1;
					break;
				case 2: 
					my_roi = temp_roi_2;
					my_obj_code = obj_code_2;
					my_color_code = color_code_2;
					//temp_has_roi = has_roi_2;
					break;
				case 3: 
					my_roi = temp_roi_3;
					my_obj_code = obj_code_3;
					my_color_code = color_code_3;
					//temp_has_roi = has_roi_3;
					break;
				case 4: 
					my_roi = temp_roi_4;
					my_obj_code = obj_code_4;
					my_color_code = color_code_4;
					//temp_has_roi = has_roi_4;
					break;
				case 5: 
					my_roi = temp_roi_5;
					my_obj_code = obj_code_5;
					my_color_code = color_code_5;
					//temp_has_roi = has_roi_5;
					break;
				case 6: 
					my_roi = temp_roi_6;
					my_obj_code = obj_code_6;
					my_color_code = color_code_6;
					//temp_has_roi = has_roi_6;
					break;
			}
			if (temp_has_roi){
				// ROS_INFO("Case: %d", i);
				double my_true_width = getTrueWidthFromObjType(my_obj_code);
				double my_certainty;

				geometry_msgs::Pose my_pose = getPoseFromROI(my_roi, my_true_width, &my_certainty);
				double temp_x = my_pose.position.x;
				double temp_y = my_pose.position.y;
				double temp_z = my_pose.position.z;
				
				visualization_msgs::Marker my_marker_msg;
				my_marker_msg.header.stamp = ros::Time::now();
				my_marker_msg.header.frame_id = "/map";
				my_marker_msg.color = getColorRGBAFromColorCode(my_color_code);
	
				my_marker_msg.type = my_obj_code;
				my_marker_msg.id = my_color_code;
				my_marker_msg.pose = my_pose;
				my_marker_msg.scale.x = 0.5;
				my_marker_msg.scale.y = 0.5;
				my_marker_msg.scale.z = 0.5;

				if (cam_frame_name == "/pan_tilt_cam" || cam_namespace == "pan_tilt_cam") my_certainty = 0.5;
				my_marker_msg.color.a = my_certainty; 
	
				obj_detector_pub.publish(my_marker_msg);
				has_rois[i - 1] =  false;
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{	
	//get the cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
	
	//Noise filtering
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (temp_cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*temp_cloud);

	current_cloud = temp_cloud;

    //Transform the cloud to camera frame
	PointCloud::Ptr temp_cloud_transformed (new PointCloud);;
	pcl::PointXYZ temp;
   
	for (int k = 0; k < current_cloud->points.size(); k++){
		temp = current_cloud->points.at(k);
		double temp_x = temp.x;
		double temp_y = temp.y;
		double temp_z = temp.z;		
	    
	    geometry_msgs::PointStamped temp_point_out;
	    geometry_msgs::PointStamped temp_point_in;
	    temp_point_in.header.frame_id = "/map";
	    temp_point_in.point.x = temp_x;
	    temp_point_in.point.y = temp_y;
	    temp_point_in.point.z = temp_z;

	    try{
			tf_listener->transformPoint(cam_frame_name, temp_point_in, temp_point_out);
	        temp_x = temp_point_out.point.x;
			temp_y = temp_point_out.point.y;
			temp_z = temp_point_out.point.z;

			double alpha = abs(atan(temp_y / temp_x));
			if (alpha < 0.872665){
				temp_cloud_transformed->points.push_back (pcl::PointXYZ(temp_x, temp_y, temp_z));
	    	}
	    }
	    catch (tf::TransformException ex){
	        ROS_ERROR("%s",ex.what());
	        ros::Duration(1.0).sleep();
	    }
	}
	current_filtered_cloud = temp_cloud_transformed;
	temp_cloud_transformed->header.frame_id = cam_frame_name;
	//pub_3D.publish (temp_cloud_transformed);
}

void obj_1_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi){
	temp_roi_1 = *_roi;
	has_rois[0] = true;
	// has_roi_1 = true;
}
void obj_2_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi){
	temp_roi_2 = *_roi;
	has_rois[1] = true;
	// has_roi_2 = true;	
}
void obj_3_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi){
	temp_roi_3 = *_roi;
	has_rois[2] = true;
	// has_roi_3 = true;	
}
void obj_4_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi){
	temp_roi_4 = *_roi;
	has_rois[3] = true;
	// has_roi_4 = true;	
}
void obj_5_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi){
	temp_roi_5 = *_roi;
	has_rois[4] = true;
	// has_roi_5 = true;	
}
void obj_6_cb (const sensor_msgs::RegionOfInterest::ConstPtr& _roi){
	temp_roi_6 = *_roi;
	has_rois[5] = true;
	// has_roi_6 = true;	
}

geometry_msgs::Pose getPoseFromROI(sensor_msgs::RegionOfInterest _roi, 
	double true_width, double *estimate_certainty){
	//Get pan and pitch angle from ROI
	//pan angle: 	alpha
	//tilt angle: 	beta
	int x_offset = _roi.x_offset;
	int y_offset = _roi.y_offset;
	int height = _roi.height;
	int width = _roi.width;

	double scale_factor = 0.8;

	double beta_hi_lim = (image_width/2 - x_offset) * angle_per_pixel * scale_factor;
	double beta_lo_lim = (image_width/2 - x_offset - width) * angle_per_pixel * scale_factor;
	double alpha_hi_lim = (image_width/2 - y_offset) * angle_per_pixel * scale_factor;
	double alpha_lo_lim = (image_width/2 - y_offset - height) * angle_per_pixel * scale_factor; 

	// pcl::PointXYZ temp;

	double pose_x = 0.0;
	double pose_y = 0.0;
	double pose_z = 0.0;

	// int point_count = 0;

	// //Try laser estimate - Filter valid points
	// for (int k = 0; k < current_filtered_cloud->points.size(); k++){
	// 	temp = current_filtered_cloud->points.at(k);
	// 	double temp_x = temp.x;
	// 	double temp_y = temp.y;
	// 	double temp_z = temp.z;

	// 	//double temp_d = sqrt(temp_x * temp_x + temp_z * temp_z);
	// 	double temp_alpha = atan(temp_z / temp_x);
	// 	double temp_beta = atan(temp_y / temp_x);

	// 	// ROS_INFO("detect: alpha=%.3f, beta=%3f", temp_beta, temp_alpha);

	// 	if (temp_alpha > alpha_lo_lim && temp_alpha < alpha_hi_lim){
	// 		if (temp_beta > beta_lo_lim && temp_beta < beta_hi_lim){
	// 			if (temp_x > 0.5){
	// 				pose_x = (pose_x * point_count + temp_x) / (point_count + 1);
	// 				pose_y = (pose_y * point_count + temp_y) / (point_count + 1);
	// 				pose_z = (pose_z * point_count + temp_z) / (point_count + 1);
	// 				point_count = point_count + 1;
	// 			}
	// 		}
	// 	}
	// }


	// if (point_count > 0){ //can estimate from laser
	// 	*estimate_certainty = 1.0;		
	// } else { 			//estimate from vision
		*estimate_certainty = 1.0;

		double pureVisionEstimate = getDistanceFromROI(width, true_width);
		double alpha_center = (alpha_hi_lim + alpha_lo_lim) / 2.0;
		double beta_center = (beta_hi_lim + beta_lo_lim) / 2.0;

		pose_y 			= - pureVisionEstimate * sin(beta_center);
		double temp_d 	= pureVisionEstimate * cos(beta_center);
		pose_z = - temp_d * sin(alpha_center);
		pose_x = temp_d * cos(alpha_center); 
	// }

	//Transform back to map frame
    geometry_msgs::PointStamped temp_point_out;
    geometry_msgs::PointStamped temp_point_in;
    temp_point_in.header.frame_id = cam_frame_name;
    temp_point_in.point.x = pose_x;
    temp_point_in.point.y = pose_y;
    temp_point_in.point.z = pose_z;

    tf::StampedTransform transform;

    try{
    	tf_listener->transformPoint("/map", temp_point_in, temp_point_out);
        tf_listener->lookupTransform("/map", "/base_link", ros::Time(0), transform);
        pose_x = temp_point_out.point.x;
		pose_y = temp_point_out.point.y;
		pose_z = temp_point_out.point.z;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

	geometry_msgs::Pose return_pose;
	return_pose.position.x = pose_x;
	return_pose.position.y = pose_y;
	return_pose.position.z = pose_z;

	double x_b = transform.getOrigin().x();
	double y_b = transform.getOrigin().y();
	double x_m = pose_x;
	double y_m = pose_y;
	double rotation_angle = atan2((y_b - y_m), (x_b - x_m));

	tf::Quaternion q;
	q.setRPY(0.0, 0.0, rotation_angle);

	return_pose.orientation.x = q.getX();
	return_pose.orientation.y = q.getY();
	return_pose.orientation.z = q.getZ();
	return_pose.orientation.w = q.getW();

	return return_pose;
}

int getColorCodeFromColorStr(std::string color_str){
	if(color_str.compare("red") == 0) 	return 0;
	if(color_str.compare("green") == 0) return 1;
	if(color_str.compare("blue") == 0) 	return 2;	
	if(color_str.compare("black") == 0) return 3;
	if(color_str.compare("white") == 0) return 4;
	if(color_str.compare("yellow") == 0) return 5;
	if(color_str.compare("orange") == 0) return 6;
	return 0;
}

int getObjCodeFromObjStr(std::string color_str){
	if(color_str.compare("triangle") == 0) 	return 0;
	if(color_str.compare("cruciform") == 0) return 1;
	if(color_str.compare("circle") == 0) 	return 2;	
	if(color_str.compare("totem") == 0) 	return 3;
	if(color_str.compare("rectangle") == 0) return 4;
	return 0;
}

std_msgs::ColorRGBA getColorRGBAFromColorCode(int color_code){
	std_msgs::ColorRGBA c;
	c.r = 0; c.g = 0; c.b = 0;
	switch (color_code) {
		case 0: c.r = 1; break; 
		case 1: c.g = 1; break;
		case 2: c.b = 1; break;
		case 3: c.r = 0; break;
		case 4: c.r = 1; c.g = 1; c.b = 1; break;
		case 5: c.r = 1; c.g = 1; c.b = 0; break;
		case 6: c.r = 1; c.g = 0.647; c.b = 0; break;
		default: c.r = 0; c.g = 1; c.b = 1; break;
	}
	return c;
}

double getTrueWidthFromObjType(int obj_type_code){
	if (obj_type_code == 1 ||
		obj_type_code == 2 ||
		obj_type_code == 3) return 0.60;
	if (obj_type_code == 4 ||
		obj_type_code == 5) return 0.4572;
	return 0.4572;
}

double getDistanceFromROI(int ROI_width, double true_width){
	double temp_dist = true_width * 140.1690566 * pow(ROI_width, -1.029);
	if (temp_dist >= constraint_distance) return constraint_distance;
	else return temp_dist;
}