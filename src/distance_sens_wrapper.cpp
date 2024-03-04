#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>
#include <vector>

#include "std_msgs/String.h"

class DistanceWrapper
{
	public:
		DistanceWrapper();
		void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

	private:
		ros::NodeHandle node_;

		ros::Publisher left_scan_publisher_;
		ros::Publisher right_scan_publisher_;

		double angle_min;
		double angle_max;
		double angle_increment;
		double time_increment;
		double scan_time;
		double range_min;
		double range_max;

		double detection_threshold;

		std::string left_destination_frame;
		std::string right_destination_frame;
		std::string left_destination_topic;
		std::string right_destination_topic;
};

DistanceWrapper::DistanceWrapper()
{ 
	ros::NodeHandle nh("~");

	nh.param<std::string>("destination_frame", left_destination_frame, "distance_sensor_left"); 
	nh.param<std::string>("destination_frame", right_destination_frame, "distance_sensor_right");
	nh.param<std::string>("left_destination_topic", left_destination_topic, "/new_left_scan");
	nh.param<std::string>("right_destination_topic", right_destination_topic, "/new_right_scan");

	nh.param("angle_min", angle_min, -0.139626);
	nh.param("angle_max", angle_max, 0.139626);
	nh.param("angle_increment", angle_increment, 0.0116355);
	nh.param("scan_time", scan_time, 0.0333333);
	nh.param("range_min", range_min, 0.15);
	nh.param("range_max", range_max, 25.0);

	nh.param("detection_threshold", detection_threshold, 3.0);

	left_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(left_destination_topic.c_str(), 1, false);
	right_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(right_destination_topic.c_str(), 1, false);
}

void DistanceWrapper::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{

    //ROS_INFO("Scan Callback");
	
	//Create output message
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	output->header.stamp = ros::Time::now();
	output->header.frame_id = scan->header.frame_id;
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	// + 1 may not be needed. Debug
	uint32_t num_readings = std::ceil((output->angle_max - output->angle_min) / output->angle_increment) + 1;
	output->ranges.resize(num_readings);

	//Calc midpoint of scan input
	auto midpoint = ((scan->angle_max - scan->angle_min) / scan->angle_increment) / 2;
	
	//ROS_INFO("Midpoint: %d", midpoint); //Debugging print statement

	//Determine if distance sensor is detecting a crator or level ground
	double fill_val = output->range_max;
	if(scan->ranges[midpoint] < detection_threshold)	//TODO: FLIP THIS TO GREATER THAN FOR ACTUAL ROBOT
	{
		fill_val = scan->ranges[midpoint];
	}

	//Fill scan range with data
	for(uint i = 0; i < num_readings; i++)
	{
		output->ranges[i] = fill_val;
	}

	//Publish data
	if(output->header.frame_id == this->left_destination_frame)
		left_scan_publisher_.publish(output);
	else if(output->header.frame_id == this->right_destination_frame)
		right_scan_publisher_.publish(output);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "distance_sens_wrapper");

    ros::NodeHandle n;

    DistanceWrapper distWrapper;

    ros::Subscriber subL = n.subscribe("distance_left_scan", 1000, &DistanceWrapper::scan_callback, &distWrapper);
	ros::Subscriber subR = n.subscribe("distance_right_scan", 1000, &DistanceWrapper::scan_callback, &distWrapper);

  ros::spin();

}