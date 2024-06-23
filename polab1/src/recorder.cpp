#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>

bool odometria_received = false;
bool gazebo_received = false;
float vel_keyvel = 0;
float x_odom, y_odom, x_gazebo, y_gazebo, diff_x, diff_y, diff;

std::vector<float> e_moment = {0};
std::vector<float> e_sum = {0};
std::vector<float> e_sides = {0};

void odomCallback(const nav_msgs::Odometry& msg)
{
	x_odom = msg.pose.pose.position.x;
	y_odom = msg.pose.pose.position.y;
	odometria_received = true;
}

void gazeboCallback(const gazebo_msgs::ModelStates& msg)
{
	x_gazebo = msg.pose[1].position.x;
	y_gazebo = msg.pose[1].position.y;
	gazebo_received = true;
}

void velCallback(const geometry_msgs::Twist& msg)
{
	vel_keyvel = sqrt(msg.linear.x * msg.linear.x + msg.linear.y * msg.linear.y);
}

void push_data()
{
	if(gazebo_received && odometria_received)
	{
		gazebo_received = false;
		odometria_received = false;
		diff_x = abs(x_odom - x_gazebo);
		diff_y = abs(y_odom - y_gazebo);
		diff = sqrt(diff_x * diff_x + diff_y * diff_y);
		e_moment.push_back(diff);
		e_sum.push_back(e_sum.back() + diff);
		if(abs(vel_keyvel) < 0.01)
		{
			e_sides.push_back(0);
		}
		else
		{
			e_sides.push_back(e_sides.back() + diff);
		}
	}
}

void save_to_file()
{
	std::ofstream F1("e_moment.txt");
	std::ofstream F2("e_sum.txt");
	std::ofstream F3("e_sides.txt");
	for(int i = 0; i < e_moment.size(); i++)
	{
		F1 << e_moment[i];
		F1 << " ";
		F2 << e_sum[i];
		F2 << " ";
		F3 << e_sides[i];
		F3 << " ";
	}
	F1.close();
	F2.close();
	F3.close();
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "recorder_node");
	ros::NodeHandle nh;
	ros::Subscriber odomSub = nh.subscribe("/mobile_base_controller/odom", 10, odomCallback);
	ros::Subscriber gazeboSub = nh.subscribe("/gazebo/model_states", 10, gazeboCallback);
	ros::Subscriber velSub = nh.subscribe("/key_vel", 10, velCallback);
	ros::Rate loopRate(10.0);
	loopRate.sleep();
	ros::Time start_time = ros::Time::now();
	while((ros::Time::now() - start_time).toSec() < 130)
	{
		ros::spinOnce();
		push_data();
		loopRate.sleep();
		save_to_file();
	}
	return 0;
}

