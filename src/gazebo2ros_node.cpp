#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
//#include <gazebo/math/gzmath.hh>
#include <ignition/math/Vector3.hh>

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <robotino_msgs/ResetOdometry.h>

double g_x, g_y, g_z;
/* TODO: Find better solution */
bool g_init = false;
ros::Publisher g_pubOdom;
ros::Publisher g_pubLightSignal;
ros::Publisher g_pubMachines;
gazebo::transport::PublisherPtr g_gripper_pub;


#include <boost/bind.hpp>
void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
void gpsCallback(ConstPosePtr &msg);
bool resetOdomSrvCallback(robotino_msgs::ResetOdometryRequest &request, robotino_msgs::ResetOdometryResponse &response);

#define ROBOTINO_NAME "robotino1"

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
	gazebo::common::Time waitTimeout(0.1);

	printf("Gazebo to ros Started\n");

	// Load gazebo
	gazebo::client::setup(argc, argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	ros::init(argc, argv, "publisher");
	ros::NodeHandle nh;
	std::string robotName;
	nh.param<std::string>("simuRobotNamespace", robotName, ROBOTINO_NAME);

	// Publish to a Gazebo topic
	gazebo::transport::PublisherPtr move_pub =
	node->Advertise<gazebo::msgs::Vector3d>("/gazebo/c3_world/" +robotName+ "/RobotinoSim/MotorMove/");

	ros::ServiceServer resetOdom_srv = nh.advertiseService("reset_odometry", resetOdomSrvCallback);


	// Wait for a subscriber to connect (blocking but will still exit on SIGINT)
	while (ros::ok() && !move_pub->WaitForConnection(waitTimeout)){ROS_DEBUG("wait for move_pub connection");}

	// Subscriber
	ros::Subscriber subCmdVel = nh.subscribe("hardware/cmd_vel", 1, &cmdVelCallback);
	g_pubOdom = nh.advertise<nav_msgs::Odometry>("hardware/odom", 1000);
	gazebo::transport::SubscriberPtr subGps = node->Subscribe("/gazebo/c3_world/" +robotName+ "/gazsim/gps/", &gpsCallback);
	// Publisher

	g_init = true;

	// Publisher loop...replace with your own code.
	g_x=0; g_y=0; g_z=0;
	ignition::math::Vector3<double> vect(g_x,g_y,g_z);
	while (ros::ok())
	{
		// Throttle Publication
		gazebo::common::Time::MSleep(100);

		// Generate a pose
		vect.Set(g_x, g_y, g_z);

		// Convert to a pose message
		gazebo::msgs::Vector3d msg;
		ignition::math::Vector3d tmp(g_x,g_y,g_z);
		msg = gazebo::msgs::Convert(tmp);

		move_pub->Publish(msg);
		ros::spinOnce();
	}

	// Make sure to shut everything down.
	gazebo::shutdown();
}

void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
	g_x=msg->linear.x;
	g_y=msg->linear.y;
	g_z=msg->angular.z;
}

void gpsCallback(ConstPosePtr &msg)
{
	ros::NodeHandle nh;
	std::string tf_prefix;
	nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
	if (tf_prefix.size() != 0)
		tf_prefix += "/";

	nav_msgs::Odometry odom_msg;
	odom_msg.child_frame_id=tf_prefix+"base_link";
	odom_msg.header.frame_id=tf_prefix+"odom";
	odom_msg.header.stamp=ros::Time::now();

	odom_msg.pose.pose.position.x=msg->position().x();
	odom_msg.pose.pose.position.y=msg->position().y();
	odom_msg.pose.pose.position.z=0;

	odom_msg.pose.pose.orientation.x=msg->orientation().x();
	odom_msg.pose.pose.orientation.y=msg->orientation().y();
	odom_msg.pose.pose.orientation.z=msg->orientation().z();
	odom_msg.pose.pose.orientation.w=msg->orientation().w();

	odom_msg.twist.twist.linear.x=g_x;
	odom_msg.twist.twist.linear.y=g_y;
	odom_msg.twist.twist.linear.z=0;
	odom_msg.twist.twist.angular.x=0;
	odom_msg.twist.twist.angular.y=0;
	odom_msg.twist.twist.angular.z=g_z;

	if (g_init)
		g_pubOdom.publish(odom_msg);
}



bool resetOdomSrvCallback(robotino_msgs::ResetOdometryRequest &request, robotino_msgs::ResetOdometryResponse &/*response*/)
{
	ROS_INFO("Got reset_odom x:%f, y:%f, phi:%f", request.x, request.y, request.phi);

	return true;
}
