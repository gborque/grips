/*******************************************************************************************
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************************/


#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <boost/bind.hpp>

void ftDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& ftdata, std::ofstream& ftDataFile)
{
	ftDataFile << ftdata->header.stamp.sec << "," << ftdata->header.stamp.nsec << ","<< ftdata->wrench.force.x << ","
		 << ftdata->wrench.force.y << "," << ftdata->wrench.force.z << "," << ftdata->wrench.torque.x
		 << "," << ftdata->wrench.torque.y << "," << ftdata->wrench.torque.z << std::endl;	
}
void imuDataCallback(const sensor_msgs::Imu::ConstPtr& imudata, std::ofstream& imuDataFile)
{
	imuDataFile << imudata->header.stamp.sec << ","<< imudata->header.stamp.nsec << "," << imudata->angular_velocity.x << ","
		 << imudata->angular_velocity.y << "," << imudata->angular_velocity.z << "," 		
		 << imudata->linear_acceleration.x << "," << imudata->linear_acceleration.y << "," 
		 << imudata->linear_acceleration.z << std::endl;
}
void imuOdomDataCallback(const nav_msgs::Odometry::ConstPtr& odomdata, std::ofstream& odomDataFile)
{
	odomDataFile << odomdata->header.stamp.sec << "," << odomdata->header.stamp.nsec << "," << odomdata->pose.pose.position.x << "," 
		 << odomdata->pose.pose.position.y << "," << odomdata->pose.pose.position.z << "," 
		 << odomdata->pose.pose.orientation.x << "," << odomdata->pose.pose.orientation.y << "," 
		 << odomdata->pose.pose.orientation.z << "," <<	odomdata->pose.pose.orientation.w << std::endl;
}
void extForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& ftdata, std::ofstream& extftDataFile)
{
	extftDataFile << ftdata->header.stamp.sec << "," << ftdata->header.stamp.nsec << ","<< ftdata->wrench.force.x << ","
		 << ftdata->wrench.force.y << "," << ftdata->wrench.force.z << "," << ftdata->wrench.torque.x
		 << "," << ftdata->wrench.torque.y << "," << ftdata->wrench.torque.z << std::endl;
}

int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "grips_robotiq_sensors_listener");
	
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;
	
	/**
	 *  Open csv files in order to record received data
	 */
	std::ofstream ftDataFile;
	ftDataFile.open("/home/guzman/ftdata.csv", std::ofstream::out);
	if(!ftDataFile)
	{
		std::cerr << "ftdata.csv could not be opened" << std::endl;
		exit(1);
	}
	else
	{
		ftDataFile << "File name: ftdata.csv" << std::endl << "Created in " << time(NULL) << std::endl;
		ftDataFile << "Time(sec),Time(nsec),ForceX(N),ForceY(N),ForceZ(N),TorqueX(Nm),TorqueY(Nm),TorqueZ(Nm)" << std::endl;
	}
	std::ofstream imuDataFile;
	imuDataFile.open("/home/guzman/imudata.csv", std::ofstream::out);
	if(!imuDataFile)
	{
		std::cerr << "imudata.csv could not be opened" << std::endl;
		exit(1);
	}
	else
	{
		imuDataFile << "File name: imudata.csv" << std::endl << "Created in " << time(NULL) << std::endl;
		imuDataFile << "Time(sec),Time(nsec),AngVelX(rad/s),AngVelY(rad/s),AngVelZ(rad/s),LinAccX(m/s2),"
					<< "LinAccY(m/s2),LinAccZ(m/s2)" << std::endl;
	}
	std::ofstream odomDataFile;
	odomDataFile.open("/home/guzman/odomdata.csv", std::ofstream::out);
	if(!odomDataFile)
	{
		std::cerr << "odomdata.csv could not be opened" << std::endl;
		exit(1);
	}
	else
	{
		odomDataFile << "File name: odomdata.csv" << std::endl << "Created in " << time(NULL) << std::endl;
		odomDataFile << "Time(sec),Time(nsec),pX(m),pY(m),pZ(m),qX,qY,qZ,qW" << std::endl;
	}
	std::ofstream extftDataFile;
	extftDataFile.open("/home/guzman/extftdata.csv", std::ofstream::out);
	if(!extftDataFile)
	{
		std::cerr << "extftdata.csv could not be opened" << std::endl;
		exit(1);
	}
	else
	{
		extftDataFile << "File name: extftdata.csv" << std::endl << "Created in " << time(NULL) << std::endl;
		extftDataFile << "Time(sec),Time(nsec),ForceX(N),ForceY(N),ForceZ(N),TorqueX(Nm),TorqueY(Nm),TorqueZ(Nm)" << std::endl;
	}
	
	/**
	* The subscribe() call is how you tell ROS that you want to receive messages
	* on a given topic.  This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing.  Messages are passed to a callback function, here
	* called chatterCallback.  subscribe() returns a Subscriber object that you
	* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	* object go out of scope, this callback will automatically be unsubscribed from
	* this topic.
	*
	* The second parameter to the subscribe() function is the size of the message
	* queue.  If messages are arriving faster than they are being processed, this
	* is the number of messages that will be buffered up before beginning to throw
	* away the oldest ones.
	*/	
	ros::Subscriber sub_ft = n.subscribe<geometry_msgs::WrenchStamped>("/grips/ft_raw", 100, 
		boost::bind(ftDataCallback, _1, boost::ref(ftDataFile)));
	ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("/grips/imu", 100, 
		boost::bind(imuDataCallback, _1, boost::ref(imuDataFile)));
	ros::Subscriber sub_odometry = n.subscribe<nav_msgs::Odometry>("/grips/imu_odom", 100, 
		boost::bind(imuOdomDataCallback, _1, boost::ref(odomDataFile)));
	ros::Subscriber sub_extft = n.subscribe<geometry_msgs::WrenchStamped>("/grips/extft_data", 100, 
		boost::bind(extForceCallback, _1, boost::ref(extftDataFile)));

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/	
	ros::spin();
	
	// Close fiel objects after pressing Ctrl-C
	ftDataFile.close();
	imuDataFile.close();
	odomDataFile.close();
	
	return 0;
}
