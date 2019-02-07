/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include "IOWrapper/InputGNSSStream.h"

#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/PoseStamped.h>


#include "util/Undistorter.h"


namespace lsd_slam
{



/**
 * Image stream provider using ROS messages.
 */
class ROSGNSSStreamThread : public InputGNSSStream
{
public:
	ROSGNSSStreamThread();
	~ROSGNSSStreamThread();

	/**
	 * Starts the thread.
	 */
	void run();

	/**
	 * Thread main function.
	 */
	void operator()();

	// get called on ros-message callbacks
	void gnssCb(const sensor_msgs::NavSatFixConstPtr &gnss);

private:
	ros::NodeHandle nh_;

	std::string gnss_channel;
	ros::Subscriber gnss_sub;

	int lastSEQ;
};

}
