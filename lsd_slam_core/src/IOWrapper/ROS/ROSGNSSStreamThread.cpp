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

#include "ROSGNSSStreamThread.h"
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include "cv_bridge/cv_bridge.h"
#include "util/settings.h"

#include <iostream>
#include <fstream>


namespace lsd_slam
{



ROSGNSSStreamThread::ROSGNSSStreamThread()
{
	// subscribe
	gnss_channel = nh_.resolveName("/left_gnss/fix");
	gnss_sub = nh_.subscribe(gnss_channel,1, &ROSGNSSStreamThread::gnssCb, this);

	// imagebuffer
	gnssBuffer = new NotifyBuffer<TimestampedGNSS>(8);
	lastSEQ = 0;

}

ROSGNSSStreamThread::~ROSGNSSStreamThread()
{
	delete gnssBuffer;
}

void ROSGNSSStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void ROSGNSSStreamThread::operator()()
{
	ros::AsyncSpinner spinner(8); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
	//ros::spin();

	exit(0);
}


void ROSGNSSStreamThread::gnssCb(const sensor_msgs::NavSatFixConstPtr &gnss)
{
	if(gnss->header.seq < (unsigned int)lastSEQ)
	{
		printf("Backward-Jump in SEQ detected, but ignoring for now (gnss).\n");
		lastSEQ = 0;
		return;
	}
	lastSEQ = gnss->header.seq;

	TimestampedGNSS bufferItem;
	if(gnss->header.stamp.toSec() != 0)
		bufferItem.timestamp =  Timestamp(gnss->header.stamp.toSec());
	else
		bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());
    
    bufferItem.positioning[0] = gnss->longitude;
    bufferItem.positioning[1] = gnss->latitude;
    bufferItem.positioning[2] = gnss->altitude;
    bufferItem.variance[0] = gnss->position_covariance[0];
    bufferItem.variance[1] = gnss->position_covariance[4];
    bufferItem.variance[2] = gnss->position_covariance[8];
    bufferItem.status = gnss->status.status;

	gnssBuffer->pushBack(bufferItem);
}

}
