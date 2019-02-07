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

#include "ROSImageStreamThread.h"
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cv_bridge/cv_bridge.h"
#include "util/settings.h"

#include <iostream>
#include <fstream>


namespace lsd_slam
{


using namespace cv;

ROSImageStreamThread::ROSImageStreamThread()
{
	// subscribe
	//vid_channel = nh_.resolveName("image");
	//vid_sub = nh_.subscribe(vid_channel,1, &ROSImageStreamThread::vidCb, this);



	
	img_sub.subscribe(nh_, "/camera/color/image_raw", 1);
	depth_sub.subscribe(nh_, "/camera/aligned_depth_to_color/image_raw", 1);

    //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> lsync(lgnss_sub, lgnss_velocity_sub, 10);
    //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> rsync(rgnss_sub, rgnss_velocity_sub, 10);

    //lsync = new message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped>(lgnss_sub, lgnss_velocity_sub, 10);
    //rsync = new message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped>(rgnss_sub, rgnss_velocity_sub, 10);
    //lsync->registerCallback(boost::bind(&ekf_publisher::lgnss_callback, this, _1, _2));
    //rsync->registerCallback(boost::bind(&ekf_publisher::rgnss_callback, this, _1, _2));

    //In your constructor initialize and register:
    vid_sync.reset(new Sync(VidSyncPolicy(10), img_sub,depth_sub));
    vid_sync->registerCallback(boost::bind(&ROSImageStreamThread::vidCb, this, _1, _2));


	// wait for cam calib
	width_ = height_ = 0;
	 

	// imagebuffer
	imageBuffer = new NotifyBuffer<TimestampedMat>(8);
	undistorter = 0;
	lastSEQ = 0;

	//haveCalib = false;
	haveCalib = false;
}

ROSImageStreamThread::~ROSImageStreamThread()
{
	delete imageBuffer;
}

void ROSImageStreamThread::setCalibration(std::string file)
{
	if(file == "")
	//if(false)
	{
		ros::Subscriber info_sub = nh_.subscribe(nh_.resolveName("camera_info"),1, &ROSImageStreamThread::infoCb, this);

		printf("WAITING for ROS camera calibration!\n");
		while(width_ == 0)
		{
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
		}
		printf("RECEIVED ROS camera calibration!\n");

		info_sub.shutdown();
	}
	else
	{
		undistorter = Undistorter::getUndistorterForFile(file.c_str());

		if(undistorter==0)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			exit(0);
		}

		fx_ = undistorter->getK().at<double>(0, 0);
		fy_ = undistorter->getK().at<double>(1, 1);
		cx_ = undistorter->getK().at<double>(2, 0);
		cy_ = undistorter->getK().at<double>(2, 1) - 4;

		width_ = undistorter->getOutputWidth();
		height_ = undistorter->getOutputHeight() -8;
	}

	haveCalib = true;
}

void ROSImageStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void ROSImageStreamThread::operator()()
{
	ros::spin();

	exit(0);
}

void ROSImageStreamThread::vidCb(const sensor_msgs::ImageConstPtr img)
{
	if(!haveCalib) return;

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

	if(img->header.seq < (unsigned int)lastSEQ)
	{
		printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
		lastSEQ = 0;
		return;
	}
	lastSEQ = img->header.seq;

	TimestampedMat bufferItem;
	if(img->header.stamp.toSec() != 0)
		bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
	else
		bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());

	if(undistorter != 0)
	{
		assert(undistorter->isValid());
		undistorter->undistort(cv_ptr->image,bufferItem.img);
	}
	else
	{
		bufferItem.img = cv_ptr->image;
	}

	imageBuffer->pushBack(bufferItem);
}


void ROSImageStreamThread::vidCb(const sensor_msgs::ImageConstPtr img, const sensor_msgs::ImageConstPtr depth)
{
	if(!haveCalib) return;

	cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);

	if(img->header.seq < (unsigned int)lastSEQ)
	{
		printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
		lastSEQ = 0;
		return;
	}
	lastSEQ = img->header.seq;

	TimestampedMat bufferItem;
	
	//* clop image *//
	cv::Mat crop_img(cv_img_ptr->image, cv::Rect(0, 4, 640, 352));
	cv::Mat crop_depth(cv_depth_ptr->image, cv::Rect(0, 4, 640, 352));
	
	cv::Matx<float,352,640> depth_data;
	
	(crop_depth).convertTo(depth_data, CV_32F,0.001);
	if(img->header.stamp.toSec() != 0)
		bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
	else
		bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());

	if(undistorter != 0)
	{
		assert(undistorter->isValid());
		undistorter->undistort(crop_img,bufferItem.img);
		bufferItem.depth = depth_data;
	}
	else
	{
		bufferItem.img = cv_img_ptr->image;
		bufferItem.depth = depth_data
		;
	}

	imageBuffer->pushBack(bufferItem);
}

void ROSImageStreamThread::infoCb(const sensor_msgs::CameraInfoConstPtr info)
{
	if(!haveCalib)
	{
		fx_ = info->P[0];
		fy_ = info->P[5];
		cx_ = info->P[2];
		cy_ = info->P[6]-4;

		if(fx_ == 0 || fy_==0)
		{
			printf("camera calib from P seems wrong, trying calib from K\n");
			fx_ = info->K[0];
			fy_ = info->K[4];
			cx_ = info->K[2];
			cy_ = info->K[5]-4;
		}

		width_ = info->width;
		height_ = info->height-8;

		printf("Received ROS Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",fx_,fy_,cx_,cy_,width_,height_);
	}
}

}
