/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle& nodeHandler):
    mpSLAM(pSLAM),
    image_trans(nodeHandler),
    image_pub(image_trans.advertise("viso_image", 5)),
    pose_pub(nodeHandler.advertise<geometry_msgs::PoseStamped>("vodom", 10))
    {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;

    image_transport::ImageTransport image_trans;
    image_transport::Publisher image_pub;
    ros::Publisher pose_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ros::NodeHandle nodeHandler;

    ImageGrabber igb(&SLAM, nodeHandler);


    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if(!pose.data) {
        return;
    }

    cv::Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*pose.rowRange(0,3).col(3);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = msg->header.frame_id;

    Eigen::Matrix3f rot = Eigen::Map<Eigen::Matrix3f>(reinterpret_cast<float*>(Rwc.data)).transpose();

    Eigen::Quaternionf quat(rot);
    // Eigen::Vector3f trans(mat.block<3, 1>(0, 3));
    Eigen::Vector3f trans(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2));

    pose_msg.pose.position.x = trans.x();
    pose_msg.pose.position.y = trans.y();
    pose_msg.pose.position.z = trans.z();
    pose_msg.pose.orientation.w = quat.w();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();

    pose_pub.publish(pose_msg);

    cv::Mat frame = mpSLAM->getLastFrame();
    auto img_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    image_pub.publish(img_msg);
}


