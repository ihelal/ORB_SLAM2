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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/MapPoint.h"


#include "geometry_msgs/PoseStamped.h"
//#include "sensor_msgs/PointCloud.h"

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace tf;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    //ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/slam/pointcloud", 1);
//    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/slam/pos", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    while (ros::ok()) {
	    // Spin
	    ros::spinOnce();

	    // Sleep
	    loop_rate.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());


    // CAMERA POSE  rviz:  red X  green Y  blue Z

    if (pose.empty()) {
            return;
    }

    /*
    tf::Matrix3x3 rh_cameraPose(    pose.at<float>(0,0),   pose.at<float>(0,1),   pose.at<float>(0,2),
                                    pose.at<float>(1,0),   pose.at<float>(1,1),   pose.at<float>(1,2),
                                    pose.at<float>(2,0),   pose.at<float>(2,1),   pose.at<float>(2,2));

    tf::Vector3 rh_cameraTranslation( pose.at<float>(0,3),pose.at<float>(1,3), pose.at<float>(2,3) );


    tf::Quaternion q_cam, q_rot, q;
    rh_cameraPose.getRotation(q_cam);
    
    double roll=3.14159, pitch=0, yaw=0;  // Rotate the previous pose by 180* about X
	q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);
	
	q = q_rot*q_cam;  // Calculate the new orientation
    q.normalize();
    
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.pose.position.x = -rh_cameraTranslation[2];
    p.pose.position.y = rh_cameraTranslation[0];
    p.pose.position.z = rh_cameraTranslation[1];
    p.pose.orientation.x = q[1];
    p.pose.orientation.y = -q[3];
    p.pose.orientation.z = q[2];
    p.pose.orientation.w = q[0];

    pos_pub.publish(p);
    */

	
	//===================1.5708 3.14159
	
	// rotation
	tf::Matrix3x3 tf3d;
	tf3d.setValue(  pose.at<float>(0,0),   pose.at<float>(0,1),   pose.at<float>(0,2),
					pose.at<float>(1,0),   pose.at<float>(1,1),   pose.at<float>(1,2),
					pose.at<float>(2,0),   pose.at<float>(2,1),   pose.at<float>(2,2));

	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);

	tf::Quaternion t(tfqt[2], tfqt[0], tfqt[1], tfqt[3] ); // swap xyzw for zxyw

//	tf3d = tf3d.inverse();
//	tfqt *= tf::createQuaternionFromRPY(3.14159, 0, 0);
//	tfqt.normalize();

	// translation 
	tf::Vector3 origin;
//	origin.setValue( pose.at<float>(0,3),pose.at<float>(1,3), pose.at<float>(2,3) );
	origin.setValue( -pose.at<float>(2,3),pose.at<float>(0,3), pose.at<float>(1,3) ); // swap xyz for -zxy
	// TODO: this vector needs to be rotated!

	tf::Transform transform;
	transform.setRotation(t);
	transform.setOrigin(origin);
	
	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_pose"));

}
