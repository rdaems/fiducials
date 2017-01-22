/*
 * Copyright (c) 2014, Austin Hendrix
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
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
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 * Author: Austin Hendrix <namniart@gmail.com>
 */

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "fiducial_pose/Fiducial.h"
#include "fiducial_pose/FiducialTransform.h"
#include "fiducial_pose/FiducialTransformArray.h"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <aruco_detect/shared_queue.h>

#include <list>
#include <string>

using namespace std;
using namespace cv;

struct DetectedMarkers {
    std_msgs::Header header;
    vector <int>  ids;
    vector <vector <Point2f> > corners;
    cv_bridge::CvImagePtr cv_ptr;
};

class FiducialsNode {
  private:
    ros::Publisher vertices_pub;
    ros::Publisher pose_pub;

    ros::Subscriber caminfo_sub;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;
  

    // if set, we publish the images that contain fiducials
    bool publish_images;

    double fiducial_len;
    
    bool haveCamInfo;
    cv::Mat K;
    cv::Mat dist;
    int frameNum;
    std::string frameId;
  
    image_transport::Publisher image_pub;

    cv::Ptr<aruco::DetectorParameters> detectorParams;
    cv::Ptr<aruco::Dictionary> dictionary;

    void imageCallback(const sensor_msgs::ImageConstPtr & msg);
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg);

    void processImage(const sensor_msgs::ImageConstPtr &msg);
    void processDetected();

    shared_queue<sensor_msgs::ImageConstPtr> input_images;
    shared_queue<DetectedMarkers> input_detected;

  public:
    FiducialsNode(ros::NodeHandle &nh);

    void run();
};


void FiducialsNode::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (haveCamInfo) {
        return;
    }

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            K.at<double>(i, j) = msg->K[i*3+j];
        }
    }

    for (int i=0; i<5; i++) {
        dist.at<double>(0,i) = msg->D[i];
    }

    haveCamInfo = true;
    frameId = msg->header.frame_id;
}

void FiducialsNode::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    ROS_INFO("Got image");
    frameNum++;

    input_images.push(msg); 
}

void FiducialsNode::processImage(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        DetectedMarkers detected;
        detected.header = msg->header;
        detected.cv_ptr = cv_ptr;

        aruco::detectMarkers(cv_ptr->image, dictionary, detected.corners, detected.ids, detectorParams);
        ROS_INFO("Detectd %d markers", (int)detected.ids.size());
 
        for (int i=0; i<detected.ids.size(); i++) {
            fiducial_pose::Fiducial fid;
            fid.header.stamp = msg->header.stamp;
            fid.header.frame_id =frameId;
            fid.image_seq = msg->header.seq;
            fid.fiducial_id = detected.ids[i];
            
            fid.x0 = detected.corners[i][0].x;
            fid.y0 = detected.corners[i][0].y;
            fid.x1 = detected.corners[i][1].x;
            fid.y1 = detected.corners[i][1].y;
            fid.x2 = detected.corners[i][2].x;
            fid.y2 = detected.corners[i][2].y;
            fid.x3 = detected.corners[i][3].x;
            fid.y3 = detected.corners[i][3].y;

            vertices_pub.publish(fid);
        }

        input_detected.push(detected);
    }
     catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
     catch(cv::Exception & e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}

void FiducialsNode::processDetected() {
    while (ros::ok()) {
        DetectedMarkers detected = input_detected.wait_and_front_pop();
        try {
            vector <Vec3d>  rvecs, tvecs;
            fiducial_pose::FiducialTransformArray fta;
            fta.header.stamp = detected.header.stamp;
            fta.header.frame_id = frameId;
            fta.image_seq = detected.header.seq;

            if (!haveCamInfo) {
                if (frameNum > 5) {
                    ROS_ERROR("No camera intrinsics");
                }
                return;
            }   

            aruco::estimatePoseSingleMarkers(detected.corners, fiducial_len, K, dist, rvecs, tvecs);
            if(detected.ids.size() > 0) {
                aruco::drawDetectedMarkers(detected.cv_ptr->image, detected.corners, detected.ids);
            }

            for (int i=0; i<detected.ids.size(); i++) {
                aruco::drawAxis(detected.cv_ptr->image, K, dist, rvecs[i], tvecs[i], fiducial_len);

                ROS_INFO("Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", detected.ids[i],
                         tvecs[i][0], tvecs[i][1], tvecs[i][2],
                         rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                double angle = norm(rvecs[i]);
                Vec3d axis = rvecs[i] / angle;
                ROS_INFO("angle %f axis %f %f %f", angle, axis[0], axis[1], axis[2]);

                fiducial_pose::FiducialTransform ft;
                ft.fiducial_id = detected.ids[i];

                ft.transform.translation.x = tvecs[i][0];
                ft.transform.translation.y = tvecs[i][1];
                ft.transform.translation.z = tvecs[i][2];
                
                tf2::Quaternion q;
                q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                ft.transform.rotation.w = q.w();
                ft.transform.rotation.x = q.x();
                ft.transform.rotation.y = q.y();
                ft.transform.rotation.z = q.z();

                fta.transforms.push_back(ft);

            }

            image_pub.publish(detected.cv_ptr->toImageMsg());

            pose_pub.publish(fta);
        }
         catch(cv_bridge::Exception & e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
         catch(cv::Exception & e) {
            ROS_ERROR("cv exception: %s", e.what());
        }
    }
}

void FiducialsNode::run() {
    while (ros::ok()) {
        processImage(input_images.wait_and_front_pop());
        boost::thread thread(&FiducialsNode::processDetected, this);
    }
}

FiducialsNode::FiducialsNode(ros::NodeHandle & nh) : it(nh)
{
    frameNum = 0;

    // Camera intrinsics
    K = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    dist = cv::Mat::zeros(1, 5, CV_64F);
  
    haveCamInfo = false;

    int dicno;

    nh.param<bool>("publish_images", publish_images, false);
    nh.param<double>("fiducial_len", fiducial_len, 0.14);
    nh.param<int>("dictionary", dicno, 7);

    image_pub = it.advertise("/fiducial_images", 1);

    vertices_pub = ros::Publisher(nh.advertise<fiducial_pose::Fiducial>("/fiducial_vertices", 1));

    pose_pub = ros::Publisher(nh.advertise<fiducial_pose::FiducialTransformArray>("/fiducial_transforms", 1)); 
    
    dictionary = aruco::getPredefinedDictionary(dicno);

    detectorParams = new aruco::DetectorParameters();
    detectorParams->doCornerRefinement = true;

    img_sub = it.subscribe("/camera", 1,
                           &FiducialsNode::imageCallback, this);

    caminfo_sub = nh.subscribe("/camera_info", 1,
			       &FiducialsNode::camInfoCallback, this);

    ROS_INFO("Aruco detection ready");
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "aruco_detect");
    ros::NodeHandle nh("~");

    FiducialsNode node(nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    node.run();

    return 0;
}
