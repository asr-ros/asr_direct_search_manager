/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <ros/ros.h>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Geometry>

#include "robot_model_services/GetDistance.h"
#include "robot_model_services/GetPose.h"
#include "robot_model_services/RobotStateMessage.h"

#define RAD_TO_DEG 180.0 / M_PI
#define DEG_TO_RAD M_PI / 180.0

namespace directSearchWS {

class PoseHelper {

private:
    static boost::shared_ptr<PoseHelper> instancePtr;
    static double viewPointDistance;
    static int distanceFuncParam;
    static double viewCenterPositionDistanceThreshold;
    static double viewCenterOrientationRadDistanceThreshold;

    ros::ServiceClient getDistanceServiceClient;
    ros::ServiceClient getRobotPoseServiceClient;
    ros::ServiceClient getCameraPoseServiceClient;

    template <typename T>
    static T waitForParam(ros::NodeHandle n, std::string param);

    //calc distance functions
    double calcDistPositionWithNBV(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);
    double calcDistPositionEucl(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);

    Eigen::Vector3d calculateViewCenterPoint(const geometry_msgs::Pose &pose);


    Eigen::Vector3d convertPositionToVec(const geometry_msgs::Pose &pose) {
        return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    }

    geometry_msgs::Pose convertVecToPosition(const Eigen::Vector3d &vec)  {
        geometry_msgs::Pose pose;
        pose.position.x = vec[0];
        pose.position.y = vec[1];
        pose.position.z = vec[2];
        return pose;
    }

    Eigen::Quaterniond convertPoseQuatToQuat(const geometry_msgs::Pose &pose) {
        return Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    }

    geometry_msgs::Pose getOriginPose() {
        geometry_msgs::Pose origin;
        origin.position.x = 0;
        origin.position.y = 0;
        origin.position.z = 0;
        origin.orientation.x = 0;
        origin.orientation.y = 0;
        origin.orientation.z = 0;
        origin.orientation.w = 1;
        return origin;
    }


    PoseHelper();

public:

    static boost::shared_ptr<PoseHelper> getInstance();
    static void resetInstance();

    geometry_msgs::Pose getCurrentRobotPose();
    geometry_msgs::Pose getCurrentCameraPose();

    double calculateDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);
    double calcAngularDistanceInRad(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);

    bool checkViewCenterPointsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);

    bool checkPosesAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double positionThreshold, const double orientationRadThreshold);
    bool checkPositionsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double positionThreshold);
    bool checkOrientationsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double orientationRadThreshold);

};

typedef boost::shared_ptr<PoseHelper> PoseHelperPtr;

}


