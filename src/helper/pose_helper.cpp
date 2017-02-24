/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "helper/pose_helper.hpp"

namespace directSearchWS {

PoseHelperPtr PoseHelper::instancePtr;
double PoseHelper::viewPointDistance;
int PoseHelper::distanceFuncParam;
double PoseHelper::viewCenterPositionDistanceThreshold;
double PoseHelper::viewCenterOrientationRadDistanceThreshold;

PoseHelperPtr PoseHelper::getInstance() {
    if (!instancePtr) {
        instancePtr = PoseHelperPtr(new PoseHelper());
        resetInstance();
    }
    return instancePtr;
}

void PoseHelper::resetInstance() {
    if (instancePtr) {
        ros::NodeHandle n(ros::this_node::getName());
        n.getParam("distanceFunc", distanceFuncParam);
        ROS_INFO_STREAM("Param: distanceFunc: " << distanceFuncParam << " (1: GetDistance from robot_model_services, 2: euclidean distance)");

        double ncp = waitForParam<double>(n, "/nbv/ncp");
        double fcp = waitForParam<double>(n, "/nbv/fcp");
        viewPointDistance = (ncp + fcp) / 2.0;
        ROS_INFO_STREAM("Param: viewPointDistance: " << viewPointDistance << " (== (/nbv/ncp + /nbv/fcp) / 2 = (" << ncp << "+" << fcp << ")/2");

        n.getParam("viewCenterPositionDistanceThreshold", viewCenterPositionDistanceThreshold);
        ROS_INFO_STREAM("Param: viewCenterPositionDistanceThreshold: " << viewCenterPositionDistanceThreshold);

        viewCenterOrientationRadDistanceThreshold = waitForParam<double>(n, "/nbv/mHypothesisUpdaterAngleThreshold");
        viewCenterOrientationRadDistanceThreshold *=  DEG_TO_RAD;
        ROS_INFO_STREAM("Param: viewCenterOrientationRadDistanceThreshold (/nbv/mHypothesisUpdaterAngleThreshold): " << viewCenterOrientationRadDistanceThreshold);
    }
}

template <typename T>
T PoseHelper::waitForParam(ros::NodeHandle n, std::string param) {
    if (!n.hasParam(param)) {
        ROS_INFO_STREAM("Wait for param: " << param);
        while (!n.hasParam(param)) {
            ros::Duration(0.1).sleep();
        }
    }
    T value;
    n.getParam(param, value);
    return value;
}

PoseHelper::PoseHelper() {
    ros::NodeHandle n(ros::this_node::getName());
    getDistanceServiceClient = n.serviceClient<robot_model_services::GetDistance>("/asr_robot_model_services/GetDistance");
    getRobotPoseServiceClient = n.serviceClient<robot_model_services::GetPose>("/asr_robot_model_services/GetRobotPose");
    getCameraPoseServiceClient = n.serviceClient<robot_model_services::GetPose>("/asr_robot_model_services/GetCameraPose");
}


double PoseHelper::calcDistPositionWithNBV(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
    robot_model_services::GetDistance srv;
    srv.request.sourcePosition.x = pose1.position.x;
    srv.request.sourcePosition.y = pose1.position.y;
    srv.request.sourcePosition.z = pose1.position.z;
    srv.request.targetPosition.x = pose2.position.x;
    srv.request.targetPosition.y = pose2.position.y;
    srv.request.targetPosition.z = pose2.position.z;
    if (getDistanceServiceClient.call(srv)) {
        return srv.response.distance;
    } else {
        ROS_ERROR("Failed to call service GetDistance, using euclian Distance instead");
        return calcDistPositionEucl(pose1, pose2);
    }
}

double PoseHelper::calcDistPositionEucl(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
    return sqrt(pow(pose1.position.x - pose2.position.x, 2)
                + pow(pose1.position.y - pose2.position.y, 2)
                + pow(pose1.position.z - pose2.position.z, 2));
}

double PoseHelper::calcAngularDistanceInRad(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
    Eigen::Quaterniond rotation1 = convertPoseQuatToQuat(pose1);
    rotation1.normalize();
    Eigen::Quaterniond rotation2 = convertPoseQuatToQuat(pose2);
    rotation2.normalize();
    return rotation1.angularDistance(rotation2);
}

Eigen::Vector3d PoseHelper::calculateViewCenterPoint(const geometry_msgs::Pose &pose) {
    Eigen::Quaterniond targetOrientation = convertPoseQuatToQuat(pose);
    Eigen::Vector3d targetViewVector = targetOrientation * Eigen::Vector3d::UnitX();
    targetViewVector.normalize();
    Eigen::Vector3d target_view_center_point = convertPositionToVec(pose) + targetViewVector * viewPointDistance;
    return target_view_center_point;
}

geometry_msgs::Pose PoseHelper::getCurrentRobotPose() {
    robot_model_services::GetPose srv;
    if (getRobotPoseServiceClient.call(srv)) {
        ROS_DEBUG_STREAM("CurrentRobotPose:\n" << srv.response.pose);
        return srv.response.pose;
    } else {
        ROS_ERROR("Failed to call service GetRobotPose, using origin instead");
        return getOriginPose();
    }
}

geometry_msgs::Pose PoseHelper::getCurrentCameraPose() {
    robot_model_services::GetPose srv;
    if (getCameraPoseServiceClient.call(srv)) {
        ROS_DEBUG_STREAM("CurrentCameraPose:\n" << srv.response.pose);
        return srv.response.pose;
    } else {
        ROS_ERROR("Failed to call service GetCameraPose, using origin instead");
        return getOriginPose();
    }
}

double PoseHelper::calculateDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
    if (distanceFuncParam == 1) {
        return calcDistPositionWithNBV(pose1, pose2);
    } else {
        return calcDistPositionEucl(pose1, pose2);
    }
}

bool PoseHelper::checkViewCenterPointsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
    Eigen::Vector3d viewCenterPoint1 = calculateViewCenterPoint(pose1);
    Eigen::Vector3d viewCenterPoint2 = calculateViewCenterPoint(pose2);

    return checkPositionsAreApproxEquale(convertVecToPosition(viewCenterPoint1), convertVecToPosition(viewCenterPoint2), viewCenterPositionDistanceThreshold)
            && checkOrientationsAreApproxEquale(pose1, pose2, viewCenterOrientationRadDistanceThreshold);
}

bool PoseHelper::checkPosesAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2,
                                           const double positionThreshold, const double orientationRadThreshold) {
    return checkPositionsAreApproxEquale(pose1, pose2, positionThreshold)
            && checkOrientationsAreApproxEquale(pose1, pose2, orientationRadThreshold);
}

bool PoseHelper::checkPositionsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double positionThreshold) {
    const double distance_position = calcDistPositionEucl(pose1, pose2);
    return positionThreshold > distance_position;
}

bool PoseHelper::checkOrientationsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double orientationRadThreshold) {
    const double distance_orientation_rad = calcAngularDistanceInRad(pose1, pose2);
    return orientationRadThreshold > distance_orientation_rad;
}

}



