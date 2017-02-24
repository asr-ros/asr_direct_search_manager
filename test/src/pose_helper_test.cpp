/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "helper/pose_helper.hpp"

#include <math.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <eigen3/Eigen/Geometry>

#include "next_best_view/GetDistance.h"

#define EPSILON 0.001

using namespace directSearchWS;

class PoseHelperSetup: public ::testing::Test {

public:

    ros::NodeHandle n;
    boost::thread t;

    PoseHelperSetup() : n(ros::this_node::getName()) {
    }

    void SetUp() {
        n.setParam("distanceFunc", 2);
        n.setParam("/nbv/ncp", 0.4);
        n.setParam("/nbv/fcp", 1.5);
        n.setParam("viewCenterPositionDistanceThreshold", 0.17);
        n.setParam("/nbv/mHypothesisUpdaterAngleThreshold", 45.0);

        PoseHelper::resetInstance();
    }

    void TearDown() {
    }

    ~PoseHelperSetup()  {
    }
};


TEST_F(PoseHelperSetup, CalcDistPositionEucl) {
    n.setParam("distanceFunc", 2);
    PoseHelper::resetInstance();
    PoseHelperPtr poseHelperPtr = PoseHelper::getInstance();

    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;

    EXPECT_DOUBLE_EQ(0.0, poseHelperPtr->calculateDistance(pose1, pose2));

    // orientation should not change anything
    pose1.orientation.x = 0.444;
    pose2.orientation.w = -0.666;

    EXPECT_DOUBLE_EQ(0.0, poseHelperPtr->calculateDistance(pose1, pose2));

    pose2.position.x = 13.33;
    pose2.position.y = -14.55;

    EXPECT_NEAR(19.733, poseHelperPtr->calculateDistance(pose1, pose2), EPSILON);

    pose1.position.x = -13.33;
    pose1.position.y = -14.55;

    EXPECT_DOUBLE_EQ(26.66, poseHelperPtr->calculateDistance(pose1, pose2));
}

TEST_F(PoseHelperSetup, CalcAngularDistanceInRad) {
    PoseHelperPtr poseHelperPtr = PoseHelper::getInstance();

    geometry_msgs::Pose pose1;
    pose1.orientation.w = 1.0;
    geometry_msgs::Pose pose2;
    pose2.orientation.w = 1.0;
    EXPECT_NEAR(0.0, poseHelperPtr->calcAngularDistanceInRad(pose1, pose2), EPSILON);

    // position should not change anything
    pose1.position.x = 13.33;
    pose2.position.y = -14.55;
    EXPECT_NEAR(0.0, poseHelperPtr->calcAngularDistanceInRad(pose1, pose2), EPSILON);

    // rotate x: 180°
    pose2.orientation.w = 0.0;
    pose2.orientation.x = 1.0;
    EXPECT_NEAR(180.0 * M_PI / 180.0, poseHelperPtr->calcAngularDistanceInRad(pose1, pose2), EPSILON);

    // rotate x: 90° and y: -90°
    pose2.orientation.w = 0.5;
    pose2.orientation.x = 0.5;
    pose2.orientation.y = -0.5;
    pose2.orientation.z = -0.5;
    EXPECT_NEAR(120.0 * M_PI / 180.0, poseHelperPtr->calcAngularDistanceInRad(pose1, pose2), EPSILON);
}

TEST_F(PoseHelperSetup, CheckViewCenterPointsAreApproxEquale) {
    PoseHelperPtr poseHelperPtr = PoseHelper::getInstance();

    geometry_msgs::Pose pose1;
    pose1.orientation.w = 1.0;
    geometry_msgs::Pose pose2;
    pose2.orientation.w = 1.0;
    EXPECT_TRUE(poseHelperPtr->checkViewCenterPointsAreApproxEquale(pose1, pose2));

    // 0.95 is viewCenterDistance and 0.16 is distance between viewpoints < viewCenterPositionDistanceThreshold ~= 0.176
    // rotate z: arctan(0.16/0.95) == 9.56° < 60° == mHypothesisUpdaterAngleThreshold
    pose2.orientation.w = 0.997;
    pose2.orientation.z = 0.083;
    EXPECT_TRUE(poseHelperPtr->checkViewCenterPointsAreApproxEquale(pose1, pose2));

    // 0.95 is viewCenterDistance and 0.19 is distance between viewpoints > viewCenterPositionDistanceThreshold ~= 0.176
    // rotate z: arctan(0.19/0.95) == 11.31° < 60° == mHypothesisUpdaterAngleThreshold
    pose2.orientation.w = 0.995;
    pose2.orientation.z = 0.099;
    EXPECT_FALSE(poseHelperPtr->checkViewCenterPointsAreApproxEquale(pose1, pose2));

    // 0.95 is viewCenterDistance
    pose2.position.x = 0.95;
    pose2.position.y = -0.95;
    // rotate z: 90° > 60° == mHypothesisUpdaterAngleThreshold
    pose2.orientation.w = 0.707;
    pose2.orientation.z = 0.707;
    EXPECT_FALSE(poseHelperPtr->checkViewCenterPointsAreApproxEquale(pose1, pose2));

    // 0.95 is viewCenterDistance
    pose2.position.x = 0.95 * 2;
    pose2.position.y = 0.0;
    // rotate z: 180°
    pose2.orientation.w = 0.0;
    pose2.orientation.z = 1.0;
    EXPECT_FALSE(poseHelperPtr->checkViewCenterPointsAreApproxEquale(pose1, pose2));
}


bool getDistance(next_best_view::GetDistance::Request &req, next_best_view::GetDistance::Response &res) {
    return true;
}

bool hasAdvertised;

int advertiseService() {
    ros::NodeHandle n(ros::this_node::getName());
    ros::ServiceServer service_GetDistance = n.advertiseService("/asr_robot_model_services/GetDistance", getDistance);
    while (true) {
        ros::spinOnce();
        hasAdvertised = true;
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
    return 0;
}

TEST_F(PoseHelperSetup, ResetInstance) {
    n.setParam("distanceFunc", 2);
    PoseHelper::resetInstance();
    PoseHelperPtr poseHelperPtr = PoseHelper::getInstance();

    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;

    pose2.position.x = 1.0;
    pose2.position.y = 0.0;

    EXPECT_NEAR(1.0, poseHelperPtr->calculateDistance(pose1, pose2), EPSILON);

    n.setParam("distanceFunc", 1);
    PoseHelper::resetInstance();

    hasAdvertised = false;
    boost::thread t = boost::thread(advertiseService);
    while (!hasAdvertised) {}

    EXPECT_NEAR(0.0, poseHelperPtr->calculateDistance(pose1, pose2), EPSILON);
    t.interrupt();
    t.join();

}


