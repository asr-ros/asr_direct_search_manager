/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Pose.h>
#include <direct_search_manager/directSearchAction.h>
#include <dynamic_reconfigure/server.h>

#include <direct_search_manager/DynamicParametersConfig.h>

#include "direct_search_handler.hpp"
#include "grid_initialisation.hpp"
#include "grid_manager.hpp"
#include "recording_manager.hpp"
#include "model/ptu_tuple.hpp"


namespace directSearchWS {

class directSearchAction {
private:

    boost::mutex mtx_;
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<direct_search_manager::directSearchAction> as_;
    // create messages that are used to published feedback/result
    direct_search_manager::directSearchFeedback feedback_;
    direct_search_manager::directSearchResult result_;
    DirectSearchHandlerPtr directSearchHandlerPtr;

    dynamic_reconfigure::Server<direct_search_manager::DynamicParametersConfig> mDynamicReconfigServer;

    void initSearchMode() {
        int directSearchMode;
        nh_.getParam("directSearchMode", directSearchMode);
        ROS_INFO_STREAM("Param: directSearchMode: " << directSearchMode);

        if (directSearchMode == 1) {
            ROS_INFO_STREAM("direct search runs with grid_manager");
            std::string initializedGridFilePath;
            nh_.getParam("initializedGridFilePath", initializedGridFilePath);
            ROS_INFO_STREAM("Initialized grid file path: " << initializedGridFilePath);
            directSearchHandlerPtr = DirectSearchHandlerPtr(new GridManager(initializedGridFilePath));
        } else if (directSearchMode == 2) {
            ROS_INFO_STREAM("direct search runs with recording_manager");
            std::string recordFilePath;
            nh_.getParam("recordFilePath", recordFilePath);
            ROS_INFO_STREAM("Record file path: " << recordFilePath);
            directSearchHandlerPtr = DirectSearchHandlerPtr(new RecordingManager(recordFilePath));
        } else {
            ROS_INFO_STREAM("direct search runs with grid_initialisation");
            std::string gridFilePath;
            nh_.getParam("gridFilePath", gridFilePath);
            ROS_INFO_STREAM("Grid file path: " << gridFilePath);
            directSearchHandlerPtr = DirectSearchHandlerPtr(new GridInitialisation(gridFilePath));
        }

        directSearchHandlerPtr->initHandler();
        ROS_INFO_STREAM("Initialization finished");
    }


    void dynamicReconfigureCallback(direct_search_manager::DynamicParametersConfig &config, uint32_t level) {
        ROS_INFO_STREAM("Dynamic reconfigure called with level: " << level);
        boost::lock_guard<boost::mutex> guard(mtx_);

        nh_.setParam("fovH", config.fovH);
        nh_.setParam("fovV", config.fovV);
        nh_.setParam("clearVision", config.clearVision);

        nh_.setParam("directSearchMode", config.directSearchMode);
        nh_.setParam("distanceFunc", config.distanceFunc);

        nh_.setParam("reorderPosesByNBV", config.reorderPosesByNBV);
        nh_.setParam("reorderPosesByTSP", config.reorderPosesByTSP);

        nh_.setParam("viewCenterPositionDistanceThreshold", config.viewCenterPositionDistanceThreshold);

        nh_.setParam("filterMinimumNumberOfDeletedNormals", config.filterMinimumNumberOfDeletedNormals);
        nh_.setParam("filterIsPositionAllowed", config.filterIsPositionAllowed);
        nh_.setParam("concatApproxEqualsPoses", config.concatApproxEqualsPoses);

        nh_.setParam("concatRobotPosePositionDistanceThreshold", config.concatRobotPosePositionDistanceThreshold);
        nh_.setParam("concatRobotPoseOrientationRadDistanceThreshold", config.concatRobotPoseOrientationRadDistanceThreshold);

        nh_.setParam("gridFilePath", config.gridFilePath);
        nh_.setParam("initializedGridFilePath", config.initializedGridFilePath);
        nh_.setParam("recordFilePath", config.recordFilePath);

        checkParametersFromOtherNode();
        PoseHelper::resetInstance();
        initSearchMode();
    }

public:

    directSearchAction() :
        nh_(ros::this_node::getName()), as_(nh_, ros::this_node::getName(), boost::bind(&directSearchAction::executeCB, this, _1), false),
        feedback_(), result_(), directSearchHandlerPtr(), mDynamicReconfigServer(nh_) {

        ROS_INFO_STREAM("Begin of initialization");

        dynamic_reconfigure::Server<direct_search_manager::DynamicParametersConfig>::CallbackType f = boost::bind(&directSearchAction::dynamicReconfigureCallback, this, _1, _2);
        // setCallback calls the callback at first start
        mDynamicReconfigServer.setCallback(f);

        as_.start();
    }

    bool setResults() {
        result_.remainingPTUPoses = directSearchHandlerPtr->getRemainingPTUPoses();
        result_.remainingRobotPoses = directSearchHandlerPtr->getRemainingRobotPoses();
        result_.remainingPosesDistances = directSearchHandlerPtr->getRemainingPosesDistances();
        result_.isNoPoseLeft = directSearchHandlerPtr->getIsNoPoseLeft();
        result_.isSameRobotPoseAsBefore = directSearchHandlerPtr->getIsSameRobotPoseAsBefore();
        result_.arePosesFromDemonstrationLeft = directSearchHandlerPtr->getArePosesFromDemonstrationLeft();

        result_.filteredSearchedObjectTypesAndIds = directSearchHandlerPtr->getActualFilteredSearchedObjectTypesAndIds();

        PosePtr actualRobotPose = directSearchHandlerPtr->getActualRobotPosePtr();
        if (actualRobotPose) {
            result_.goalRobotPose = *actualRobotPose;
        } else {
            result_.goalRobotPose = geometry_msgs::Pose();
        }

        PosePtr actualCameraPose = directSearchHandlerPtr->getActualCameraPosePtr();
        if (actualCameraPose) {
            result_.goalCameraPose = *actualCameraPose;
        } else {
            result_.goalCameraPose = geometry_msgs::Pose();
        }

        PtuTuplePtr actualPTU = directSearchHandlerPtr->getActualPtuPtr();
        if (actualPTU) {
            result_.pan = actualPTU->getPan();
            result_.tilt = actualPTU->getTilt();
        } else {
            result_.pan = 0;
            result_.tilt = 0;
        }

        ROS_DEBUG_STREAM("result:\n" << result_);
        return true;
    }

    void executeCB(const direct_search_manager::directSearchGoalConstPtr &goal) {
        ROS_INFO_STREAM("Received command: " << goal->command);
        boost::lock_guard<boost::mutex> guard(mtx_);

        const std::string &command = goal->command;
        bool success = true;
        if (command == "Reset") {
            success &= directSearchHandlerPtr->resetHandler();
        } else if(command == "BackToInitial") {
            success &= directSearchHandlerPtr->backToInitial(goal->searchedObjectTypesAndIds);
        } else if(command == "GetGoalCameraPose") {
            success &= directSearchHandlerPtr->getNextRobotState(goal->searchedObjectTypesAndIds);
        } else {
            ROS_WARN("Unknown command!");
            return;
        }

        success &= setResults();
        if (success) {
            as_.setSucceeded(result_);
        } else {
            as_.setAborted(result_);
        }

    }

};

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "direct_search_manager");
    ros::start();

    directSearchWS::directSearchAction direct_search_manager;
    ros::spin();

    return 0;
}



