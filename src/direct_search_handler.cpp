/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "direct_search_handler.hpp"

namespace directSearchWS {

DirectSearchHandler::DirectSearchHandler() : posesToExplorePtr(new RobotStatePtrVec()),
    nextCameraPosePtr(), nextRobotPosePtr(), nextPtuPtr(), nextFilteredSearchedObjectTypesAndIds(), remainingPosesDistances(0.0),
    isSameRobotPoseAsBefore(false), isRobotPoseChanged(true), isNoPoseLeft(false), arePosesFromDemonstrationLeft(false), poseHelperPtr(PoseHelper::getInstance()) {
}

DirectSearchHandler::~DirectSearchHandler() { }

bool DirectSearchHandler::resetHandler() {
    checkParametersFromOtherNode();
    PoseHelper::resetInstance();
    posesToExplorePtr->clear();
    nextCameraPosePtr.reset();
    nextRobotPosePtr.reset();
    nextPtuPtr.reset();
    nextFilteredSearchedObjectTypesAndIds.clear();
    remainingPosesDistances = 0.0;
    isSameRobotPoseAsBefore = false;
    isRobotPoseChanged = true;
    isNoPoseLeft = false;
    arePosesFromDemonstrationLeft = false;
    return true;
}

bool DirectSearchHandler::getNextRobotState(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds) {
    if (posesToExplorePtr->empty()) {
        ROS_INFO("Get next robot state called, when no poses were left");
        isNoPoseLeft = true;
        return true;
    } else {
        isNoPoseLeft = false;
    }

    nextRobotPosePtr = posesToExplorePtr->front()->getRobotPosePtr();
    isSameRobotPoseAsBefore = !isRobotPoseChanged;

    if (posesToExplorePtr->front()->getPtuListSize() == 0) {
        ROS_ERROR("It is not allowed to have a robot_pose with no PTU config left!");
        return false;
    }

    nextPtuPtr = posesToExplorePtr->front()->getTopPtuTuplePtr();
    posesToExplorePtr->front()->eraseFrontPTUTuplePtr();
    nextCameraPosePtr = nextPtuPtr->getCameraPosePtr();

    nextFilteredSearchedObjectTypesAndIds = searchedObjectTypesAndIds;
    // Erase the object_types in the next view which were already searched there
    filterSearchedObjectTypesAndIds(nextFilteredSearchedObjectTypesAndIds, nextPtuPtr->getAlreadySearchedObjectTypes());

    if (posesToExplorePtr->front()->getPtuListSize() == 0) {
        ROS_INFO("Get next robot state called, when only one PTU tuple was left");
        posesToExplorePtr->erase(posesToExplorePtr->begin());
        isRobotPoseChanged = true;
        if (!posesToExplorePtr->empty()) {
            remainingPosesDistances -= poseHelperPtr->calculateDistance(*nextRobotPosePtr, *posesToExplorePtr->front()->getRobotPosePtr());
            arePosesFromDemonstrationLeft = posesToExplorePtr->front()->getTopPtuTuplePtr()->getRatedViewport().rating > 0;
        } else {
            arePosesFromDemonstrationLeft = false;
            remainingPosesDistances = 0.0;
        }
        return true;
    } else {
        ROS_INFO("Get next robot state called, when more than one PTU tuple was left");
        arePosesFromDemonstrationLeft = posesToExplorePtr->front()->getTopPtuTuplePtr()->getRatedViewport().rating > 0;
        isRobotPoseChanged = false;
        return true;
    }
}

bool DirectSearchHandler::backToInitial(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds) {
    isRobotPoseChanged = true;
    double distance = DBL_MAX;
    const geometry_msgs::Pose &robotPose = poseHelperPtr->getCurrentRobotPose();
    RobotStatePtrVec::iterator saved_it = posesToExplorePtr->begin();
    for (RobotStatePtrVec::iterator it = posesToExplorePtr->begin(); it != posesToExplorePtr->end(); ++it) {
        double loop_distance = poseHelperPtr->calculateDistance(robotPose, *it->get()->getRobotPosePtr());
        if (loop_distance < distance) {
            distance = loop_distance;
            saved_it = it;
        }
    }
    if (saved_it != posesToExplorePtr->begin()) {
        ROS_INFO("Poses to explore have been reordered.");
        RobotStatePtrVecPtr buffer_vectorPtr = RobotStatePtrVecPtr(new RobotStatePtrVec());
        buffer_vectorPtr->insert(buffer_vectorPtr->end(),saved_it, posesToExplorePtr->end());
        buffer_vectorPtr->insert(buffer_vectorPtr->end(),posesToExplorePtr->begin(), saved_it);
        posesToExplorePtr->clear();
        posesToExplorePtr = buffer_vectorPtr;
    }

    return true;
}

void DirectSearchHandler::calculateRemainingPosesDistances() {
    remainingPosesDistances = 0.0;
    if (posesToExplorePtr->size() <= 1) {
        return;
    }
    for (RobotStatePtrVec::iterator it = posesToExplorePtr->begin(); it != posesToExplorePtr->end() - 1; ++it) {
        remainingPosesDistances += poseHelperPtr->calculateDistance(*it->get()->getRobotPosePtr(), *(it+1)->get()->getRobotPosePtr());
    }
    ROS_INFO_STREAM("new calculated remainingPosesDistances: " << remainingPosesDistances);
}


void filterSearchedObjectTypes(SearchedObjectTypes &searched_object_types_to_filter, const SearchedObjectTypes &filter_searched_object_types) {
    for (const std::string &filter_searched_object_type : filter_searched_object_types) {
        bool isFound = false;
        do {
            SearchedObjectTypes::iterator iter = std::find(searched_object_types_to_filter.begin(), searched_object_types_to_filter.end(), filter_searched_object_type);
            if (iter != searched_object_types_to_filter.end()) {
                searched_object_types_to_filter.erase(iter);
                isFound = true;
            }  else {
                isFound = false;
            }
        } while (isFound);
    }
}

void filterSearchedObjectTypesAndIds(SearchedObjectTypesAndIds &searched_object_types_and_ids_to_filter, const SearchedObjectTypes &filter_searched_object_types) {
    for (const std::string &filter_searched_object_type : filter_searched_object_types) {
        SearchedObjectTypesAndIds::iterator searched_object_type_and_id_to_filter_iter = searched_object_types_and_ids_to_filter.begin();
        while (searched_object_type_and_id_to_filter_iter != searched_object_types_and_ids_to_filter.end()) {
            if (searched_object_type_and_id_to_filter_iter->type == filter_searched_object_type) {
                searched_object_type_and_id_to_filter_iter = searched_object_types_and_ids_to_filter.erase(searched_object_type_and_id_to_filter_iter);
            } else {
                ++searched_object_type_and_id_to_filter_iter;
            }
        }
    }
}

SearchedObjectTypes getIntersectionOfSearchObjectTypes(const SearchedObjectTypes &first_searched_object_types,
                                                       const SearchedObjectTypes &second_searched_object_types) {
    SearchedObjectTypes resultSearchedObjectTypes;

    SearchedObjectTypes copySearchecObjectTypes1(first_searched_object_types);
    SearchedObjectTypes copySearchecObjectTypes2(second_searched_object_types);
    std::sort(copySearchecObjectTypes1.begin(), copySearchecObjectTypes1.end());
    std::sort(copySearchecObjectTypes2.begin(), copySearchecObjectTypes2.end());

    std::set_intersection(copySearchecObjectTypes1.begin(), copySearchecObjectTypes1.end(),
                          copySearchecObjectTypes2.begin(), copySearchecObjectTypes2.end(),
                          std::back_inserter(resultSearchedObjectTypes));

    return resultSearchedObjectTypes;
}

void checkParametersFromOtherNode() {
    ros::NodeHandle nh(ros::this_node::getName());

    checkParameterFromOtherNode<std::string>(nh, "rp_ism_node", "dbfilename");

    checkParameterFromOtherNode<double>(nh, "asr_flir_ptu_driver", "pan_min_angle");
    checkParameterFromOtherNode<double>(nh, "asr_flir_ptu_driver", "pan_max_angle");
    checkParameterFromOtherNode<double>(nh, "asr_flir_ptu_driver", "tilt_min_angle");
    checkParameterFromOtherNode<double>(nh, "asr_flir_ptu_driver", "tilt_max_angle");
}

template <typename T>
void checkParameterFromOtherNode(ros::NodeHandle &nh, const std::string &node_name, const std::string &param_name) {
    std::string full_name = "/" + node_name + "/" + param_name;
    if (nh.hasParam(full_name)) {
        T param;
        nh.getParam(full_name, param);
        nh.setParam(param_name, param);
        ROS_INFO_STREAM("Param: " << full_name << ": " << param);
    }
}

}

