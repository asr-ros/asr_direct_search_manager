/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "recording_manager.hpp"

namespace directSearchWS {

RecordingManager::RecordingManager(std::string recordFilePath) : DirectSearchManager(), recordFilePath(recordFilePath) {
    ros::NodeHandle n(ros::this_node::getName());

    n.getParam("filterMinimumNumberOfDeletedNormals", filterMinimumNumberOfDeletedNormalsParam);
    ROS_INFO_STREAM("Param: filterMinimumNumberOfDeletedNormals: " << filterMinimumNumberOfDeletedNormalsParam);
    n.getParam("filterIsPositionAllowed", filterIsPositionAllowedParam);
    ROS_INFO_STREAM("Param: filterIsPositionAllowed: " << filterIsPositionAllowedParam);
    n.getParam("concatApproxEqualsPoses", concatApproxEqualsPosesParam);
    ROS_INFO_STREAM("Param: concatApproxEqualsPoses: " << concatApproxEqualsPosesParam);

    n.getParam("concatRobotPosePositionDistanceThreshold", concatRobotPosePositionDistanceThreshold);
    ROS_INFO_STREAM("Param: concatRobotPosePositionDistanceThreshold: " << concatRobotPosePositionDistanceThreshold);
    n.getParam("concatRobotPoseOrientationRadDistanceThreshold", concatRobotPoseOrientationRadDistanceThreshold);
    ROS_INFO_STREAM("Param: concatRobotPoseOrientationRadDistanceThreshold: " << concatRobotPoseOrientationRadDistanceThreshold);

    n.getParam("reorderPosesByTSP", reorderPosesByTSPParam);
    ROS_INFO_STREAM("Param: reorderPosesByTSP: " << reorderPosesByTSPParam);
    sortHelperPtr = SortHelperPtr(new SortHelper(posesToExplorePtr, reorderPosesByTSPParam));
}

RecordingManager::~RecordingManager()  { }

bool RecordingManager::resetHandler() {
    bool success = DirectSearchManager::resetHandler();
    success &= initHandler();
    return success;
}

bool RecordingManager::initHandler() {
    bool success = DirectSearchManager::parsePosesToExploreFromXML(recordFilePath);
    DoFilterIteration doFilterIteration(posesToExplorePtr);

    // TODO: A nice feature would be to filter poses where the frustum was empty while recording.

    FilterBasicPtr filterMinimumNumberOfDeletedNormalsPtr =
            FilterMinimumNumberOfDeletedNormalsPtr(new FilterMinimumNumberOfDeletedNormals(posesToExplorePtr, filterMinimumNumberOfDeletedNormalsParam));
    doFilterIteration.addFilter(filterMinimumNumberOfDeletedNormalsPtr);

    if (filterIsPositionAllowedParam) {
        FilterBasicPtr filterIsPositionAllowedPtr =
                FilterIsPositionAllowedPtr(new FilterIsPositionAllowed(posesToExplorePtr));
        doFilterIteration.addFilter(filterIsPositionAllowedPtr);
    }

    success &= doFilterIteration.doIteration();

    if (concatApproxEqualsPosesParam) {
        // start concat in seperat iteration
        DoFilterIteration concatFilterIteration(posesToExplorePtr);
        FilterBasicPtr concatApproxEqualePosesPtr = ConcatApproxEqualePosesPtr(
                    new ConcatApproxEqualePoses(posesToExplorePtr, concatRobotPosePositionDistanceThreshold, concatRobotPoseOrientationRadDistanceThreshold));
        concatFilterIteration.addFilter(concatApproxEqualePosesPtr);
        success &= concatFilterIteration.doIteration();
    }

    ROS_DEBUG_STREAM("Number of filtered posesToExplore: " << posesToExplorePtr->size());

    sortHelperPtr->nearest_neighbour_and_update_distance(remainingPosesDistances);
    sortHelperPtr->two_opt_and_update_distance(remainingPosesDistances);
    std::reverse(posesToExplorePtr->begin(), posesToExplorePtr->end());

    if (!reorderPosesByTSPParam) {
        DirectSearchHandler::calculateRemainingPosesDistances();
    }

    return success;
}

bool RecordingManager::getNextRobotState(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds) {
    bool success = DirectSearchManager::getNextRobotState(searchedObjectTypesAndIds);
    if (!reorderPosesByNBVParam && filterPosesDependingOnAlreadyFoundObjectTypesPtr->needReorder()) {
        sortHelperPtr->two_opt_and_update_distance(remainingPosesDistances);
    }
    return success;
}

bool RecordingManager::backToInitial(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds) {
    bool success = DirectSearchManager::backToInitial(searchedObjectTypesAndIds);
    if (!reorderPosesByNBVParam) {
        sortHelperPtr->two_opt_and_update_distance(remainingPosesDistances);
    }
    return success;
}

}

