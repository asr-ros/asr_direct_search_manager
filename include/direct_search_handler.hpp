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

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <tuple>
#include <memory>

#include <ros/ros.h>
#include <rapidxml.hpp>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Geometry>

#include "tf/transform_datatypes.h"
#include "asr_msgs/AsrTypeAndId.h"

#include "model/robot_state.hpp"
#include "model/ptu_tuple.hpp"
#include "helper/pose_helper.hpp"


namespace directSearchWS {

typedef std::vector<asr_msgs::AsrTypeAndId> SearchedObjectTypesAndIds;

class DirectSearchHandler {

private:

protected:
    RobotStatePtrVecPtr posesToExplorePtr;

    PosePtr nextCameraPosePtr;
    PosePtr nextRobotPosePtr;
    PtuTuplePtr nextPtuPtr;
    SearchedObjectTypesAndIds nextFilteredSearchedObjectTypesAndIds;
    double remainingPosesDistances;
    bool isSameRobotPoseAsBefore;
    bool isRobotPoseChanged;
    bool isNoPoseLeft;
    bool arePosesFromDemonstrationLeft;

    PoseHelperPtr poseHelperPtr;

    void calculateRemainingPosesDistances();

public:
    DirectSearchHandler();
    virtual ~DirectSearchHandler();

    virtual bool initHandler() = 0;
    virtual bool resetHandler();
    virtual bool getNextRobotState(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds); // decide next robot pose and ptu config
    virtual bool backToInitial(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds);

    //getter
    PosePtr getActualRobotPosePtr() const {
        return nextRobotPosePtr;
    }

    PosePtr getActualCameraPosePtr() const {
        return nextCameraPosePtr;
    }

    PtuTuplePtr getActualPtuPtr() const {
        return nextPtuPtr;
    }

    SearchedObjectTypesAndIds getActualFilteredSearchedObjectTypesAndIds() const {
        return nextFilteredSearchedObjectTypesAndIds;
    }

    bool getIsSameRobotPoseAsBefore() const {
        return isSameRobotPoseAsBefore;
    }

    bool getIsNoPoseLeft() const {
        return isNoPoseLeft;
    }

    bool getArePosesFromDemonstrationLeft() const {
        return arePosesFromDemonstrationLeft;
    }

    int getRemainingPTUPoses() const {
        if (posesToExplorePtr->empty()) {
            return 0;
        }
        return posesToExplorePtr->front()->getPtuListSize();
    }

    int getRemainingRobotPoses() const {
        return posesToExplorePtr->size();
    }

    double getRemainingPosesDistances() const {
        return remainingPosesDistances;
    }

};

typedef boost::shared_ptr<DirectSearchHandler> DirectSearchHandlerPtr;

void filterSearchedObjectTypes(SearchedObjectTypes &searched_object_types_to_filter, const SearchedObjectTypes &filter_searched_object_types);
void filterSearchedObjectTypesAndIds(SearchedObjectTypesAndIds &searched_object_types_and_ids_to_filter,
                                     const SearchedObjectTypes &filter_searched_object_types);
SearchedObjectTypes getIntersectionOfSearchObjectTypes(const SearchedObjectTypes &first_searched_object_types,
                                                       const SearchedObjectTypes &second_searched_object_types);
void checkParametersFromOtherNode();
template <typename T>
void checkParameterFromOtherNode(ros::NodeHandle &nh, const std::string &node_name, const std::string &param_name);

}

