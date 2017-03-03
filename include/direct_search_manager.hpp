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

#include <direct_search_handler.hpp>
#include <asr_world_model/GetViewportList.h>
#include "asr_msgs/AsrViewport.h"

#include <sensor_msgs/JointState.h>
#include <asr_next_best_view/RobotStateMessage.h>
#include <asr_next_best_view/SetInitRobotState.h>
#include <asr_next_best_view/SetAttributedPointCloud.h>
#include <asr_next_best_view/RateViewports.h>

#include "filter/filter_basic.hpp"
#include "filter/filter_poses_depending_on_already_found_object_types.hpp"
#include "filter/do_filter_iteration.hpp"
#include "filter/concat_approx_equale_poses.hpp"

#include <ISM/utility/TableHelper.hpp>


namespace directSearchWS {

class DirectSearchManager : public DirectSearchHandler {

private:
    std::string dbfilenameParam;

    ros::ServiceClient getViewportListServiceClient;

    ros::Subscriber ptuDriverStateSubscriber;
    ros::ServiceClient setInitRobotStateServiceClient;
    ros::ServiceClient setAttributedPointCloudServiceClient;
    ros::ServiceClient rateViewportsServiceClient;

    DoFilterIterationPtr doFilterIterationPtrForPosesDependingOnAlreadyFoundObjectTypes;

    double currentPan;
    double currentTilt;
    bool needSortByNBV;

    PosePtr parsePose(const rapidxml::xml_node<> *node, const char *poseName) const;
    bool getViewportsFromWorldModel(std::vector<asr_msgs::AsrViewport> &viewports);

    bool reorderPosesByNBV(const SearchedObjectTypes &searchedObjectTypes);
    bool deleteAllRedundantPoses(const RobotStatePtrVecPtr &robotStates) const;
    bool filterPosesDependingOnAlreadyFoundObjectTypes(const RobotStatePtrVecPtr &robotStates, const SearchedObjectTypes &searchedObjectTypes) const;

    bool setInitialRobotState();
    bool setPointCloudInNBV(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds);
    void ptuDriverStateCallback(const sensor_msgs::JointState::ConstPtr &ptuState);

protected:
    SearchedObjectTypes lastSearchedObjectTypes;

    bool reorderPosesByNBVParam;

    FilterPosesDependingOnAlreadyFoundObjectTypesPtr filterPosesDependingOnAlreadyFoundObjectTypesPtr;

    bool parsePosesToExploreFromXML(const std::string &path);

public:
    DirectSearchManager();
    virtual ~DirectSearchManager();

    virtual bool initHandler() = 0;
    virtual bool resetHandler();
    virtual bool getNextRobotState(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds);
    virtual bool backToInitial(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds);

};

SearchedObjectTypes getSearchedObjectTypesFromAWithoutB(const SearchedObjectTypes &a, const SearchedObjectTypes &b);
bool checkSearchedObjectTypesAreEquale(const SearchedObjectTypes &searchecObjectTypes1, const SearchedObjectTypes &searchecObjectTypes2);

}


