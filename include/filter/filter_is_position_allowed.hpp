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

#include "filter/filter_basic.hpp"
#include "robot_model_services/IsPositionAllowed.h"

namespace directSearchWS {

class FilterIsPositionAllowed : public FilterBasic {

private:
    int deleteCount;
    bool wasSuccessful;

    ros::ServiceClient isPositionAllowedServiceClient;

public:
    FilterIsPositionAllowed(const RobotStatePtrVecPtr &posesToExplorePtr) : FilterBasic(posesToExplorePtr), deleteCount(0), wasSuccessful(true) {
        ros::NodeHandle n(ros::this_node::getName());
        isPositionAllowedServiceClient = n.serviceClient<robot_model_services::IsPositionAllowed>("/asr_robot_model_services/IsPositionAllowed");
    }

    virtual ~FilterIsPositionAllowed() { }

    bool needIteration() {
        deleteCount = 0;
        wasSuccessful = true;
        isPositionAllowedServiceClient.waitForExistence();
        return true;
    }

    bool wasIterationSuccessful() {
        ROS_INFO_STREAM("Number of deleted ptuTuple by FilterIsPositionAllowed: " << deleteCount);
        return wasSuccessful;
    }

    bool shouldPtuTupleBeDeleted(const RobotStatePtrVec::iterator &posesToExploreIter, const PtuTuplePtrVec::iterator &ptuTuplePtrIter) {
        bool shouldDelete = false;
        robot_model_services::IsPositionAllowed isPositionAllowedCall;
        isPositionAllowedCall.request.targetPosition = posesToExploreIter->get()->getRobotPosePtr()->position;
        if (isPositionAllowedServiceClient.call(isPositionAllowedCall)) {
            shouldDelete = !isPositionAllowedCall.response.isAllowed;
        } else {
            ROS_ERROR_STREAM("Service IsPositionAllowed to robot_model_services was not successful -> no filter of FilterIsPositionAllowed");
            wasSuccessful = false;
        }

        if (shouldDelete) {
            ++deleteCount;
        }
        return shouldDelete;
    }

};

typedef boost::shared_ptr<FilterIsPositionAllowed> FilterIsPositionAllowedPtr;

}




