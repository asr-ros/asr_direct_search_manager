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
#include "direct_search_handler.hpp"
#include "helper/pose_helper.hpp"

namespace directSearchWS {

class ConcatApproxEqualePoses : public FilterBasic {

private:
    int concatCount;
    int deleteCount;
    double positionDistanceThreshold;
    double orientationRadDistanceThreshold;
    bool checkOnlyNext;
    bool shouldDelete;
    bool shouldConcat;

    PoseHelperPtr poseHelperPtr;


public:
    ConcatApproxEqualePoses(const RobotStatePtrVecPtr &posesToExplorePtr, double positionDistanceThreshold, double orientationRadDistanceThreshold,
                            bool checkOnlyNext = false, bool shouldDelete = true, bool shouldConcat = true) :
        FilterBasic(posesToExplorePtr), concatCount(0), deleteCount(0), positionDistanceThreshold(positionDistanceThreshold),
        orientationRadDistanceThreshold(orientationRadDistanceThreshold), checkOnlyNext(checkOnlyNext), shouldDelete(shouldDelete), shouldConcat(shouldConcat),
        poseHelperPtr(PoseHelper::getInstance()) {

    }
    virtual ~ConcatApproxEqualePoses() { }

    bool needIteration() {
        concatCount = 0;
        deleteCount = 0;
        return true;
    }

    bool wasIterationSuccessful() {
        if (shouldConcat) {
            ROS_INFO_STREAM("Number of concatenated ptu_tuple by ConcatApproxEqualePoses: " << concatCount << " (number is not representative)");
        }
        if (shouldDelete) {
            ROS_INFO_STREAM("Number of deleted ptu_tuple by ConcatApproxEqualePoses: " << deleteCount);
        }
        return true;
    }

    bool shouldPtuTupleBeDeleted(const RobotStatePtrVec::iterator &posesToExploreIter, const PtuTuplePtrVec::iterator &ptuTuplePtrIter) {
        RobotStatePtrVec::iterator robotEnd = posesToExplorePtr->end();
        if (checkOnlyNext && posesToExploreIter + 1 != posesToExplorePtr->end()) {
            robotEnd = posesToExploreIter + 2;
        }

        // Check if robot_poses are approx_equale and concate to one robot_state with multiple ptuTuples
        for (RobotStatePtrVec::iterator posesToExploreCompareIter = posesToExploreIter + 1; posesToExploreCompareIter != robotEnd; ++posesToExploreCompareIter) {
            // If poses are approx_equale push current ptu_tuple in other pose and delete/concat it
            if (poseHelperPtr->checkPosesAreApproxEquale(*posesToExploreIter->get()->getRobotPosePtr(),
                                                         *posesToExploreCompareIter->get()->getRobotPosePtr(), positionDistanceThreshold, orientationRadDistanceThreshold)) {
                // Only push back if the PTU-tuple does not exists in the other part
                bool isEquale = false;
                PtuTuplePtrVec::iterator ptuTuplePtrCompareIter = posesToExploreCompareIter->get()->getPtuTuplePtrVecPtr()->begin();
                for (;ptuTuplePtrCompareIter != posesToExploreCompareIter->get()->getPtuTuplePtrVecPtr()->end(); ++ptuTuplePtrCompareIter) {
                    if (ptuTuplePtrCompareIter->get()->getPan() == ptuTuplePtrIter->get()->getPan()
                            && ptuTuplePtrCompareIter->get()->getTilt() == ptuTuplePtrIter->get()->getTilt()) {
                        //ROS_DEBUG("Two poses are approx_- and ptu_configs are equale -> one ptu_config will be removed");
                        isEquale = true;
                        break;
                    }
                }
                if (isEquale) {
                    if (shouldDelete) {
                        ++deleteCount;
                        SearchedObjectTypes firstSearchObjectTypes = ptuTuplePtrCompareIter->get()->getAlreadySearchedObjectTypes();
                        SearchedObjectTypes secondSearchObjectTypes = ptuTuplePtrIter->get()->getAlreadySearchedObjectTypes();
                        ptuTuplePtrCompareIter->get()->resetAlreadySearchedObjectTypes();
                        SearchedObjectTypes intersectionSearchObjectTypes = getIntersectionOfSearchObjectTypes(firstSearchObjectTypes, secondSearchObjectTypes);
                        ptuTuplePtrCompareIter->get()->addAlreadySearchedObjectTypes(intersectionSearchObjectTypes);
                    } else {
                        continue;
                    }
                } else {
                    if (shouldConcat) {
                        posesToExploreCompareIter->get()->getPtuTuplePtrVecPtr()->insert(posesToExploreCompareIter->get()->getPtuTuplePtrVecPtr()->end() - 1, *ptuTuplePtrIter);
                        ++concatCount;
                    } else {
                        continue;
                    }
                }

                return true;
            }
        }
        return false;
    }

};

typedef boost::shared_ptr<ConcatApproxEqualePoses> ConcatApproxEqualePosesPtr;

}




