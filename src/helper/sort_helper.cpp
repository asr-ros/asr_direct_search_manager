/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "helper/sort_helper.hpp"

namespace directSearchWS {

SortHelper::SortHelper(RobotStatePtrVecPtr &posesToExplorePtr, bool needreorderPosesByTSP) :
    poseHelperPtr(PoseHelper::getInstance()), posesToExplorePtr(posesToExplorePtr), needreorderPosesByTSP(needreorderPosesByTSP) {
}

void SortHelper::nearest_neighbour_and_update_distance(double &distance) {
    if (!needreorderPosesByTSP) {
        return;
    }
    ROS_INFO("nearest_neighbour started");
    if (posesToExplorePtr->size() < 3) {
        ROS_DEBUG("There are less than 3 posesToExplore -> no optimization possible");
        return;
    }

    ros::Time begin = ros::Time::now();

    int numberOfSwaps = 0;
    for (RobotStatePtrVec::iterator mainRobotIt = posesToExplorePtr->begin(); mainRobotIt != posesToExplorePtr->end() - 2; ++mainRobotIt) {
        double currentBestDistance = DBL_MAX;
        RobotStatePtrVec::iterator currentBestRobotIt;
        for (RobotStatePtrVec::iterator robotIt = mainRobotIt + 1; robotIt != posesToExplorePtr->end(); ++robotIt) {
            double distance = poseHelperPtr->calculateDistance(*mainRobotIt->get()->getRobotPosePtr(), *robotIt->get()->getRobotPosePtr());
            if (distance < currentBestDistance) {
                currentBestDistance = distance;
                currentBestRobotIt = robotIt;
            }
        }
        std::iter_swap(mainRobotIt + 1, currentBestRobotIt);
        distance += currentBestDistance;
        ++numberOfSwaps;
    }
    distance += poseHelperPtr->calculateDistance(*(posesToExplorePtr->end() - 2)->get()->getRobotPosePtr(),
                                                 *(posesToExplorePtr->end() - 1)->get()->getRobotPosePtr());
    ROS_INFO_STREAM("nearest_neighbour finished. It took: " << (ros::Time::now() - begin).toSec() << "s, with numberOfSwaps: " << numberOfSwaps);
}

void SortHelper::two_opt_and_update_distance(double &distance) {
    if (!needreorderPosesByTSP) {
        return;
    }
    ROS_INFO("two_opt started");
    if (posesToExplorePtr->size() < 4) {
        ROS_DEBUG("There are less than 4 posesToExplore -> no optimization possible");
        return;
    }
    ROS_INFO_STREAM("distance before two_opt: " << distance);
    ros::Time begin = ros::Time::now();

    int numberOfSwaps = 0;
    bool improve;
    do {
        improve = false;
        for (RobotStatePtrVec::iterator robotIt1 = posesToExplorePtr->begin(); robotIt1 != posesToExplorePtr->end() - 3; ++robotIt1) {
            double bestChange = 0;
            RobotStatePtrVec::iterator swap1;
            RobotStatePtrVec::iterator swap2;
            for (RobotStatePtrVec::iterator robotIt2 = robotIt1 + 2; robotIt2 != posesToExplorePtr->end() - 1; ++robotIt2) {
                double change = poseHelperPtr->calculateDistance(*robotIt1->get()->getRobotPosePtr(), *robotIt2->get()->getRobotPosePtr())
                        + poseHelperPtr->calculateDistance(*(robotIt1 + 1)->get()->getRobotPosePtr(), *(robotIt2 + 1)->get()->getRobotPosePtr())
                        - poseHelperPtr->calculateDistance(*robotIt1->get()->getRobotPosePtr(), *(robotIt1 + 1)->get()->getRobotPosePtr())
                        - poseHelperPtr->calculateDistance(*robotIt2->get()->getRobotPosePtr(), *(robotIt2 + 1)->get()->getRobotPosePtr());
                if (change < bestChange) {
                    bestChange = change;
                    swap1 = robotIt1 + 1;
                    swap2 = robotIt2;
                    improve = true;
                }
            }
            if (bestChange < 0) {
                distance += bestChange;
                // Reverse [begin, end)
                std::reverse(swap1, (swap2 + 1));
                ++numberOfSwaps;
            }
        }
    } while (improve);
    ROS_INFO_STREAM("two_opt finished. It took: " << (ros::Time::now() - begin).toSec() << "s, with numberOfSwaps: " << numberOfSwaps);
    ROS_INFO_STREAM("distance after two_opt: " << distance);
}

}




