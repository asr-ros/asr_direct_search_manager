/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "model/robot_state.hpp"

namespace directSearchWS {


RobotState::RobotState(PosePtr robotPosePtr, PtuTuplePtrVecPtr ptuTuplePtrVecPtr):
    robotPosePtr(robotPosePtr), ptuTuplePtrVecPtr(ptuTuplePtrVecPtr){
}

//Getter
PosePtr RobotState::getRobotPosePtr() const {
    return robotPosePtr;
}

PtuTuplePtr RobotState::getTopPtuTuplePtr() const {
    return ptuTuplePtrVecPtr->front();
}

PtuTuplePtrVecPtr RobotState::getPtuTuplePtrVecPtr() const {
    return ptuTuplePtrVecPtr;
}

void RobotState::eraseFrontPTUTuplePtr() {
    ptuTuplePtrVecPtr->erase(ptuTuplePtrVecPtr->begin());
}

int RobotState::getPtuListSize() const {
    return ptuTuplePtrVecPtr->size();
}


std::ostream& operator<<(std::ostream &strm, const RobotState &robot_state) {
    strm << "RobotPose:\n" << robot_state.getRobotPosePtr();
    strm << "PtuTuples:\n" << robot_state.getPtuTuplePtrVecPtr();
    return strm;
}

std::ostream& operator<<(std::ostream &strm, const RobotStatePtr &robot_state_ptr) {
    return strm << *robot_state_ptr;
}

std::ostream& operator<<(std::ostream &strm, const RobotStatePtrVec &robot_state_ptr_vec) {
    for (const RobotStatePtr &robot_state_ptr : robot_state_ptr_vec) {
        strm << robot_state_ptr;
    }
    return strm;
}

std::ostream& operator<<(std::ostream &strm, const RobotStatePtrVecPtr &robot_state_ptr_vec_ptr) {
    return strm << *robot_state_ptr_vec_ptr;
}

}
