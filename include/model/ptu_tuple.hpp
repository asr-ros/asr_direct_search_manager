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

#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <asr_next_best_view/RatedViewport.h>

namespace directSearchWS {

typedef std::vector<std::string> SearchedObjectTypes;
typedef boost::shared_ptr<geometry_msgs::Pose> PosePtr;

class PtuTuple {

private:
    int pan;
    int tilt;
    PosePtr cameraPosePtr;
    int deactivated_object_normals_count;
    SearchedObjectTypes already_searched_object_types;
    asr_next_best_view::RatedViewport ratedViewport;

public:
    PtuTuple(int pan, int tilt, PosePtr cameraPosePtr);
    PtuTuple(int pan, int tilt, PosePtr cameraPosePtr, int deactivated_object_normals_count);
    PtuTuple(int pan, int tilt, PosePtr cameraPosePtr, int deactivated_object_normals_count, SearchedObjectTypes already_searched_object_types);
    PtuTuple(int pan, int tilt, PosePtr cameraPosePtr, int deactivated_object_normals_count, SearchedObjectTypes already_searched_object_types, asr_next_best_view::RatedViewport ratedViewport);

    //Getter
    int getPan() const;

    int getTilt() const;

    SearchedObjectTypes getAlreadySearchedObjectTypes() const;

    void addAlreadySearchedObjectType(std::string newAlreadySearchedObjectType);

    void addAlreadySearchedObjectTypes(const SearchedObjectTypes &newAlreadySearchedObjectTypes);

    void resetAlreadySearchedObjectTypes();

    PosePtr getCameraPosePtr() const;

    int getDeactivatedObjectNormalsCount() const;

    void setRatedViewport(const asr_next_best_view::RatedViewport &newRatedViewport);

    asr_next_best_view::RatedViewport getRatedViewport() const;

};

typedef boost::shared_ptr<PtuTuple> PtuTuplePtr;
typedef std::vector<PtuTuplePtr> PtuTuplePtrVec;
typedef boost::shared_ptr<PtuTuplePtrVec> PtuTuplePtrVecPtr;

std::ostream& operator<<(std::ostream &strm, const SearchedObjectTypes &already_searched_object_types);
std::ostream& operator<<(std::ostream &strm, const PosePtr &pose_ptr);

std::ostream& operator<<(std::ostream &strm, const PtuTuple &ptu_tuple);
std::ostream& operator<<(std::ostream &strm, const PtuTuplePtr &ptu_tuple_ptr);
std::ostream& operator<<(std::ostream &strm, const PtuTuplePtrVec &ptu_tuple_ptr_vec);
std::ostream& operator<<(std::ostream &strm, const PtuTuplePtrVecPtr &ptu_tuple_ptr_vec_ptr);

}
