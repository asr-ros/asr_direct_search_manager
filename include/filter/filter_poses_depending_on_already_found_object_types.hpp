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

namespace directSearchWS {

class FilterPosesDependingOnAlreadyFoundObjectTypes : public FilterBasic {

private:
    int deleteCount;
    SearchedObjectTypes searchedObjectTypes;
    bool isNeedReorder;

public:
    FilterPosesDependingOnAlreadyFoundObjectTypes(const RobotStatePtrVecPtr &posesToExplorePtr) :
        FilterBasic(posesToExplorePtr), deleteCount(0), searchedObjectTypes(), isNeedReorder(false) { }
    virtual ~FilterPosesDependingOnAlreadyFoundObjectTypes() { }

    bool needIteration() {
        deleteCount = 0;
        isNeedReorder = false;
        return true;
    }

    bool wasIterationSuccessful() {
        ROS_INFO_STREAM("Number of deleted ptuTuple by FilterPosesDependingOnAlreadyFoundObjectTypes: " << deleteCount);
        return true;
    }

    bool shouldPtuTupleBeDeleted(const RobotStatePtrVec::iterator &posesToExploreIter, const PtuTuplePtrVec::iterator &ptuTuplePtrIter) {
        SearchedObjectTypes copySearchedObjectTypes(searchedObjectTypes);
        filterSearchedObjectTypes(copySearchedObjectTypes, ptuTuplePtrIter->get()->getAlreadySearchedObjectTypes());
        // Check if every searchedObjectTypes is present in alreadySearchedObjectTypes
        bool shouldDelete = copySearchedObjectTypes.empty();
        if (shouldDelete) {
            ++deleteCount;
            isNeedReorder = true;
        }
        return shouldDelete;
    }

    void updateSearchedObjectTypes(const SearchedObjectTypes &newSearchedObjectTypes) {
        searchedObjectTypes = newSearchedObjectTypes;
    }

    bool needReorder() const {
        return isNeedReorder;
    }

};

typedef boost::shared_ptr<FilterPosesDependingOnAlreadyFoundObjectTypes> FilterPosesDependingOnAlreadyFoundObjectTypesPtr;

}




