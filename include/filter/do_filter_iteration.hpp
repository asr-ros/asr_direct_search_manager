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

#include <boost/shared_ptr.hpp>

#include "model/robot_state.hpp"
#include "model/ptu_tuple.hpp"

#include "filter/filter_basic.hpp"


namespace directSearchWS {

class DoFilterIteration {

private:
    std::vector<FilterBasicPtr> filtersToApply;
    const RobotStatePtrVecPtr &posesToExplorePtr;

public:
    DoFilterIteration(const RobotStatePtrVecPtr &posesToExplorePtr) : filtersToApply(), posesToExplorePtr(posesToExplorePtr) { }

    void addFilter(const FilterBasicPtr &filter) {
        filtersToApply.push_back(filter);
    }

    bool doIteration() {
        std::vector<FilterBasicPtr> filteredFiltersToApply;
        for (const FilterBasicPtr &filter_ptr : filtersToApply) {
            if (filter_ptr->needIteration()) {
                filteredFiltersToApply.push_back(filter_ptr);
            }
        }

        // Iterate over all poses and ptu_tuples and apply filters
        RobotStatePtrVec::iterator posesToExploreIter = posesToExplorePtr->begin();
        while (posesToExploreIter != posesToExplorePtr->end()) {
            PtuTuplePtrVec::iterator ptuTuplePtrIter = posesToExploreIter->get()->getPtuTuplePtrVecPtr()->begin();
            while (ptuTuplePtrIter != posesToExploreIter->get()->getPtuTuplePtrVecPtr()->end()) {
                bool shouldDelete = false;
                for (const FilterBasicPtr &filter_ptr : filteredFiltersToApply) {
                    shouldDelete |= filter_ptr->shouldPtuTupleBeDeleted(posesToExploreIter, ptuTuplePtrIter);
                    if (shouldDelete) {
                        break;
                    }
                }

                if (shouldDelete) {
                    ptuTuplePtrIter = posesToExploreIter->get()->getPtuTuplePtrVecPtr()->erase(ptuTuplePtrIter);
                } else {
                    ++ptuTuplePtrIter;
                }
            }
            if (posesToExploreIter->get()->getPtuListSize() == 0) {
                posesToExploreIter = posesToExplorePtr->erase(posesToExploreIter);
            } else {
                ++posesToExploreIter;
            }
        }

        bool wasSuccessful = true;
        for (const FilterBasicPtr &filter_ptr : filteredFiltersToApply) {
            wasSuccessful &= filter_ptr->wasIterationSuccessful();
        }
        return wasSuccessful;
    }

};

typedef boost::shared_ptr<DoFilterIteration> DoFilterIterationPtr;

}



