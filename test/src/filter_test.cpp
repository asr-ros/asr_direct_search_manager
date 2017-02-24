/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include "filter/do_filter_iteration.hpp"
#include "filter/filter_basic.hpp"

using namespace directSearchWS;


class MyFilter1 : public FilterBasic {
public:
    int deleteCount;

    MyFilter1(const RobotStatePtrVecPtr &posesToExplorePtr) :
        FilterBasic(posesToExplorePtr), deleteCount(0) {
    }
    virtual ~MyFilter1() { }

    bool needIteration() {
        deleteCount = 0;
        return true;
    }

    bool wasIterationSuccessful() {
        return true;
    }

    bool shouldPtuTupleBeDeleted(const RobotStatePtrVec::iterator &posesToExploreIter, const PtuTuplePtrVec::iterator &ptuTuplePtrIter) {
        bool shouldDelete = ptuTuplePtrIter->get()->getDeactivatedObjectNormalsCount() == 1;
        if (shouldDelete) {
            ++deleteCount;
        }
        return shouldDelete;
    }
};

typedef boost::shared_ptr<MyFilter1> MyFilter1Ptr;

class MyFilter2 : public FilterBasic {
public:
    int deleteCount;

    MyFilter2(const RobotStatePtrVecPtr &posesToExplorePtr) :
        FilterBasic(posesToExplorePtr), deleteCount(0) {
    }
    virtual ~MyFilter2() { }

    bool needIteration() {
        deleteCount = 0;
        return true;
    }

    bool wasIterationSuccessful() {
        return true;
    }

    bool shouldPtuTupleBeDeleted(const RobotStatePtrVec::iterator &posesToExploreIter, const PtuTuplePtrVec::iterator &ptuTuplePtrIter) {
        bool shouldDelete = ptuTuplePtrIter->get()->getDeactivatedObjectNormalsCount() >= 1;
        if (shouldDelete) {
            ++deleteCount;
        }
        return shouldDelete;
    }
};

typedef boost::shared_ptr<MyFilter2> MyFilter2Ptr;

class MyFilter3 : public FilterBasic {
public:
    int deleteCount;

    MyFilter3(const RobotStatePtrVecPtr &posesToExplorePtr) :
        FilterBasic(posesToExplorePtr), deleteCount(0) {
    }
    virtual ~MyFilter3() { }

    bool needIteration() {
        deleteCount = 0;
        return false;
    }

    bool wasIterationSuccessful() {
        return true;
    }

    bool shouldPtuTupleBeDeleted(const RobotStatePtrVec::iterator &posesToExploreIter, const PtuTuplePtrVec::iterator &ptuTuplePtrIter) {
        ++deleteCount;
        return true;
    }
};

typedef boost::shared_ptr<MyFilter3> MyFilter3Ptr;

RobotStatePtrVecPtr getRobotStates() {
    RobotStatePtrVecPtr robotStates = RobotStatePtrVecPtr(new RobotStatePtrVec());

    PtuTuplePtrVecPtr ptuTuples1 = PtuTuplePtrVecPtr(new PtuTuplePtrVec());
    ptuTuples1->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 1)));
    ptuTuples1->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 1)));
    robotStates->push_back(RobotStatePtr(new RobotState(nullptr, ptuTuples1)));

    PtuTuplePtrVecPtr ptuTuples2 = PtuTuplePtrVecPtr(new PtuTuplePtrVec());
    ptuTuples2->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 0)));
    ptuTuples2->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 2)));
    ptuTuples2->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 0)));
    ptuTuples2->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 1)));
    robotStates->push_back(RobotStatePtr(new RobotState(nullptr, ptuTuples2)));

    PtuTuplePtrVecPtr ptuTuples3 = PtuTuplePtrVecPtr(new PtuTuplePtrVec());
    ptuTuples3->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 1)));
    ptuTuples3->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 2)));
    ptuTuples3->push_back(PtuTuplePtr(new PtuTuple(0, 0, nullptr, 1)));
    robotStates->push_back(RobotStatePtr(new RobotState(nullptr, ptuTuples3)));

    return robotStates;
}


TEST(DoFilterIterationTest, NoFilter) {
    RobotStatePtrVecPtr robotStates = getRobotStates();
    DoFilterIteration filterIteration(robotStates);

    EXPECT_TRUE(filterIteration.doIteration());
    ASSERT_EQ(3, robotStates->size());
    for (int robotCount = 0; robotCount < robotStates->size(); ++robotCount) {
        RobotStatePtr &robotStatePtr = (*robotStates)[robotCount];
        if (robotCount == 0) {
            ASSERT_EQ(2, robotStatePtr->getPtuListSize());
        } else if (robotCount == 1) {
            ASSERT_EQ(4, robotStatePtr->getPtuListSize());
        } else {
            ASSERT_EQ(3, robotStatePtr->getPtuListSize());
        }
        for (int ptuCount = 0; ptuCount < robotStatePtr->getPtuListSize(); ++ptuCount) {
            PtuTuplePtr &ptuTuplePtr = (*robotStatePtr->getPtuTuplePtrVecPtr())[ptuCount];
            if (robotCount == 0) {
                EXPECT_EQ(1, ptuTuplePtr->getDeactivatedObjectNormalsCount());
            } else if (robotCount == 1) {
                if (ptuCount == 0 || ptuCount == 2) {
                    EXPECT_EQ(0, ptuTuplePtr->getDeactivatedObjectNormalsCount());
                } else if (ptuCount == 1) {
                    EXPECT_EQ(2, ptuTuplePtr->getDeactivatedObjectNormalsCount());
                } else  {
                    EXPECT_EQ(1, ptuTuplePtr->getDeactivatedObjectNormalsCount());
                }
            } else {
                if (ptuCount == 0 || ptuCount == 2) {
                    EXPECT_EQ(1, ptuTuplePtr->getDeactivatedObjectNormalsCount());
                } else  {
                    EXPECT_EQ(2, ptuTuplePtr->getDeactivatedObjectNormalsCount());
                }
            }
        }
    }
}

TEST(DoFilterIterationTest, OneFilter) {
    RobotStatePtrVecPtr robotStates = getRobotStates();
    DoFilterIteration filterIteration(robotStates);

    MyFilter1Ptr filter1 = MyFilter1Ptr(new MyFilter1(robotStates));
    filterIteration.addFilter(filter1);

    EXPECT_TRUE(filterIteration.doIteration());
    ASSERT_EQ(2, robotStates->size());
    EXPECT_EQ(5, filter1->deleteCount);

    for (int robotCount = 0; robotCount < robotStates->size(); ++robotCount) {
        RobotStatePtr &robotStatePtr = (*robotStates)[robotCount];
        if (robotCount == 0) {
            ASSERT_EQ(3, robotStatePtr->getPtuListSize());
        } else {
            ASSERT_EQ(1, robotStatePtr->getPtuListSize());
        }
        for (int ptuCount = 0; ptuCount < robotStatePtr->getPtuListSize(); ++ptuCount) {
            PtuTuplePtr &ptuTuplePtr = (*robotStatePtr->getPtuTuplePtrVecPtr())[ptuCount];
            if (robotCount == 0) {
                if (ptuCount == 0 || ptuCount == 2) {
                    EXPECT_EQ(0, ptuTuplePtr->getDeactivatedObjectNormalsCount());
                } else {
                    EXPECT_EQ(2, ptuTuplePtr->getDeactivatedObjectNormalsCount());
                }
            } else {
                EXPECT_EQ(2, ptuTuplePtr->getDeactivatedObjectNormalsCount());
            }
        }
    }
}

TEST(DoFilterIterationTest, MultipleFilters) {
    RobotStatePtrVecPtr robotStates = getRobotStates();
    DoFilterIteration filterIteration(robotStates);

    MyFilter1Ptr filter1 = MyFilter1Ptr(new MyFilter1(robotStates));
    filterIteration.addFilter(filter1);
    MyFilter2Ptr filter2 = MyFilter2Ptr(new MyFilter2(robotStates));
    filterIteration.addFilter(filter2);

    EXPECT_TRUE(filterIteration.doIteration());
    ASSERT_EQ(1, robotStates->size());
    EXPECT_EQ(5, filter1->deleteCount);
    EXPECT_EQ(2, filter2->deleteCount);

    RobotStatePtr &robotStatePtr = (*robotStates)[0];
    ASSERT_EQ(2, robotStatePtr->getPtuListSize());
    for (int ptuCount = 0; ptuCount < robotStatePtr->getPtuListSize(); ++ptuCount) {
        PtuTuplePtr &ptuTuplePtr = (*robotStatePtr->getPtuTuplePtrVecPtr())[ptuCount];
        EXPECT_EQ(0, ptuTuplePtr->getDeactivatedObjectNormalsCount());
    }


    robotStates = getRobotStates();
    MyFilter3Ptr filter3 = MyFilter3Ptr(new MyFilter3(robotStates));
    filterIteration.addFilter(filter3);

    EXPECT_TRUE(filterIteration.doIteration());
    ASSERT_EQ(1, robotStates->size());
    EXPECT_EQ(5, filter1->deleteCount);
    EXPECT_EQ(2, filter2->deleteCount);
    EXPECT_EQ(0, filter3->deleteCount);

    robotStatePtr = (*robotStates)[0];
    ASSERT_EQ(2, robotStatePtr->getPtuListSize());
    for (int ptuCount = 0; ptuCount < robotStatePtr->getPtuListSize(); ++ptuCount) {
        PtuTuplePtr &ptuTuplePtr = (*robotStatePtr->getPtuTuplePtrVecPtr())[ptuCount];
        EXPECT_EQ(0, ptuTuplePtr->getDeactivatedObjectNormalsCount());
    }
}

TEST(DoFilterIterationTest, NoPoses) {
    RobotStatePtrVecPtr robotStates = RobotStatePtrVecPtr(new RobotStatePtrVec());
    DoFilterIteration filterIteration(robotStates);

    MyFilter1Ptr filter1 = MyFilter1Ptr(new MyFilter1(robotStates));
    filterIteration.addFilter(filter1);
    MyFilter2Ptr filter2 = MyFilter2Ptr(new MyFilter2(robotStates));
    filterIteration.addFilter(filter2);

    EXPECT_TRUE(filterIteration.doIteration());
    ASSERT_EQ(0, robotStates->size());
    EXPECT_EQ(0, filter1->deleteCount);
    EXPECT_EQ(0, filter2->deleteCount);
}






