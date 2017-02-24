/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <algorithm>

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "direct_search_manager.hpp"

using namespace directSearchWS;


TEST(DirectSearchManagerTest, CheckSearchedObjectTypesAreEquale) {
    SearchedObjectTypes SOT1;
    SearchedObjectTypes SOT2;

    ASSERT_TRUE(checkSearchedObjectTypesAreEquale(SOT1, SOT2));

    SOT1.push_back("PlateDeep");
    ASSERT_FALSE(checkSearchedObjectTypesAreEquale(SOT1, SOT2));

    SOT2.push_back("Marker4");
    ASSERT_FALSE(checkSearchedObjectTypesAreEquale(SOT1, SOT2));

    SOT2.push_back("PlateDeep");
    ASSERT_FALSE(checkSearchedObjectTypesAreEquale(SOT1, SOT2));

    SOT1.push_back("Marker4");
    ASSERT_TRUE(checkSearchedObjectTypesAreEquale(SOT1, SOT2));

    SOT1.push_back("Marker4");
    ASSERT_FALSE(checkSearchedObjectTypesAreEquale(SOT1, SOT2));

    SOT2.push_back("Marker4");
    ASSERT_TRUE(checkSearchedObjectTypesAreEquale(SOT1, SOT2));
}

TEST(DirectSearchManagerTest, GetSearchedObjectTypesFromAWithoutB) {
    SearchedObjectTypes A;
    SearchedObjectTypes B;
    SearchedObjectTypes AwithoutB;

    AwithoutB = getSearchedObjectTypesFromAWithoutB(A, B);
    EXPECT_EQ(0, AwithoutB.size());

    A.push_back("PlateDeep");
    B.push_back("Marker0");
    AwithoutB = getSearchedObjectTypesFromAWithoutB(A, B);
    ASSERT_EQ(1, AwithoutB.size());
    EXPECT_EQ("PlateDeep", AwithoutB[0]);

    B.push_back("PlateDeep");
    AwithoutB = getSearchedObjectTypesFromAWithoutB(A, B);
    ASSERT_EQ(0, AwithoutB.size());

    A.push_back("PlateDeep");
    AwithoutB = getSearchedObjectTypesFromAWithoutB(A, B);
    ASSERT_EQ(0, AwithoutB.size());

    B.push_back("Marker0");
    AwithoutB = getSearchedObjectTypesFromAWithoutB(A, B);
    ASSERT_EQ(0, AwithoutB.size());
}



