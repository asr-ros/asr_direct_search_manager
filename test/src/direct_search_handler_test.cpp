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
#include "direct_search_handler.hpp"
#include "asr_msgs/AsrTypeAndId.h"

using namespace directSearchWS;


TEST(DirectSearchHandlerTest, FilterSearchedObjectTypes) {
    SearchedObjectTypes SOTToFilter;
    SearchedObjectTypes filterSOT;

    filterSOT.push_back("PlateDeep");
    filterSOT.push_back("Marker0");
    filterSOT.push_back("Marker1");
    filterSOT.push_back("Marker1");


    filterSearchedObjectTypes(SOTToFilter, filterSOT);
    ASSERT_EQ(0, SOTToFilter.size());

    SOTToFilter.push_back("PlateDeep");
    SOTToFilter.push_back("Marker1");
    SOTToFilter.push_back("PlateDeep");
    filterSearchedObjectTypes(SOTToFilter, filterSOT);
    ASSERT_EQ(0, SOTToFilter.size());

    SOTToFilter.push_back("Marker1");
    SOTToFilter.push_back("PlateDeep");
    SOTToFilter.push_back("Marker10");
    SOTToFilter.push_back("Marker1");
    SOTToFilter.push_back("Marker1");
    SOTToFilter.push_back("PlateDeep");
    filterSearchedObjectTypes(SOTToFilter, filterSOT);
    ASSERT_EQ(1, SOTToFilter.size());
    ASSERT_EQ("Marker10", SOTToFilter[0]);
}

TEST(DirectSearchHandlerTest, filterSearchedObjectTypesAndIds) {
    SearchedObjectTypesAndIds SOTToFilter;
    SearchedObjectTypes filterSOT;

    filterSOT.push_back("PlateDeep");
    filterSOT.push_back("Marker0");
    filterSOT.push_back("Marker1");
    filterSOT.push_back("Marker1");


    filterSearchedObjectTypesAndIds(SOTToFilter, filterSOT);
    ASSERT_EQ(0, SOTToFilter.size());

    asr_msgs::AsrTypeAndId typeAndId1;
    typeAndId1.type = "PlateDeep";
    typeAndId1.identifier = "2";
    SOTToFilter.push_back(typeAndId1);
    asr_msgs::AsrTypeAndId typeAndId2;
    typeAndId2.type = "Marker1";
    typeAndId2.identifier = "3";
    SOTToFilter.push_back(typeAndId2);
    asr_msgs::AsrTypeAndId typeAndId3;
    typeAndId3.type = "PlateDeep";
    typeAndId3.identifier = "2";
    SOTToFilter.push_back(typeAndId3);
    filterSearchedObjectTypesAndIds(SOTToFilter, filterSOT);
    ASSERT_EQ(0, SOTToFilter.size());

    asr_msgs::AsrTypeAndId typeAndId4;
    typeAndId4.type = "Marker1";
    typeAndId4.identifier = "3";
    SOTToFilter.push_back(typeAndId4);
    asr_msgs::AsrTypeAndId typeAndId5;
    typeAndId5.type = "PlateDeep";
    typeAndId5.identifier = "4";
    SOTToFilter.push_back(typeAndId5);
    asr_msgs::AsrTypeAndId typeAndId6;
    typeAndId6.type = "Marker10";
    typeAndId6.identifier = "4";
    SOTToFilter.push_back(typeAndId6);
    asr_msgs::AsrTypeAndId typeAndId7;
    typeAndId7.type = "Marker1";
    typeAndId7.identifier = "2";
    SOTToFilter.push_back(typeAndId7);
    asr_msgs::AsrTypeAndId typeAndId8;
    typeAndId8.type = "Marker1";
    typeAndId8.identifier = "3";
    SOTToFilter.push_back(typeAndId8);
    asr_msgs::AsrTypeAndId typeAndId9;
    typeAndId9.type = "PlateDeep";
    typeAndId9.identifier = "3";
    SOTToFilter.push_back(typeAndId9);
    filterSearchedObjectTypesAndIds(SOTToFilter, filterSOT);
    ASSERT_EQ(1, SOTToFilter.size());
    ASSERT_EQ("Marker10", SOTToFilter[0].type);
    ASSERT_EQ("4", SOTToFilter[0].identifier);
}

TEST(DirectSearchHandlerTest, GetIntersectionOfSearchObjectTypes) {
    SearchedObjectTypes SOT1;
    SearchedObjectTypes SOT2;
    SearchedObjectTypes intersectionSOT;

    intersectionSOT = getIntersectionOfSearchObjectTypes(SOT1, SOT2);
    ASSERT_EQ(0, intersectionSOT.size());

    SOT1.push_back("PlateDeep");
    SOT1.push_back("Marker0");
    SOT1.push_back("Marker10");

    SOT2.push_back("Marker2");
    SOT2.push_back("Marker3");
    SOT2.push_back("Marker4");

    intersectionSOT = getIntersectionOfSearchObjectTypes(SOT1, SOT2);
    ASSERT_EQ(0, intersectionSOT.size());


    SOT2.push_back("Marker0");
    SOT2.push_back("Marker3");
    SOT2.push_back("Marker0");

    intersectionSOT = getIntersectionOfSearchObjectTypes(SOT1, SOT2);
    ASSERT_EQ(1, intersectionSOT.size());
    ASSERT_EQ("Marker0", intersectionSOT[0]);


    SOT1.push_back("Marker0");
    intersectionSOT = getIntersectionOfSearchObjectTypes(SOT1, SOT2);
    ASSERT_EQ(2, intersectionSOT.size());
    ASSERT_EQ("Marker0", intersectionSOT[0]);
    ASSERT_EQ("Marker0", intersectionSOT[1]);


    SOT1.push_back("Marker3");
    intersectionSOT = getIntersectionOfSearchObjectTypes(SOT1, SOT2);
    ASSERT_EQ(3, intersectionSOT.size());
    ASSERT_NE(intersectionSOT.end(), std::find(intersectionSOT.begin(), intersectionSOT.end(), "Marker0"));
    ASSERT_NE(intersectionSOT.end(), std::find(intersectionSOT.begin(), intersectionSOT.end(), "Marker3"));
}



