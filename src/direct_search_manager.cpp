/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "direct_search_manager.hpp"

namespace directSearchWS {

DirectSearchManager::DirectSearchManager(): DirectSearchHandler(), currentPan(0.0), currentTilt(0.0), needSortByNBV(true), lastSearchedObjectTypes() {
    ros::NodeHandle n(ros::this_node::getName());
    getViewportListServiceClient = n.serviceClient<world_model::GetViewportList>("/env/world_model/get_viewport_list");

    ptuDriverStateSubscriber = n.subscribe("/asr_flir_ptu_driver/state", 1000, &DirectSearchManager::ptuDriverStateCallback, this);
    setInitRobotStateServiceClient = n.serviceClient<next_best_view::SetInitRobotState>("/nbv/set_init_robot_state");
    setAttributedPointCloudServiceClient = n.serviceClient<next_best_view::SetAttributedPointCloud>("/nbv/set_point_cloud");
    rateViewportsServiceClient = n.serviceClient<next_best_view::RateViewports>("/nbv/rate_viewports");

    n.getParam("reorderPosesByNBV", reorderPosesByNBVParam);
    ROS_INFO_STREAM("Param: reorderPosesByNBV: " << reorderPosesByNBVParam);

    n.getParam("dbfilename", dbfilenameParam);
    ROS_INFO_STREAM("Param: dbfilename: " << dbfilenameParam);

    filterPosesDependingOnAlreadyFoundObjectTypesPtr = FilterPosesDependingOnAlreadyFoundObjectTypesPtr(
                new FilterPosesDependingOnAlreadyFoundObjectTypes(posesToExplorePtr));
    doFilterIterationPtrForPosesDependingOnAlreadyFoundObjectTypes = DoFilterIterationPtr(new DoFilterIteration(posesToExplorePtr));
    doFilterIterationPtrForPosesDependingOnAlreadyFoundObjectTypes->addFilter(filterPosesDependingOnAlreadyFoundObjectTypesPtr);
}

DirectSearchManager::~DirectSearchManager()  { }

bool DirectSearchManager::parsePosesToExploreFromXML(const std::string &path) {
    //read config file.
    std::ifstream ConfigFile(path.c_str(),std::ifstream::in);
    if (!ConfigFile) {
        ROS_ERROR_STREAM("Unable to locate " << path << ". Please make sure the launch file is correct and start again");
        return false;
    }
    std::vector<char> buffer((std::istreambuf_iterator<char>(ConfigFile)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    // Parse the buffer using the xml file parsing library into doc
    rapidxml::xml_document<> doc;
    doc.parse<0>(&buffer[0]);

    // Find our root node
    rapidxml::xml_node<> *root_node = doc.first_node("root");
    for (rapidxml::xml_node<> *state_node = root_node->first_node("state"); state_node; state_node = state_node->next_sibling()) {
        const PosePtr &goal_camera_pose_ptr = parsePose(state_node, "goal_camera_pose");
        const PosePtr &goal_robot_pose_ptr = parsePose(state_node, "goal_robot_pose");

        rapidxml::xml_node<> *goal_ptu_position_node = state_node->first_node("goal_ptu_position");
        int goal_ptu_position_pan = boost::lexical_cast<int>((goal_ptu_position_node->first_attribute("pan")->value()));
        int goal_ptu_position_tilt = boost::lexical_cast<int>((goal_ptu_position_node->first_attribute("tilt")->value()));

        int deactivated_object_normals_count = 0;
        if (state_node->first_node("deactivated_object_normals_count") != nullptr) {
            deactivated_object_normals_count = boost::lexical_cast<int>((state_node->first_node("deactivated_object_normals_count")->first_attribute("count")->value()));
        }

        PtuTuplePtrVecPtr ptu_tuple_ptr_vecotr_ptr = PtuTuplePtrVecPtr(new PtuTuplePtrVec());
        ptu_tuple_ptr_vecotr_ptr->push_back(PtuTuplePtr(new PtuTuple(goal_ptu_position_pan, goal_ptu_position_tilt, goal_camera_pose_ptr, deactivated_object_normals_count)));

        posesToExplorePtr->push_back(RobotStatePtr(new RobotState(goal_robot_pose_ptr, ptu_tuple_ptr_vecotr_ptr)));
    }
    // ROS_DEBUG_STREAM("Parsed posesToExplore:\n" << posesToExplorePtr);
    ROS_DEBUG_STREAM("Number of parsed posesToExplore: " << posesToExplorePtr->size());
    return true;
}

PosePtr DirectSearchManager::parsePose(const rapidxml::xml_node<> *node, const char *poseName) const {
    PosePtr pose_ptr = PosePtr(new geometry_msgs::Pose());
    rapidxml::xml_node<> *pose_node = node->first_node(poseName);

    rapidxml::xml_node<> *position_node = pose_node->first_node("position");
    pose_ptr->position.x = boost::lexical_cast<double>((position_node->first_attribute("x")->value()));
    pose_ptr->position.y = boost::lexical_cast<double>((position_node->first_attribute("y")->value()));
    pose_ptr->position.z = boost::lexical_cast<double>((position_node->first_attribute("z")->value()));

    rapidxml::xml_node<> *orientation_node = pose_node->first_node("orientation");
    pose_ptr->orientation.x = boost::lexical_cast<double>((orientation_node->first_attribute("x")->value()));
    pose_ptr->orientation.y = boost::lexical_cast<double>((orientation_node->first_attribute("y")->value()));
    pose_ptr->orientation.z = boost::lexical_cast<double>((orientation_node->first_attribute("z")->value()));
    pose_ptr->orientation.w = boost::lexical_cast<double>((orientation_node->first_attribute("w")->value()));
    return pose_ptr;
}


bool DirectSearchManager::resetHandler() {
    bool success = DirectSearchHandler::resetHandler();
    ros::NodeHandle n(ros::this_node::getName());
    n.getParam("dbfilename", dbfilenameParam);
    ROS_INFO_STREAM("Param: dbfilename: " << dbfilenameParam);
    lastSearchedObjectTypes.clear();
    needSortByNBV = true;
    return success;
}

bool DirectSearchManager::getNextRobotState(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds) {
    SearchedObjectTypes searchedObjectTypes;
    for (const asr_msgs::AsrTypeAndId &searchedObjectType : searchedObjectTypesAndIds) {
        searchedObjectTypes.push_back(searchedObjectType.type);
    }
    if (checkSearchedObjectTypesAreEquale(lastSearchedObjectTypes, searchedObjectTypes) && !searchedObjectTypes.empty()) {
        ROS_INFO("Same searched object types -> nothing to update");
    } else {
        ROS_INFO("Different searched object types -> update");
        // Check if some poses are not needed anymore, because the searchedObjectTypes were already searched there
        filterPosesDependingOnAlreadyFoundObjectTypesPtr->updateSearchedObjectTypes(searchedObjectTypes);
        doFilterIterationPtrForPosesDependingOnAlreadyFoundObjectTypes->doIteration();

        if (reorderPosesByNBVParam && needSortByNBV) {
            if (!setInitialRobotState()) {
                ROS_ERROR("Service call to setInitialRobotState was not successful");
                return false;
            }

            if (!setPointCloudInNBV(searchedObjectTypesAndIds)) {
                ROS_ERROR("Service call to setPointCloudInNBV was not successful");
                return false;
            }
        }
    }
    lastSearchedObjectTypes = searchedObjectTypes;

    bool success = true;
    if (reorderPosesByNBVParam && needSortByNBV) {
        success &= reorderPosesByNBV(searchedObjectTypes);
        calculateRemainingPosesDistances();
    }

    if (reorderPosesByNBVParam && !posesToExplorePtr->empty()) {
        ROS_DEBUG_STREAM("nextRatedViewort:\n" <<posesToExplorePtr->front()->getTopPtuTuplePtr()->getRatedViewport());

        if (needSortByNBV && posesToExplorePtr->front()->getTopPtuTuplePtr()->getRatedViewport().rating == 0) {
            // reorder poses to current position, if there is no improve from nbv anymore
            success &= DirectSearchHandler::backToInitial(searchedObjectTypesAndIds);
            needSortByNBV = false;
        }
    }
    success &= DirectSearchHandler::getNextRobotState(searchedObjectTypesAndIds);
    return success;
}

bool DirectSearchManager::getViewportsFromWorldModel(std::vector<asr_msgs::AsrViewport> &viewports) {
    world_model::GetViewportList getViewportServiceCall;
    getViewportServiceCall.request.object_type = "all";
    bool success = getViewportListServiceClient.call(getViewportServiceCall);
    viewports = getViewportServiceCall.response.viewport_list;
    return success;
}

bool DirectSearchManager::backToInitial(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds) {
    needSortByNBV = true;
    std::vector<asr_msgs::AsrViewport> viewports;
    bool success = getViewportsFromWorldModel(viewports);
    ROS_INFO_STREAM("Got viewports from world_model: " << viewports.size());
    // Iterate over all viewports from world_model and update the already_found_object_types in the ptu_tuples
    for (const asr_msgs::AsrViewport &viewport : viewports) {
        for (const RobotStatePtr &robotStatePtr : *posesToExplorePtr) {
            for (const PtuTuplePtr &ptuTuplePtr : *robotStatePtr->getPtuTuplePtrVecPtr()) {
                if (poseHelperPtr->checkViewCenterPointsAreApproxEquale(viewport.pose, *ptuTuplePtr->getCameraPosePtr())) {
                    ptuTuplePtr->addAlreadySearchedObjectTypes(viewport.object_type_name_list);
                }
            }
        }
    }

    success &= DirectSearchHandler::backToInitial(searchedObjectTypesAndIds);

    if (reorderPosesByNBVParam) {
        if (!setInitialRobotState()) {
            ROS_ERROR("Service call to setInitialRobotState was not successful");
            return false;
        }

        if (!setPointCloudInNBV(searchedObjectTypesAndIds)) {
            ROS_ERROR("Service call to setPointCloudInNBV was not successful");
            return false;
        }

        SearchedObjectTypes searchedObjectTypes;
        for (const asr_msgs::AsrTypeAndId &searchedObjectType : searchedObjectTypesAndIds) {
            searchedObjectTypes.push_back(searchedObjectType.type);
        }
        success &= reorderPosesByNBV(searchedObjectTypes);
        calculateRemainingPosesDistances();

        if (!posesToExplorePtr->empty()) {
            arePosesFromDemonstrationLeft = posesToExplorePtr->front()->getTopPtuTuplePtr()->getRatedViewport().rating > 0;
        }
    } else {
        arePosesFromDemonstrationLeft = false;
    }

    return success;
}

bool DirectSearchManager::reorderPosesByNBV(const SearchedObjectTypes &searchedObjectTypes) {
    bool success = true;
    success &= deleteAllRedundantPoses(posesToExplorePtr);

    next_best_view::RateViewports rateViewports;
    rateViewports.request.current_pose = poseHelperPtr->getCurrentCameraPose();
    rateViewports.request.object_type_name_list = searchedObjectTypes;
    rateViewports.request.use_object_type_to_rate = false;

    RobotStatePtrVecPtr robotStatesToReorder = RobotStatePtrVecPtr(new RobotStatePtrVec());

    const int orginalNumberOfPoses = posesToExplorePtr->size();
    ROS_DEBUG_STREAM("Initial number of poses to explore: " << orginalNumberOfPoses);

    for (const RobotStatePtr &robotStatePtr : *posesToExplorePtr) {
        for (const PtuTuplePtr &ptuTuplePtr : *robotStatePtr->getPtuTuplePtrVecPtr()) {
            rateViewports.request.viewports.push_back(*ptuTuplePtr->getCameraPosePtr());

            PtuTuplePtrVecPtr tempPtuTuplePtrVecPtr = PtuTuplePtrVecPtr(new PtuTuplePtrVec());
            // do not copy ratedViewport
            tempPtuTuplePtrVecPtr->push_back(PtuTuplePtr(new PtuTuple(ptuTuplePtr->getPan(),
                                                                      ptuTuplePtr->getTilt(),
                                                                      ptuTuplePtr->getCameraPosePtr(),
                                                                      ptuTuplePtr->getDeactivatedObjectNormalsCount(),
                                                                      ptuTuplePtr->getAlreadySearchedObjectTypes())));
            RobotStatePtr robotStatePtrCopy = RobotStatePtr(new RobotState(robotStatePtr->getRobotPosePtr(), tempPtuTuplePtrVecPtr));
            robotStatesToReorder->push_back(robotStatePtrCopy);
        }
    }

    ROS_DEBUG_STREAM("Number of poses to explore with only one PTU config per each: " << robotStatesToReorder->size());

    if (!rateViewportsServiceClient.call(rateViewports)) {
        ROS_ERROR("Service call to rateViewportsServiceClient was not successful");
        return false;
    }

    const std::vector<next_best_view::RatedViewport> &ratedViewports = rateViewports.response.sortedRatedViewports;

    ROS_ASSERT_MSG(ratedViewports.size() == robotStatesToReorder->size(),
                   "The number of returned viewports from RateViewports were not equale, expect: %zu actual: %zu",
                   robotStatesToReorder->size(), ratedViewports.size());

    int numberOfPosesWhereRatingIsNot0 = 0;

    RobotStatePtrVecPtr reorderedRobotStates = RobotStatePtrVecPtr(new RobotStatePtrVec());
    for (unsigned int currentIndex = 0; currentIndex < ratedViewports.size(); ++currentIndex) {
        const next_best_view::RatedViewport &currentRatedViewport = ratedViewports[currentIndex];
        ROS_ASSERT_MSG(poseHelperPtr->checkPosesAreApproxEquale(currentRatedViewport.pose,
                                                                *(*robotStatesToReorder)[currentRatedViewport.oldIdx]->getTopPtuTuplePtr()->getCameraPosePtr(),
                       0.0001, 0.0001),
                "The Pose of currentRatedViewport does not fit to my Pose at oldIdx, for currentIndexOfCurrentRatedViewport: %u currentRatedViewport.oldIndex: %u",
                currentIndex, currentRatedViewport.oldIdx);

        const RobotStatePtr &robotStatePtrToInsert = (*robotStatesToReorder)[currentRatedViewport.oldIdx];

        if (currentRatedViewport.rating != 0.0) {
            ++numberOfPosesWhereRatingIsNot0;

            // Split robotState in two. In first run look for expected objects and in second for the remaining
            PtuTuplePtrVecPtr copyPtuTuplePtrVecPtr = PtuTuplePtrVecPtr(new PtuTuplePtrVec());
            copyPtuTuplePtrVecPtr->push_back(PtuTuplePtr(new PtuTuple(robotStatePtrToInsert->getTopPtuTuplePtr()->getPan(),
                                                                      robotStatePtrToInsert->getTopPtuTuplePtr()->getTilt(),
                                                                      robotStatePtrToInsert->getTopPtuTuplePtr()->getCameraPosePtr(),
                                                                      robotStatePtrToInsert->getTopPtuTuplePtr()->getDeactivatedObjectNormalsCount(),
                                                                      robotStatePtrToInsert->getTopPtuTuplePtr()->getAlreadySearchedObjectTypes())));
            RobotStatePtr robotStatePtrToInsertCopy = RobotStatePtr(new RobotState(robotStatePtrToInsert->getRobotPosePtr(), copyPtuTuplePtrVecPtr));
            robotStatePtrToInsertCopy->getTopPtuTuplePtr()->setRatedViewport(currentRatedViewport);


            // alreadySearchedObjectTypesForFirstRun = searchedObjectTypes \ currentRatedViewport.object_type_name_list
            // -> nextFilteredSearchedObjectTypes = searchedObjectTypes \ alreadySearchedObjectTypesForFirstRun = currentRatedViewport.object_type_name_list
            SearchedObjectTypes alreadySearchedObjectTypesForFirstRun =
                    getSearchedObjectTypesFromAWithoutB(searchedObjectTypes, currentRatedViewport.object_type_name_list);
            robotStatePtrToInsertCopy->getTopPtuTuplePtr()->addAlreadySearchedObjectTypes(alreadySearchedObjectTypesForFirstRun);
            reorderedRobotStates->push_back(robotStatePtrToInsertCopy);

            robotStatePtrToInsert->getTopPtuTuplePtr()->addAlreadySearchedObjectTypes(currentRatedViewport.object_type_name_list);
            robotStatePtrToInsert->getTopPtuTuplePtr()->setRatedViewport(next_best_view::RatedViewport());
        }
    }
    ROS_DEBUG_STREAM("numberOfPosesWhereRatingIsNot0: " << numberOfPosesWhereRatingIsNot0);

    ROS_DEBUG_STREAM("Number of new poses to explore: " << reorderedRobotStates->size());
    reorderedRobotStates->insert(reorderedRobotStates->end(), robotStatesToReorder->begin(), robotStatesToReorder->end());
    ROS_DEBUG_STREAM("Number of all poses to explore: " << reorderedRobotStates->size());

    ROS_DEBUG_STREAM("Check if some poses are not needed anymore, because they have no objects to search for");
    success &= filterPosesDependingOnAlreadyFoundObjectTypes(reorderedRobotStates, searchedObjectTypes);
    ROS_DEBUG_STREAM("Number of poses to explore after filterPosesDependingOnAlreadyFoundObjectTypes: " << reorderedRobotStates->size());

    ROS_DEBUG_STREAM("Concat poses again");
    DoFilterIteration concatFilterIteration(reorderedRobotStates);
    FilterBasicPtr concatApproxEqualePosesPtr =
            ConcatApproxEqualePosesPtr(new ConcatApproxEqualePoses(reorderedRobotStates, 0.0001, 0.0001, true, false));
    concatFilterIteration.addFilter(concatApproxEqualePosesPtr);
    success &= concatFilterIteration.doIteration();

    posesToExplorePtr = reorderedRobotStates;
    ROS_DEBUG_STREAM("Number of poses to explore after concate again: " << posesToExplorePtr->size());

    // With one swap there can be at max two new poses
    int numberOfNewPoses = static_cast<int>(posesToExplorePtr->size()) - orginalNumberOfPoses;
    ROS_DEBUG_STREAM("Number of new poses after reorder: " << numberOfNewPoses);
    return success;
}

bool DirectSearchManager::deleteAllRedundantPoses(const RobotStatePtrVecPtr &robotStates) const {
    ROS_DEBUG_STREAM("deleteAllRedundantPoses");
    DoFilterIteration concatFilterIterationForDeleteRedundant(robotStates);
    FilterBasicPtr deleteApproxEqualePosesPtr =
            ConcatApproxEqualePosesPtr(new ConcatApproxEqualePoses(robotStates, 0.0001, 0.0001, false, true, false));
    concatFilterIterationForDeleteRedundant.addFilter(deleteApproxEqualePosesPtr);
    return concatFilterIterationForDeleteRedundant.doIteration();
}

bool DirectSearchManager::filterPosesDependingOnAlreadyFoundObjectTypes(const RobotStatePtrVecPtr &robotStates, const SearchedObjectTypes &searchedObjectTypes) const {
    ROS_DEBUG_STREAM("filterPosesDependingOnAlreadyFoundObjectTypes");
    DoFilterIteration doFilterIterationForFilterPosesDependingOnAlreadyFoundObjectTypes = DoFilterIteration(robotStates);
    FilterPosesDependingOnAlreadyFoundObjectTypesPtr filterPosesDependingOnAlreadyFoundObjectTypes = FilterPosesDependingOnAlreadyFoundObjectTypesPtr(
                new FilterPosesDependingOnAlreadyFoundObjectTypes(robotStates));
    filterPosesDependingOnAlreadyFoundObjectTypes->updateSearchedObjectTypes(searchedObjectTypes);
    doFilterIterationForFilterPosesDependingOnAlreadyFoundObjectTypes.addFilter(filterPosesDependingOnAlreadyFoundObjectTypes);
    return doFilterIterationForFilterPosesDependingOnAlreadyFoundObjectTypes.doIteration();
}

bool DirectSearchManager::setInitialRobotState() {
    next_best_view::SetInitRobotState setInitRobotState;
    setInitRobotState.request.robotState.pan = currentPan;
    setInitRobotState.request.robotState.tilt = currentTilt;

    geometry_msgs::Pose currentRobotPose = poseHelperPtr->getCurrentRobotPose();
    setInitRobotState.request.robotState.x = currentRobotPose.position.x;
    setInitRobotState.request.robotState.y = currentRobotPose.position.y;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(currentRobotPose.orientation, quat);
    setInitRobotState.request.robotState.rotation = quat.getAngle();

    return setInitRobotStateServiceClient.call(setInitRobotState);
}

void DirectSearchManager::ptuDriverStateCallback(const sensor_msgs::JointState::ConstPtr &ptuState) {
    currentPan = ptuState->position[0];
    currentTilt = ptuState->position[1];
}

bool DirectSearchManager::setPointCloudInNBV(const SearchedObjectTypesAndIds &searchedObjectTypesAndIds) {
    next_best_view::SetAttributedPointCloud setAttributePointCloud;
    bool success = getViewportsFromWorldModel(setAttributePointCloud.request.viewports_to_filter);

    ISM::TableHelperPtr table_helper = ISM::TableHelperPtr(new ISM::TableHelper(dbfilenameParam));

    const std::vector<int> &setIds = table_helper->getSetIds();
    for (const int &setId : setIds) {
        const ISM::ObjectSetPtr &objectSetPtr = table_helper->getRecordedObjectSet(setId);
        for (const ISM::ObjectPtr &object : objectSetPtr->objects) {
            bool shouldAdd = false;
            // only add objects and ids which we are looking for
            for (const asr_msgs::AsrTypeAndId &searchedObjectTypeAndId : searchedObjectTypesAndIds) {
                if (searchedObjectTypeAndId.type == object->type && searchedObjectTypeAndId.identifier == object->observedId) {
                    shouldAdd = true;
                    break;
                }
            }
            if (shouldAdd) {
                asr_msgs::AsrAttributedPoint pointCloudPoint;
                pointCloudPoint.type = object->type;
                pointCloudPoint.identifier = object->observedId;
                pointCloudPoint.pose.position.x = object->pose->point->eigen[0];
                pointCloudPoint.pose.position.y = object->pose->point->eigen[1];
                pointCloudPoint.pose.position.z = object->pose->point->eigen[2];
                pointCloudPoint.pose.orientation.w = object->pose->quat->eigen.w();
                pointCloudPoint.pose.orientation.x = object->pose->quat->eigen.x();
                pointCloudPoint.pose.orientation.y = object->pose->quat->eigen.y();
                pointCloudPoint.pose.orientation.z = object->pose->quat->eigen.z();
                setAttributePointCloud.request.point_cloud.elements.push_back(pointCloudPoint);
            }
        }
    }

    success &= setAttributedPointCloudServiceClient.call(setAttributePointCloud);
    success &= setAttributePointCloud.response.is_valid;
    return success;
}


bool checkSearchedObjectTypesAreEquale(const SearchedObjectTypes &searchecObjectTypes1, const SearchedObjectTypes &searchecObjectTypes2) {
    bool equale = searchecObjectTypes1.size() == searchecObjectTypes2.size();
    if (equale) {
        SearchedObjectTypes copySearchecObjectTypes1(searchecObjectTypes1);
        SearchedObjectTypes copySearchecObjectTypes2(searchecObjectTypes2);
        std::sort(copySearchecObjectTypes1.begin(), copySearchecObjectTypes1.end());
        std::sort(copySearchecObjectTypes2.begin(), copySearchecObjectTypes2.end());
        equale = std::equal(copySearchecObjectTypes1.begin(), copySearchecObjectTypes1.begin() + copySearchecObjectTypes1.size(), copySearchecObjectTypes2.begin());
    }
    return equale;
}

SearchedObjectTypes getSearchedObjectTypesFromAWithoutB(const SearchedObjectTypes &a, const SearchedObjectTypes &b) {
    // aWithoutB = a \ b
    SearchedObjectTypes aWithoutB = a;
    filterSearchedObjectTypes(aWithoutB, b);
    return aWithoutB;
}

}


