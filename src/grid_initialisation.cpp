/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "grid_initialisation.hpp"

namespace directSearchWS {

GridInitialisation::GridInitialisation(std::string gridFilePath)
    : DirectSearchHandler(), gridFilePath(gridFilePath), ptuTuplePtrVecPtr(), clearVision(0.0) {
}

GridInitialisation::~GridInitialisation()  { }

void GridInitialisation::initPTUPoses() {
    ros::NodeHandle nh(ros::this_node::getName());
    double panMin, panMax;
    double tiltMin, tiltMax;
    double fovH, fovV;
    nh.getParam("pan_min_angle", panMin);
    nh.getParam("pan_max_angle", panMax);
    nh.getParam("tilt_min_angle", tiltMin);
    nh.getParam("tilt_max_angle", tiltMax);
    nh.getParam("clearVision", clearVision);
    nh.getParam("fovH", fovH);
    nh.getParam("fovV", fovV);
    ROS_DEBUG_STREAM("We got params panMin:" << panMin << ", panMax: " << panMax << ", tiltMin: " << tiltMin << ", tiltMax: " << tiltMax << ", fovH: " << fovH << ", fovV :" <<fovV );
    double rangeV = tiltMax - tiltMin;
    if (panMin < -clearVision) {
        panMin = -clearVision;
    }
    if (panMax > clearVision) {
        panMax = clearVision;
    }
    double rangeH = panMax - panMin;
    int countH = ceil(rangeH/fovH);
    int countV = ceil(rangeV/fovV);
    double stepH = rangeH/countH;
    double stepV = rangeV/countV;
    ROS_DEBUG_STREAM("We calculted stepH: " << stepH << ", stepV: " << stepV << ", rangeV: "<< rangeV << ", rangeH: " << rangeH << ", countV: " << countV << ", countH: "<< countH);

    ptuTuplePtrVecPtr = PtuTuplePtrVecPtr(new PtuTuplePtrVec);
    for (int i=1;i<=countH;i++) {
        for (int j=1;j<=countV;j++) {
            ptuTuplePtrVecPtr->push_back(PtuTuplePtr(new PtuTuple(panMin+i*stepH, tiltMin+j*stepV, nullptr)));
            ROS_DEBUG_STREAM("PTU tuple created : "<< panMin+i*stepH << " , " <<tiltMin+j*stepV);
        }
    }
}

bool GridInitialisation::resetHandler() {
    bool success = DirectSearchHandler::resetHandler();
    ptuTuplePtrVecPtr->clear();
    success &= initHandler();
    return success;
}


bool GridInitialisation::initHandler() {
    initPTUPoses();
    //read config file.
    // Read the xml file into a vector
    std::ifstream ConfigFile(gridFilePath.c_str(),std::ifstream::in);
    if (!ConfigFile) {
        ROS_ERROR_STREAM("Unable to locate " << gridFilePath << ". Please make sure the launch file is correct and start again");
        return false;
    }
    std::vector<char> buffer((std::istreambuf_iterator<char>(ConfigFile)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    // Parse the buffer using the xml file parsing library into doc
    rapidxml::xml_document<> doc;
    doc.parse<0>(&buffer[0]);

    std::vector<geometry_msgs::Pose> orderedGrid;
    // Find our root node
    rapidxml::xml_node<> * root_node = doc.first_node("Posen");
    for (rapidxml::xml_node<> * pose_node = root_node->first_node("Pose"); pose_node; pose_node = pose_node->next_sibling()) {
        double X = boost::lexical_cast<double>((pose_node->first_attribute("X")->value()));
        double Y = boost::lexical_cast<double>((pose_node->first_attribute("Y")->value()));
        double Z = boost::lexical_cast<double>((pose_node->first_attribute("Z")->value()));
        geometry_msgs::Pose pose;
        pose.position.x = X;
        pose.position.y = Y;
        pose.position.z = Z;
        orderedGrid.push_back(pose);
        ROS_DEBUG_STREAM("Got Pose from XML X : " << X << ", Y : " << Y << ", Z : " << Z);

    }

    int rotationPerPose = round(180/(clearVision*2));
    clearVision = 180/(rotationPerPose*2);

    geometry_msgs::Pose robotPose = poseHelperPtr->getCurrentRobotPose();
    double dist = DBL_MAX;
    auto iter = orderedGrid.begin();
    for (auto it = orderedGrid.begin(); it != orderedGrid.end(); ++it) {
        double currentDist = poseHelperPtr->calculateDistance(*it, robotPose);
        if (currentDist < dist) {
            dist = currentDist;
            iter = it;
        }
    }
    std::vector<geometry_msgs::Pose> temp;
    if (iter != orderedGrid.begin()) {
        for (auto it = iter; it != orderedGrid.end(); ++it) {
            temp.push_back(*it);
        }
        for (auto it = orderedGrid.begin(); it != iter; ++it) {
            temp.push_back(*it);
        }
        orderedGrid = temp;
    }
    temp.clear();
    //myvector.emplace ( myvector.begin(), 100 );
    //RobotState(Pose,1,std::make_tuple(0,0));

    std::vector<geometry_msgs::Pose> returnPath = orderedGrid;
    std::reverse(returnPath.begin(), returnPath.end());
    double yaw;
    std::vector<double> yawArray;
    ROS_DEBUG_STREAM("return size " << returnPath.size());
    ROS_DEBUG_STREAM("size " << orderedGrid.size());
    ROS_DEBUG_STREAM("Begin calculation for ordered grid");
    for (auto it = orderedGrid.begin(); it != orderedGrid.end(); ++it) {
        if (it != orderedGrid.end()-1) {
            yaw = getYaw(it->position,(it+1)->position);
            ROS_DEBUG_STREAM("got yaw : " << yaw*57);
        } else {
            yaw = getYaw(it->position,(orderedGrid.front()).position);
            ROS_DEBUG_STREAM("got yaw : " << yaw*57);
        }
        yawArray.push_back(yaw);
        for (int i=0;i<rotationPerPose;i++) {
            geometry_msgs::Pose robotPose;
            geometry_msgs::Quaternion quat= tf::createQuaternionMsgFromYaw(yaw+((i+1)*(clearVision*2))*M_PI/180);
            robotPose.orientation = quat;
            robotPose.position = it->position;
            temp.push_back(robotPose);
        }
    }
    ROS_DEBUG_STREAM("Begin calculation for return grid");
    for (auto it = returnPath.begin(); it != returnPath.end(); ++it) {
        if (yawArray.back() > M_PI) {
            yaw = yawArray.back() - M_PI;
            ROS_DEBUG_STREAM("got yaw : " << yaw*57);
        } else {
            yaw = yawArray.back() + M_PI;
            ROS_DEBUG_STREAM("got yaw : " << yaw*57);
        }
        yawArray.pop_back();
        for (int i=0;i<rotationPerPose;i++) {
            geometry_msgs::Pose robotPose;
            geometry_msgs::Quaternion quat= tf::createQuaternionMsgFromYaw(yaw+((i+1)*(clearVision*2))*M_PI/180);
            robotPose.orientation = quat;
            robotPose.position = it->position;
            temp.push_back(robotPose);
        }
    }
    orderedGrid = temp;

    for (auto it = orderedGrid.begin()+1; it != orderedGrid.end(); ++it) {
        remainingPosesDistances += poseHelperPtr->calculateDistance(*it,*(it-1));
    }

    posesToExplorePtr = RobotStatePtrVecPtr(new RobotStatePtrVec);
    for (const geometry_msgs::Pose &pose : orderedGrid) {
        PtuTuplePtrVecPtr ptuList = PtuTuplePtrVecPtr(new PtuTuplePtrVec);
        for (const PtuTuplePtr &tuple : *ptuTuplePtrVecPtr) {
            ptuList->push_back(tuple);
        }

        PosePtr posePtr = boost::make_shared<geometry_msgs::Pose>(pose);
        posesToExplorePtr->push_back(RobotStatePtr(
                                         new RobotState(
                                             posePtr,
                                             ptuList)));
    }
    return true;
}

double GridInitialisation::getYaw(const geometry_msgs::Point &pointA, const geometry_msgs::Point &pointB) const {
    double yaw = atan2((pointB.y-pointA.y),(pointB.x-pointA.x));
    if ( yaw < 0.0f ) {
        yaw += 2.0f * M_PI - (90+this->clearVision)*M_PI/180;
    }
    return yaw;
}

}

