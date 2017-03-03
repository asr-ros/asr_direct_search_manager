/**

Copyright (c) 2016, Borella Jocelyn, Karrenbauer Oliver, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <model/ptu_tuple.hpp>

namespace directSearchWS {

PtuTuple::PtuTuple(int pan, int tilt, PosePtr cameraPosePtr) : PtuTuple(pan, tilt, cameraPosePtr, -1) {
}

PtuTuple::PtuTuple(int pan, int tilt, PosePtr cameraPosePtr, int deactivated_object_normals_count) :
    PtuTuple(pan, tilt, cameraPosePtr, deactivated_object_normals_count, SearchedObjectTypes()) {
}

PtuTuple::PtuTuple(int pan, int tilt, PosePtr cameraPosePtr, int deactivated_object_normals_count, SearchedObjectTypes already_searched_object_types) :
    PtuTuple(pan, tilt, cameraPosePtr, deactivated_object_normals_count, already_searched_object_types, asr_next_best_view::RatedViewport()){
}

PtuTuple::PtuTuple(int pan, int tilt, PosePtr cameraPosePtr, int deactivated_object_normals_count, SearchedObjectTypes already_searched_object_types,
    asr_next_best_view::RatedViewport ratedViewport) : pan(pan), tilt(tilt), cameraPosePtr(cameraPosePtr), deactivated_object_normals_count(deactivated_object_normals_count),
    already_searched_object_types(already_searched_object_types), ratedViewport(ratedViewport) {
}


int PtuTuple::getPan() const {
    return pan;
}

int PtuTuple::getTilt() const {
    return tilt;
}

SearchedObjectTypes PtuTuple::getAlreadySearchedObjectTypes() const {
    return already_searched_object_types;
}

void PtuTuple::addAlreadySearchedObjectType(std::string newAlreadySearchedObjectType) {
    if (std::find(already_searched_object_types.begin(), already_searched_object_types.end(), newAlreadySearchedObjectType) == already_searched_object_types.end()) {
        already_searched_object_types.push_back(newAlreadySearchedObjectType);
    }
}

void PtuTuple::addAlreadySearchedObjectTypes(const SearchedObjectTypes &newAlreadySearchedObjectTypes) {
    for (const std::string &newSearchedObjectType : newAlreadySearchedObjectTypes) {
        addAlreadySearchedObjectType(newSearchedObjectType);
    }
}

void PtuTuple::resetAlreadySearchedObjectTypes() {
    already_searched_object_types.clear();
}

PosePtr PtuTuple::getCameraPosePtr() const {
    return cameraPosePtr;
}

int PtuTuple::getDeactivatedObjectNormalsCount() const {
    return deactivated_object_normals_count;
}

void PtuTuple::setRatedViewport(const asr_next_best_view::RatedViewport &newRatedViewport) {
    ratedViewport = newRatedViewport;
}

asr_next_best_view::RatedViewport PtuTuple::getRatedViewport() const {
    return ratedViewport;
}


std::ostream& operator<<(std::ostream &strm, const PosePtr &pose_ptr) {
    return strm << *pose_ptr;
}

std::ostream& operator<<(std::ostream &strm, const SearchedObjectTypes &already_searched_object_types) {
    strm << "(already) searched object types:";
    for (const std::string &object_type : already_searched_object_types) {
        strm << " " << object_type;
    }
    strm << '\n';
    return strm;
}


std::ostream& operator<<(std::ostream &strm, const PtuTuple &ptu_tuple) {
    strm << "PTU tuple:\n";
    strm << "pan: " << ptu_tuple.getPan() << " tilt: " << ptu_tuple.getTilt() << '\n';
    strm << ptu_tuple.getAlreadySearchedObjectTypes();
    strm << "CameraPose:\n" << ptu_tuple.getCameraPosePtr();
    strm << "DeactivatedObjectNormalsCount: " << ptu_tuple.getDeactivatedObjectNormalsCount() << '\n';
    strm << "RatedViewport: " << ptu_tuple.getRatedViewport();
    return strm;
}

std::ostream& operator<<(std::ostream &strm, const PtuTuplePtr &ptu_tuple_ptr) {
    return strm << *ptu_tuple_ptr;
}

std::ostream& operator<<(std::ostream &strm, const PtuTuplePtrVec &ptu_tuple_ptr_vec) {
    for (const PtuTuplePtr &ptu_tuple_ptr : ptu_tuple_ptr_vec) {
        strm << ptu_tuple_ptr;
    }
    return strm;
}

std::ostream& operator<<(std::ostream &strm, const PtuTuplePtrVecPtr &ptu_tuple_ptr_vec_ptr) {
    return strm << *ptu_tuple_ptr_vec_ptr;
}
}
