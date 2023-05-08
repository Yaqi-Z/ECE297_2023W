//
// Created by lihanli3 on 3/23/23.
//

#ifndef MAPPER_M3_EXTEND_H
#define MAPPER_M3_EXTEND_H

#include <queue>
#include <map>
#include <unordered_map>
#include <queue>
#include <climits>
// Graph data structure goes here

// Graph node: used for path storing and backtracking
struct myNode {
    // outgoing edges
    // std::vector<StreetSegmentIdx> toEdge;
    // record the path; where do I come from
    StreetSegmentIdx fromEdge;
    double bestTime;
    bool isClosed;
    myNode(){ fromEdge = -2; bestTime = INT_MAX; isClosed = false;}
    myNode(double bestTime_){ bestTime = bestTime_; isClosed = false;}
    myNode(StreetSegmentIdx fromEdge_, double bestTime_, bool isClosed_) {
        fromEdge = fromEdge_; bestTime = bestTime_; isClosed = isClosed_;
    }
};

struct DijkstraNode {
    IntersectionIdx nodeID;
    StreetSegmentIdx edgeFrom;
    double travelTime;
    DijkstraNode(){}
    DijkstraNode(int nodeID_, int edgeFrom_, double travelTime_): nodeID(nodeID_), edgeFrom(edgeFrom_), travelTime(travelTime_) {}

    bool operator>(const DijkstraNode& rhs) const { return travelTime > rhs.travelTime; }
    bool operator<(const DijkstraNode& rhs) const { return travelTime < rhs.travelTime; }
    bool operator>=(const DijkstraNode& rhs) const { return travelTime >= rhs.travelTime; }
    bool operator<=(const DijkstraNode& rhs) const { return travelTime <= rhs.travelTime; }
    bool operator==(const DijkstraNode& rhs) const { return travelTime == rhs.travelTime; }
    bool operator!=(const DijkstraNode& rhs) const { return travelTime != rhs.travelTime; }
};

struct AStarNode{
    IntersectionIdx nodeID;
    StreetSegmentIdx edgeFrom;
    double GTravelTime;
    double HEstTime;
    double FVal;
    AStarNode(){}
    AStarNode(int nodeID_, int edgeFrom_, double GTravelTime_, double HEstTime_){
        nodeID = nodeID_; edgeFrom = edgeFrom_; GTravelTime = GTravelTime_; HEstTime = HEstTime_;
        FVal = GTravelTime + HEstTime;
    }
    bool operator>(const AStarNode& rhs) const {
        return FVal > rhs.FVal;
    }
    bool operator<(const AStarNode& rhs) const {
        return FVal < rhs.FVal;
    }
    bool operator>=(const AStarNode& rhs) const {
        return FVal >= rhs.FVal;
    }
    bool operator<=(const AStarNode& rhs) const {
        return FVal <= rhs.FVal;
    }
    bool operator==(const AStarNode& rhs) const {
        return FVal == rhs.FVal;
    }
    bool operator!=(const AStarNode& rhs) const {
        return FVal != rhs.FVal;
    }
};

bool DijkstraSearch(std::vector<myNode>& myNodesVec,
        const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids,
        const double turn_penalty);

bool AStarSearch(
        std::vector<myNode>& myNodesVec,
        const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids,
        const double turn_penalty);

double heuristicFunc(IntersectionIdx point_from, IntersectionIdx point_to);

bool isTurning(StreetSegmentIdx ss1, StreetSegmentIdx ss2);

template<typename T>
void insert_descending_sorted(T &new_ele, std::vector<T> &cur_vector);

std::string showPathDetails(const std::vector<StreetSegmentIdx>& path, const double turn_penalty);

std::string formatTotalDistance(double inputNum);
std::string formatTotalTime(double inputNum);
std::string streetNameToPrint(std::string& streetName);
std::string formatToTenth(double inputNum);
std::string formateToWhole(double inputNum);
std::string getTurningDir(IntersectionIdx prev, IntersectionIdx cur, IntersectionIdx next);


#endif //MAPPER_M3_EXTEND_H

