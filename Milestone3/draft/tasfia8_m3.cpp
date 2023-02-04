/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "m1.h"
//#include "m2.h"
#include "m3_helper_functions.cpp"
#include "point.hpp"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <unordered_map>
#include <map>
#include "math.h"
//#include "m1.cpp"
#include <chrono>
#include <string>
#include <algorithm>
#include <list>
#include <queue>
#include <functional>
#include <queue>
#include <vector>
#include <iostream>
#include <limits>
#include "globals.h"
#include "OSMDatabaseAPI.h"

#define NO_EDGE -1


bool findPath(Node *sourceNode, int destID, const double turn_penalty);
double initial_time = std::numeric_limits<double>::max();
std::vector<StreetSegmentIdx> bfsTraceBack(int destID);



//struct called waveElem to store the nodes as we search through the path

struct waveElem {
    Node *node; //pointer to node object
    StreetSegmentIdx edgeID; // the street segment used to reach to the node/intersection
    double travelTime; // total time used to reach to the node/intersection

    waveElem(Node *n, StreetSegmentIdx id) { //constructor
        node = n;
        edgeID = id;
    }

    waveElem(Node *n, StreetSegmentIdx id, float time) {
        node = n;
        edgeID = id;
        travelTime = time;

    }
};

//Comparing the travel times of two waveElem and returning the one with the lesser travel time

struct Compare {bool operator()(const struct waveElem &lhs, const struct waveElem &rhs) {
        return lhs.travelTime > rhs.travelTime;}};



//Function used to check if fastest path found or not

bool findPath(Node *sourceNode, int destID, const double turn_penalty) {

    std::priority_queue<waveElem, std::vector<waveElem>, Compare> wavefront;
    bool checkoneWay = true;
    wavefront.push(waveElem(sourceNode, NO_EDGE, 0.0)); //push sourceNode into the priority queue

    sourceNode = &(searchNode[sourceNode->id]);

    sourceNode->reachingEdge = -1;
    if (sourceNode->visitedNode == true) {
        sourceNode->visitedNode = false;
    }

    sourceNode->bestTime = 0;
    double timeTaken = 0;
    while (wavefront.size() != 0) {
        waveElem wave = wavefront.top();
        wavefront.pop(); //remove node from wavefront 
        Node *currNode = wave.node; //setting a pointer to the node removed from the wavefront

        currNode = &(searchNode[currNode->id]);

        //if node is equal to the destination id, return true           
        if (currNode->id == destID) {
            return true;
        }

        currNode->visitedNode = true;

        for (int x = 0; x < findAdjs[currNode->id].size(); x++) {

            IntersectionIdx n;
            StreetIdx streetiden = -1;
            if (currNode->reachingEdge != -1) { //don't do this check if current node is source node
                streetiden = getStreetSegmentInfo(currNode->reachingEdge).streetID;
            }

            bool oneW = true;

            StreetSegmentIdx newst = findAdjs[currNode->id][x];
            //finding intersection at end of street segment
            IntersectionIdx fromInt = (getStreetSegmentInfo(newst).from);
            IntersectionIdx toInt = (getStreetSegmentInfo(newst).to);
            oneW = getStreetSegmentInfo(newst).oneWay;

            bool check2 = true;

            if (toInt == currNode->id && oneW == false) {
                n = fromInt;
            } else {
                n = toInt;
            }

            Node *toNode;

            if (checkoneWay == true) {
                toNode = &(searchNode[n]);
            }

            //check if the node is visited or not
            if (toNode->visitedNode == false && currNode->visitedNode == true) {


                double check = findStreetSegmentTravelTime(newst);
                StreetIdx tonodeStreetID = getStreetSegmentInfo(newst).streetID;
                //if street changes, add turn penalty
                if (currNode->reachingEdge != -1 && streetiden != tonodeStreetID) {
                    timeTaken = currNode->bestTime + check + turn_penalty;
                } else {
                    timeTaken = currNode->bestTime + check;
                }


                if (toNode->bestTime == initial_time) {
                    toNode->bestTime = timeTaken;
                    toNode->reachingEdge = newst;
                }
                //only update bestTime and reachingedge for node if timetaken is less than the node's previous best time
                if (timeTaken < toNode->bestTime) {
                    toNode->bestTime = timeTaken;
                    toNode->reachingEdge = newst;
                }

                //don't update if timeTaken is greater than toNode 
                if (timeTaken > toNode->bestTime && toNode->bestTime != initial_time) {
                    check2 = false;
                }

                //push the node into the wavefront 
                if (check2 == true) {
                    wavefront.push(waveElem(toNode, newst, (timeTaken)));
                }
            }
        }
    }
    return false;
}


//Tracing back the path returned from the destination id to the source id

std::vector<StreetSegmentIdx> bfsTraceBack(int destID) {
    std::list<int> path;

    //Get destination id node and set the previous edge
    Node *currNode = &(searchNode[destID]);
    int prevEdge = currNode->reachingEdge;

    //loop to keep checking each node's previous edge until it reaches the source node which has an edge of -1
    while (prevEdge != NO_EDGE) {

        IntersectionIdx m = getStreetSegmentInfo(prevEdge).from;
        path.push_front(prevEdge);
        if (m != currNode->id) {
           currNode = &(searchNode[m]);
            prevEdge = currNode->reachingEdge;
        } else {
            currNode = &(searchNode[getStreetSegmentInfo(prevEdge).to]);
            prevEdge = currNode->reachingEdge;
        }
    }
    //converting list into a vector
    std::vector<StreetSegmentIdx> result(path.begin(), path.end());
    //return the vector
    return result;
}

double computePathTravelTime(const std::vector<StreetSegmentIdx>& path,
        const double turn_penalty) {
    double totalTravelTime = 0;
    StreetSegmentIdx segId;
    StreetIdx currentStreetId, previousStreetId;

    if (path.size() == 0) return 0; //if empty vector passed in

    for (int segNum = 0; segNum < path.size(); segNum++) {
        segId = path[segNum];
        totalTravelTime = totalTravelTime + findStreetSegmentTravelTime(segId); //for the first street segment

        currentStreetId = getStreetSegmentInfo(segId).streetID;
        //if street changes, add turn penalty 
        if (segNum >= 1 && currentStreetId != previousStreetId) {
            totalTravelTime = totalTravelTime + turn_penalty;

        }
        previousStreetId = currentStreetId;
    }

    return (totalTravelTime); //return total travel time
}

std::vector<StreetSegmentIdx> findPathBetweenIntersections(
        const IntersectionIdx intersect_id_start,
        const IntersectionIdx intersect_id_destination,
        const double turn_penalty) {

    Node* y = &(searchNode[intersect_id_start]);
    //resetting the values for all the nodes before findPath is called
    for (IntersectionIdx intno = 0; intno < getNumIntersections(); intno++) {
        Node *reset = &(searchNode[intno]);
        reset->visitedNode = false;
        reset->bestTime = initial_time;
        reset->reachingEdge = NO_EDGE;
    }

    //calling findPath function to check if there's a path or not
    bool found = findPath(y, intersect_id_destination, turn_penalty);

    //return path if path is found
    if (found == true) {
        return bfsTraceBack(intersect_id_destination);
    }        //return vector 0
    else {
        std::vector<StreetSegmentIdx> x;
        x.push_back(0);
        return x;
    }
}
