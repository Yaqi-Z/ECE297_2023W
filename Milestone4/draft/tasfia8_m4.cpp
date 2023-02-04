/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


#include "m4.h"
#include "m1.h"
#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include <math.h>





#include <iostream>
#include <math.h>
#include <string>
#include <algorithm>
#include <vector>
#include <unordered_set>
#include <map>
#include <cctype>
#include <locale>
#include <unordered_map>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>

#include "OSMDatabaseAPI.h"
#include "globals.h"

#define TIME_LIMIT 50



struct deliveryDistStruct1 {
    std::vector<double> pickUp;
    std::vector<double> dropOff;
};

struct deliveryDistStruct2 {
    std::vector<deliveryDistStruct1> pickUp;
    std::vector<deliveryDistStruct1> dropOff;
};


// Create a copy of DeliveryInf data structure from m4.h to modify if needed
struct DeliveryInfo {
    //Specifies a delivery order (input to your algorithm).
    //
    //To satisfy the order the item-to-be-delivered must have been picked-up 
    //from the pickUp intersection before visiting the dropOff intersection.

    DeliveryInfo(int pick_up, int drop_off)
        : pickUp(pick_up), dropOff(drop_off) {}

    //The intersection id where the item-to-be-delivered is picked-up.
    int pickUp;

    //The intersection id where the item-to-be-delivered is dropped-off.
    int dropOff;
};



struct DeliveryIntersection {
    DeliveryIntersection(std::string Type, int Intersec_Num)
        : type(Type), num(Intersec_Num) {}
    
    std::string type;
    int num;
};


DeliveryIntersection findNextDeliveryIntersection(const deliveryDistStruct2 & deliveryInterDistances,
                    const DeliveryIntersection fromIntersection, std::vector<bool> pickedUp, std::vector<bool> droppedOff, bool applyRandomness) {
    
    if (fromIntersection.type == "pickUp") {
        pickedUp[fromIntersection.num] = true;
    } else if (fromIntersection.type == "dropOff") {
        droppedOff[fromIntersection.num] = true;
    }

    DeliveryIntersection currentIntersection = fromIntersection;    
    DeliveryIntersection nextIntersection("pickUp", -1);
    DeliveryIntersection secondBestNextInter("pickUp", -1);
    
    double dist;
    double minDist = -1;
        
    if (currentIntersection.type == "pickUp") {
        //check through all pickUp intersections
        for (int i = 0; i < deliveryInterDistances.pickUp[currentIntersection.num].pickUp.size(); i++) {
            if (pickedUp[i] == false && currentIntersection.num != i) {
                dist = deliveryInterDistances.pickUp[currentIntersection.num].pickUp[i];
                if (minDist == -1 || dist < minDist) {
                    minDist = dist;
                    secondBestNextInter = nextIntersection;
                    nextIntersection.num = i;
                }
            }
        }

        //check through all permissible dropOff intersections
        for (int i = 0; i < deliveryInterDistances.pickUp[currentIntersection.num].dropOff.size(); i++) {
            if (pickedUp[i] == true && droppedOff[i] == false) {
                dist = deliveryInterDistances.pickUp[currentIntersection.num].dropOff[i];
                if (minDist == -1 || dist < minDist) {
                    minDist = dist;
                    secondBestNextInter = nextIntersection;
                    nextIntersection.num = i;
                    nextIntersection.type = "dropOff";
                }
            }
        }
    } else if (currentIntersection.type == "dropOff") {
        for (int i = 0; i < deliveryInterDistances.dropOff[currentIntersection.num].pickUp.size(); i++) {
            if (pickedUp[i] == false) {
                dist = deliveryInterDistances.dropOff[currentIntersection.num].pickUp[i];
                if (minDist == -1 || dist < minDist) {
                    minDist = dist;
                    secondBestNextInter = nextIntersection;
                    nextIntersection.num = i;
                }
            }
        }

        //check through all permissible dropOff intersections
        for (int i = 0; i < deliveryInterDistances.dropOff[currentIntersection.num].dropOff.size(); i++) {
            if (pickedUp[i] == true && droppedOff[i] == false  && currentIntersection.num != i) {
                dist = deliveryInterDistances.dropOff[currentIntersection.num].dropOff[i];
                if (minDist == -1 || dist < minDist) {
                    minDist = dist;
                    secondBestNextInter = nextIntersection;
                    nextIntersection.num = i;
                    nextIntersection.type = "dropOff";
                }
            }
        }
    }
    
    
    int randNum = rand() % 100;
    
    if (applyRandomness == false || minDist == 0.0 || secondBestNextInter.num == -1 || randNum < 90) {
        return nextIntersection;
    } else {
        return secondBestNextInter;
    }
}



std::vector<IntersectionIdx> deliveryOrderIntersections(const std::vector<DeliveryInf>& deliveries, const deliveryDistStruct2 & deliveryInterDistances, 
                        std::vector<IntersectionIdx>  deliveryOrderIntersec, std::vector<bool> pickedUp, std::vector<bool> droppedOff, int pickUpStartNum, bool applyRandomness){
    
    //find the order to traverse through the delivery intersections
    DeliveryIntersection currentIntersection("pickUp", pickUpStartNum);
    DeliveryIntersection nextIntersection("pickUp", -1);
    DeliveryIntersection next_nextIntersection("pickUp", -1);
    
    DeliveryIntersection candidate_1aheadInter("pickUp", -1);
    
    while (deliveryOrderIntersec.size() < 2 * deliveries.size() + 1) {
                    
        nextIntersection = findNextDeliveryIntersection(deliveryInterDistances, currentIntersection, pickedUp, droppedOff, applyRandomness);

        if (nextIntersection.type == "pickUp") {
            pickedUp[nextIntersection.num] = true;
            deliveryOrderIntersec.push_back(deliveries[nextIntersection.num].pickUp);
        } else if (nextIntersection.type == "dropOff") {
            droppedOff[nextIntersection.num] = true;
            deliveryOrderIntersec.push_back(deliveries[nextIntersection.num].dropOff);
        }

        currentIntersection = nextIntersection;
    }
    
    
    
   //Return the order of intersections to run the delivery
    return deliveryOrderIntersec;
}



std::vector<CourierSubPath> travelingCourier(const std::vector<DeliveryInf>& deliveries,
	       	        const std::vector<int>& depots, const float turn_penalty) {
    
    //start timing to make sure program doesn't exceed time limit
    auto startTime = std::chrono::high_resolution_clock::now();
    bool timeOut = false;
    
    //seed the random number generator
    srand(time(NULL));
    
    
    std::vector<CourierSubPath> courierPath;
    
    int numDeliveryTasks;
    numDeliveryTasks = deliveries.size();
    
    //create a 2D vector to store the geometric distances between each pair of delivery intersections
    //******* Example: the distance between the pickUp intersection of the 0th delivery and the dropOff intersection of the 3rd delivery is given by:  
    // deliveryInterDistances.pickUp[0].dropOff[3] *******
    deliveryDistStruct1 tempInitialization;
    tempInitialization.pickUp.resize(numDeliveryTasks, 0);
    tempInitialization.dropOff.resize(numDeliveryTasks, 0);
    deliveryDistStruct2 deliveryInterDistances;
    deliveryInterDistances.pickUp.resize(numDeliveryTasks, tempInitialization);
    deliveryInterDistances.dropOff.resize(numDeliveryTasks, tempInitialization);
        
    LatLon inter1_position, inter2_position;
    
    #pragma omp parallel for
    for (int i = 0; i < numDeliveryTasks; i++) {
        for (int j = 0; j < numDeliveryTasks; j++) {
            inter1_position = getIntersectionPosition(deliveries[i].pickUp);
            inter2_position = getIntersectionPosition(deliveries[j].pickUp);
            deliveryInterDistances.pickUp[i].pickUp[j] = findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> (inter1_position, inter2_position));
            
            inter1_position = getIntersectionPosition(deliveries[i].pickUp);
            inter2_position = getIntersectionPosition(deliveries[j].dropOff);
            deliveryInterDistances.pickUp[i].dropOff[j] = findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> (inter1_position, inter2_position));
            
            inter1_position = getIntersectionPosition(deliveries[i].dropOff);
            inter2_position = getIntersectionPosition(deliveries[j].pickUp);
            deliveryInterDistances.dropOff[i].pickUp[j] = findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> (inter1_position, inter2_position));
            
            inter1_position = getIntersectionPosition(deliveries[i].dropOff);
            inter2_position = getIntersectionPosition(deliveries[j].dropOff);
            deliveryInterDistances.dropOff[i].dropOff[j] = findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> (inter1_position, inter2_position));
        }
    }
    
    //load geometric distances from each depot to each intersection
    std::vector<deliveryDistStruct1> depotInterDistances;
    depotInterDistances.resize(depots.size(), tempInitialization);
    
    
    for (int i = 0; i < depots.size(); i++) {
        for (int j = 0; j < numDeliveryTasks; j++) {
            inter1_position = getIntersectionPosition(depots[i]);
            inter2_position = getIntersectionPosition(deliveries[j].pickUp);
            depotInterDistances[i].pickUp[j] = findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> (inter1_position, inter2_position));
            
            inter1_position = getIntersectionPosition(depots[i]);
            inter2_position = getIntersectionPosition(deliveries[j].dropOff);
            depotInterDistances[i].dropOff[j] = findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> (inter1_position, inter2_position));
        }
    }
    
    
    
    //Making a vector of unique deliveries, depots intersections then put it in the intersection sets
    std::vector<IntersectionIdx>  deliveryOrderIntersec;
    std::vector<bool> pickedUp(deliveries.size(), false);
    std::vector<bool> droppedOff(deliveries.size(), false);
    
    //find a valid depot to start at as well as the closest pickUp intersection to it
    int depotStartNum = 0;
    std::vector<double>::iterator depotMin = std::min_element(depotInterDistances[depotStartNum].pickUp.begin(), depotInterDistances[depotStartNum].pickUp.end());
    int pickUpStartNum = std::distance(depotInterDistances[depotStartNum].pickUp.begin(), depotMin);
    while (findPathBetweenIntersections(depots[depotStartNum], deliveries[pickUpStartNum].pickUp, 0.0)[0] == 0) {
        depotStartNum++;
        depotMin = std::min_element(depotInterDistances[depotStartNum].pickUp.begin(), depotInterDistances[depotStartNum].pickUp.end());
        pickUpStartNum = std::distance(depotInterDistances[depotStartNum].pickUp.begin(), depotMin);
    }
        
    deliveryOrderIntersec.push_back(depots[depotStartNum]);
    deliveryOrderIntersec.push_back(deliveries[pickUpStartNum].pickUp);
    pickedUp[pickUpStartNum] = true;
    
    
    
    //find path based on always taking the shortest distance to the next intersection
    std::vector<IntersectionIdx> deliveryOrderInters = deliveryOrderIntersections(deliveries, deliveryInterDistances, 
            deliveryOrderIntersec, pickedUp, droppedOff, pickUpStartNum, false);  //this vector represents the intersection IDs of each delivery order
    

    double totalPathTravelTime = 0;

       //iterating through the order by calling findPathBetweenIntersections function finding the optimal path between start and end intersection
    for (int interNum = 0 ; interNum + 1 < deliveryOrderInters.size(); interNum++) {
        CourierSubPath subPath;
        subPath.start_intersection = deliveryOrderInters[interNum];
        subPath.end_intersection = deliveryOrderInters[interNum + 1];
        subPath.subpath = findPathBetweenIntersections(subPath.start_intersection, subPath.end_intersection, turn_penalty);
        courierPath.push_back(subPath);

        totalPathTravelTime = totalPathTravelTime + computePathTravelTime(subPath.subpath, turn_penalty);
    }
    //std::cout << "total travel time: " << totalPathTravelTime << "\n";

    double bestTotalTravelTime = totalPathTravelTime;
    std::vector<CourierSubPath> bestCourierPath = courierPath;
    courierPath.clear();
    
    int numTrials = 1;
    
    //apply randonness factor in selecting the next intersection to travel to
    /*while (!timeOut && numTrials <= 10) {
        
        std::vector<IntersectionIdx> deliveryOrderInters = deliveryOrderIntersections(deliveries, deliveryInterDistances, 
            deliveryOrderIntersec, pickedUp, droppedOff, pickUpStartNum, true); 
        
        totalPathTravelTime = 0;

           //iterating through the order by calling findPathBetweenIntersections function finding the optimal path between start and end intersection
        for (int interNum = 0 ; interNum + 1 < deliveryOrderInters.size(); interNum++) {
            CourierSubPath subPath;
            subPath.start_intersection = deliveryOrderInters[interNum];
            subPath.end_intersection = deliveryOrderInters[interNum + 1];
            subPath.subpath = findPathBetweenIntersections(subPath.start_intersection, subPath.end_intersection, turn_penalty);
            courierPath.push_back(subPath);

            totalPathTravelTime = totalPathTravelTime + computePathTravelTime(subPath.subpath, turn_penalty);
        }
        std::cout << "total travel time: " << totalPathTravelTime << "\n";
        
        if (totalPathTravelTime < bestTotalTravelTime) {
            bestTotalTravelTime = totalPathTravelTime;
            bestCourierPath = courierPath;
        }
        courierPath.clear();
        
        
        //examine current time and indicate timeOut if exceeded 45 seconds
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
        if (wallClock.count() > 0.7 * TIME_LIMIT) {
            timeOut = true;
            std::cout << "Execution time: " << wallClock.count() << "\n";
        }
        
        numTrials++;
    }*/
    
    
    
    //find the closest valid depot to end the delivery
    /*double depotDist, depotMinDist;
    int depotEndNum;
    bool invalidDepot[depots.size()] = {false};
    bool depotNotFound = true;
    while (depotNotFound) {
        depotMinDist = -1;
        for (int i = 0; i < depots.size(); i++) {
            if (invalidDepot[i] == false) {
                depotDist = depotInterDistances[i].dropOff[currentIntersection.num];
                if (depotMinDist == -1 || depotDist < depotMinDist) {
                    depotMinDist = depotDist;
                    depotEndNum = i;
                }
            }
        }
        
        depotMin = std::min_element(depotInterDistances[depotStartNum].pickUp.begin(), depotInterDistances[depotStartNum].pickUp.end());
        pickUpStartNum = std::distance(depotInterDistances[depotStartNum].pickUp.begin(), depotMin);

        if (findPathBetweenIntersections(depots[depotEndNum], deliveries[depotStartNum].pickUp, 0.0)[0] == 0) {
            invalidDepot[depotEndNum] = true;
            depotNotFound = true;
        } else {
            depotNotFound = false;
        }
    }*/
    int depotEndNum = depotStartNum;
    CourierSubPath subPath;
    subPath.start_intersection = bestCourierPath[2 * deliveries.size() - 1].end_intersection;
    subPath.end_intersection = depots[depotEndNum];
    subPath.subpath = findPathBetweenIntersections(subPath.start_intersection, subPath.end_intersection, turn_penalty);
    bestCourierPath.push_back(subPath);

    return bestCourierPath;
} 

