
// To change this license header, choose License Headers in Project Properties.
// To change this template file, choose Tools | Templates
// and open the template in the editor.
///
#include "m1.h"
#include "m3.h"
#include "m3_helper_functions.h"
#include <math.h>
#include "globals.h"
#include "point.hpp"
#include <string>

void printTravelDirections(std::vector<StreetSegmentIdx> path) {
    StreetSegmentIdx segId;
    StreetIdx currentStreetId, previousStreetId;
    std::string currentStreetName;
    double streetTravelTime = 0;
    
    std::cout << "\n" << "**Travel Directions**" << "\n" << "\n";
    
    //if empty vector passed in
    if(path.size() == 0) {
        std::cout << "Empty travel path!" << "\n" << "\n";
        return;
    }
        
    //if path is not an empty vector
    for(int segNum = 0; segNum < path.size(); segNum++) {
        segId = path[segNum];
        currentStreetId = getStreetSegmentInfo(segId).streetID;
                
        if(segNum == 0) { //for the first street, simply print out its name
            currentStreetName = getStreetName(currentStreetId);
            std::cout << "Start on: " << currentStreetName << " (id: " << currentStreetId << ")" << " and drive for about " << (int) streetTravelTime << " seconds" << "\n" << "\n";
        } else if (segNum >= 1 && currentStreetId != previousStreetId) { //if a turning has occurred, then print instruction to turn and give new street name
            currentStreetName = getStreetName(currentStreetId);
            std::cout << "Proceed onto: " << currentStreetName << " (id: " << currentStreetId << ")" << " and drive for about " << (int) streetTravelTime << " seconds" << "\n" << "\n";
            streetTravelTime = 0;
        }
        
        if (segNum == path.size() - 1) {
            std::cout << "Arrived at the destination!" << "\n" << "\n";
        }
        
        streetTravelTime = streetTravelTime + findStreetSegmentTravelTime(segId);
        
        previousStreetId = currentStreetId;
    }
    return;
}
