#include "m1.h"
#include "m3.h"
#include "m1_extend.h"
#include "m2_extend.h"
#include "m3_extend.h"
#include "m4_extend.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "FindSimplePath.h"
#include <algorithm>
#include <random>
#define totalRandFunc 9
extern std::vector<myStreetIntersection> streetIntersectionData;
extern std::vector<myStreetSegment> mySegData;

// Random number seeds
std::random_device rd;   // create a random device for seeding the generator
std::mt19937 g(rd());    // create a Mersenne Twister engine and seed it with the random device

// This routine takes in a vector of D deliveries (pickUp, dropOff
// intersection pairs), another vector of N intersections that
// are legal start and end points for the path (depots), and a turn 
// penalty in seconds (see m3.h for details on turn penalties).
//
// The first vector 'deliveries' gives the delivery information.  Each delivery
// in this vector has pickUp and dropOff intersection ids.
// A delivery can only be dropped-off after the associated item has been picked-up. 
// 
// The second vector 'depots' gives the intersection ids of courier company
// depots containing trucks; you start at any one of these depots and end at
// any one of the depots.
//
// This routine returns a vector of CourierSubPath objects that form a delivery route.
// The CourierSubPath is as defined above. The first street segment id in the
// first subpath is connected to a depot intersection, and the last street
// segment id of the last subpath also connects to a depot intersection.
// A package will not be dropped off if you haven't picked it up yet.
//
// The start_intersection of each subpath in the returned vector should be 
// at least one of the following (a pick-up and/or drop-off can only happen at 
// the start_intersection of a CourierSubPath object):
//      1- A start depot.
//      2- A pick-up location
//      3- A drop-off location. 
//
// You can assume that D is always at least one and N is always at least one
// (i.e. both input vectors are non-empty).
//
// It is legal for the same intersection to appear multiple times in the pickUp
// or dropOff list (e.g. you might have two deliveries with a pickUp
// intersection id of #50). The same intersection can also appear as both a
// pickUp location and a dropOff location.
//        
// If you have two pickUps to make at an intersection, traversing the
// intersection once is sufficient to pick up both packages. Additionally, 
// one traversal of an intersection is sufficient to drop off all the 
// (already picked up) packages that need to be dropped off at that intersection.
//
// Depots will never appear as pickUp or dropOff locations for deliveries.
//  
// If no valid route to make *all* the deliveries exists, this routine must
// return an empty (size == 0) vector.
std::vector<CourierSubPath>
travelingCourier(const std::vector<DeliveryInf>& deliveries,
                 const std::vector<IntersectionIdx>& depots, const float turn_penalty){ 
    // std::cout << "Input param: " << "size of d: " << deliveries.size() << " size of D: " << depots.size() << std::endl;
    // for(int i = 0; i< deliveries.size(); i++){
    //     std::cout << deliveries[i].pickUp << ", " << deliveries[i].dropOff << std::endl;
    // }
    // for(int j=0; j<depots.size(); j++){
    //     std::cout << depots[j] << std::endl;
    // }
    auto startTime = std::chrono::high_resolution_clock::now();
    std::vector<CourierSubPath> result;
    // special case: no delivery or depot, not necessary
    if(deliveries.empty() || depots.empty()){ return result;}

    // get numbers of inputs
    int numOfDeliv = deliveries.size();
    int numOfDepot = depots.size();
    int matrixSize = numOfDeliv * 2 + numOfDepot; // 2*deliveries + Depot

    // store all intersection ids to a vector for matrix forming
    // also create a map from intersection ids to their index in the matrix
    std::vector<IntersectionIdx> courierIntersections;
    courierIntersections.resize(matrixSize);
    std::multimap<IntersectionIdx,int> intersectionsToIndex;
    for(int i = 0; i<numOfDeliv; i++){
        courierIntersections[2*i] = deliveries[i].pickUp;
        courierIntersections[2*i+1] = deliveries[i].dropOff;
        intersectionsToIndex.insert(std::make_pair(deliveries[i].pickUp, 2*i));
        intersectionsToIndex.insert(std::make_pair(deliveries[i].dropOff, 2*i+1));
    }
    for(int i = 0; i<numOfDepot; i++){
        courierIntersections[numOfDeliv*2 + i] = depots[i];
    }

    // global vector for path backtracing
    std::vector<double> travelTimeMatrix;
    std::vector<CourierSubPath> subPathMatrix;
    // must resize the matrices before loading
    travelTimeMatrix.resize(matrixSize*matrixSize);
    subPathMatrix.resize(matrixSize*matrixSize);
    // Load the matrices here
    bool loadSuccess = LoadMatrix(travelTimeMatrix, subPathMatrix, matrixSize, courierIntersections, turn_penalty);
    if(!loadSuccess){
        std::cout << "Milestone 4 travelingCourier: Dijkstra search detects intersection with no valid outgoing edge."
        << std::endl;
    }
    auto curTime = std::chrono::high_resolution_clock::now();
    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    // std::cout << "Milestone 4 Pre-computing: Pre-computing took:  " << wallClock.count() << " seconds." << std::endl;
    std::cout << "Milestone 4 Pre-computing: End of pre-computing; Time consumed (accumulative):  " << wallClock.count() << " seconds." << std::endl;
    
    /*
    END of PRE-COMPUTING
    */

    // MAJOR PART STARTS HERE:
    // totalTravelTime will have size of numOfDeliv (num of pick up points) and will store
    // the corresponding total travel time found by starting @ that pick up point
    // indexMatrix will be of size numOfDeliv * (~2*numOfDeliv):
    // note the col of the indexMatrix can be less than 2*numOfDeliv as it's possible
    // to pickup or dropoff more than one parcel @ certain point
    // each row will be the path (in index) found by greedy starting @ that pick up point
    // auto greedyStartTime = std::chrono::high_resolution_clock::now();
    std::vector<double> totalTravelTime(numOfDeliv, 0.0);
    std::vector<std::vector<int>> indexMatrix;
    indexMatrix.resize(numOfDeliv);

    // traverse through all starting point and use them as the starting point;
    for(int pickup_id = 0; pickup_id<numOfDeliv; pickup_id++){
        std::vector<bool> picked(numOfDeliv, false);
        std::vector<bool> dropped(numOfDeliv, false);
        std::vector<bool> legalNext;
        legalNext.resize(numOfDeliv*2);
        // update next legal intersection list here: all pickup legal and dropoff illegal at the beginning
        for(int j=0; j < legalNext.size(); j++){
            if(j%2 == 0){legalNext[j] = true;}
            else {legalNext[j] = false;}
        }
        // initialize the starting point
        int cur_index = pickup_id*2;
        bool nextExist = true;
        while(nextExist){
            int cur_inter = courierIntersections[cur_index];
            if(cur_index%2 == 0){
                auto range = intersectionsToIndex.equal_range(cur_inter);
                for(auto kv_itr = range.first; kv_itr != range.second; ++kv_itr) {
                    int temp_index = kv_itr->second; 
                    if((temp_index & 0x1) == 0){
                        picked[temp_index/2] = true;
                        legalNext[temp_index] = false;
                        legalNext[temp_index+ 1] = true;
                    }
                }
            } else {
                auto range = intersectionsToIndex.equal_range(cur_inter);
                for(auto kv_itr = range.first; kv_itr != range.second; ++kv_itr) {
                    int temp_index = kv_itr->second; 
                    if((temp_index & 0x1) == 1){
                        dropped[temp_index / 2] = true;
                        legalNext[temp_index] = false;
                    }
                }
            }
            // check for the next legal intersection
            int next_index = -1;
            double closestDis = INT_MAX;
            nextExist = false;
            for(int id = 0; id < legalNext.size(); id++){
                if(!legalNext[id] || (travelTimeMatrix[cur_index * matrixSize + id] == -1)){continue;}
                if(travelTimeMatrix[cur_index * matrixSize + id] < closestDis){
                    nextExist = true;
                    closestDis = travelTimeMatrix[cur_index * matrixSize + id];
                    next_index = id;
                }
            }
            indexMatrix[pickup_id].emplace_back(cur_index); 
            if(next_index != -1){
                totalTravelTime[pickup_id] += travelTimeMatrix[cur_index * matrixSize + next_index];
            }
            cur_index = next_index;
        }
        // check if all parcels are delivered (check if the solution exist)
        for(int k=0; k<numOfDeliv; k++){
            if(!dropped[k]){
                std::cout << "Milestone 4 travelingCourier: Starting index "
                << pickup_id*2 << " exists greedy with no valid path. " << std::endl;
                totalTravelTime[pickup_id] = INT_MAX;
                indexMatrix[pickup_id].clear();
                break;
            }
        }
    }

    // traverse through the travel time vector to find the starting point (pickup)
    // with the least greedy travelling time and take the corresponding path
    int greedyBestIndex = 0;
    double greedyBestTime = INT_MAX;
    for(int i=0; i<numOfDeliv; i++){
        if(totalTravelTime[i] < greedyBestTime){
            greedyBestIndex = i;
            greedyBestTime = totalTravelTime[i];
        }
    }
    std::vector<int> greedyBestPathIndices = indexMatrix[greedyBestIndex];
    indexMatrix.clear();
    totalTravelTime.clear();

    curTime = std::chrono::high_resolution_clock::now();
    // wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - greedyStartTime);
    // std::cout << "Milestone 4 Greedy algorithm: Greedy algorithm took: " << wallClock.count() << " seconds." << std::endl;
    wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    std::cout << "Milestone 4 Greedy algorithm: End of greedy algorithm; Time consumed (accumulative): " << wallClock.count() << " seconds." << std::endl;
    
    /*
    END OF COMPUTING PICKUP AND DROPOFF (END OF GREEDY)
    */


    auto randomizationStartTime = std::chrono::high_resolution_clock::now();

    std::vector<int> globalBestPath = greedyBestPathIndices;
    double globalBestTime = greedyBestTime;
    double weight[6] = {1, 1, 1, 1, 5, 5};
    int numOfupdates = 0;
    // double acceptanceRate = 0.0;
    std::cout << "Time before iterative: " << globalBestTime << std::endl;
    for(int i=0; i<3500000; ++i){
        bool isGood = pathOptimizer(globalBestPath, globalBestTime, deliveries, travelTimeMatrix, matrixSize, intersectionsToIndex, weight, 6);
        if(isGood){++numOfupdates;}
    }
    // weight[0] = 1; weight[1] = 1; weight[2] = 1;
    // weight[3] = 1; weight[4] = 1; weight[5] = 1; weight[6] = 1;
    // for(int i=0; i<400000; ++i){
    //     bool isGood = pathOptimizer(globalBestPath, globalBestTime, deliveries, travelTimeMatrix, matrixSize, intersectionsToIndex, weight, 9);
    //     if(isGood){++numOfupdates;}
    // }
    
    curTime = std::chrono::high_resolution_clock::now();
    wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - randomizationStartTime);
    std::cout << "Milestone 4 Iterative optimization: Iterative optimization took: " << wallClock.count() << " seconds." << std::endl;
    wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    std::cout << "Milestone 4 Iterative optimization: End of local optimization; Time consumed (accumulative): " << wallClock.count() << " seconds." << std::endl;
    std::cout << "Time after iterative: " << globalBestTime << " NumOfUpdate accepted: " << numOfupdates << std::endl;

    /*
    END OF LOCAL RANDOMIZATION
    */

    auto SAStartTime = std::chrono::high_resolution_clock::now();  
    globalBestPath = Simulated_Annealing(globalBestPath, globalBestTime, deliveries, travelTimeMatrix, matrixSize, intersectionsToIndex, startTime);

    double temp_time = timeFromPath(globalBestPath, deliveries, travelTimeMatrix, matrixSize, intersectionsToIndex);
    std::cout << "Time after SA: " << temp_time << std::endl;

    curTime = std::chrono::high_resolution_clock::now();
    wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - SAStartTime);
    std::cout << "Milestone 4 travelingCourier: Simulated Annealing took: " << wallClock.count() << " seconds." << std::endl;
    wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    std::cout << "Milestone 4 travelingCourier: End of Simulated Annealing; Time consumed (accumulative): " << wallClock.count() << " seconds." << std::endl;

    result = resultFromPath(globalBestPath, numOfDeliv, numOfDepot, travelTimeMatrix, subPathMatrix);

    /*
    END OF FUNCTION
    */
    
    curTime = std::chrono::high_resolution_clock::now();
    wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    std::cout << "Milestone 4 travelingCourier: Result generated; Time consumed (accumulative): " << wallClock.count() << " seconds.\n" << std::endl;
    return result;
}

bool LoadMatrix(std::vector<double>&travelTimeMatrix, std::vector<CourierSubPath>& subPathMatrix,
                int matrixSize, const std::vector<IntersectionIdx>& inters, const float turn_penalty){
    // std::cout << "Size of matrix (by input): " << matrixSize << std::endl;
    // std::cout << "Size of matrix (by matrix): " << travelTimeMatrix.size() << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for 
    for(int i = 0; i<matrixSize; i++){
        // call Dij for each intersection
        bool pathFound;
        std::vector<myNode> myNodesVec(getNumIntersections(), myNode());
        IntersectionIdx src = inters[i];
        pathFound = MultiDestDijkstraSearch(myNodesVec, src, inters, turn_penalty);
        if(!pathFound){
            std::cout << "Milestone 4 LoadMatrix: intersection with no valid outgoing edge found." << std::endl;
        }
        // record the path and the travel time
        // record all pairs for i->0, i->i ..., i->matrixSize
        for(int j = 0; j<matrixSize; j++){
            IntersectionIdx target = inters[j];
            if(target == src){
                travelTimeMatrix[i*matrixSize + j] = -1;
                subPathMatrix[i*matrixSize + j].start_intersection = -1;
                subPathMatrix[i*matrixSize + j].end_intersection = -1;
                subPathMatrix[i*matrixSize + j].subpath = {-1};
            } else if (myNodesVec[target].fromEdge == -2){
                // special case: not path found between two intersections
                travelTimeMatrix[i*matrixSize + j] = INT_MAX;
                subPathMatrix[i*matrixSize + j].start_intersection = -1;
                subPathMatrix[i*matrixSize + j].end_intersection = -1;
                subPathMatrix[i*matrixSize + j].subpath = {-1};
            } else {
                // general case: create corresponding element to put into the matrix
                std::vector<StreetSegmentIdx> tempPath;
                StreetSegmentInfo SSInfo_;
                IntersectionIdx cur_inter = target;
                // Add path to vector in the reverse order
                while(myNodesVec[cur_inter].fromEdge != -1) {
                    tempPath.emplace_back(myNodesVec[cur_inter].fromEdge);
                    SSInfo_ = mySegData[myNodesVec[cur_inter].fromEdge];
                    cur_inter = (SSInfo_.from == cur_inter) ? SSInfo_.to : SSInfo_.from;
                }
                std::reverse(tempPath.begin(), tempPath.end());
                double tempTravelTime = computePathTravelTime(tempPath, turn_penalty);
                // store to the matrix
                travelTimeMatrix[i*matrixSize + j] = tempTravelTime;
                subPathMatrix[i*matrixSize + j].start_intersection = src;
                subPathMatrix[i*matrixSize + j].end_intersection = target;
                subPathMatrix[i*matrixSize + j].subpath = tempPath;
                tempPath.clear();
            }
        }
        myNodesVec.clear();
    }
    auto curTime = std::chrono::high_resolution_clock::now();
    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    // std::cout << "Milestone 4 LoadMatrix: Pre-computing (loading matrices) took " << wallClock.count() << " seconds." << std::endl;
    // std::cout << "Here is the travel time matrix: " << std::endl;
    // std::cout << "{ ";
    // for(int i=0; i<matrixSize; i++){
    //     std::cout << "{ ";
    //     for(int j=0; j<matrixSize; j++){
    //         std::cout << travelTimeMatrix[i*matrixSize + j] << ", ";
    //     }
    //     std::cout << "}" << std::endl;
    // }
    // std::cout << " }" << std::endl;
    return true;
}

bool MultiDestDijkstraSearch(
        std::vector<myNode>& myNodesVec,
        const IntersectionIdx src,
        const std::vector<IntersectionIdx>& dest,
        const double turn_penalty) {
    std::vector<IntersectionIdx> targets = dest;
    targets.erase(std::remove(targets.begin(), targets.end(), src), targets.end());
    myNodesVec[src].fromEdge = -1;
    // Priority_Queue for Dijkstra optimization (min heap)
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> myPathQueue;

    // initialize the starting point (src), root's fromEdge is -1
    myPathQueue.emplace(DijkstraNode(src, -1, 0.0));

    // start searching
    while(!myPathQueue.empty()) {
        // if(targets.empty()){
        //     std::cout << "Dij end!" << std::endl;
        //     return true;
        // }
        DijkstraNode cur_node = myPathQueue.top();
        myPathQueue.pop();
        int cur_node_id = cur_node.nodeID;
        myNode listNode = myNodesVec[cur_node_id];
        if(listNode.isClosed){continue;}
        if(cur_node.travelTime < listNode.bestTime) {
            myNode updatedNode(cur_node.edgeFrom, cur_node.travelTime, true);
            myNodesVec[cur_node_id] = updatedNode;
            targets.erase(std::remove(targets.begin(), targets.end(), cur_node_id), targets.end());
            if(targets.empty()){
                return true;
            }

            // add adjacent nodes & edges to priority queue
            for(outgoingEdge edge : streetIntersectionData[cur_node_id].outgoingEdges) {
                IntersectionIdx node_id = edge.inter_id;
                if(myNodesVec[node_id].isClosed){continue;}
                StreetSegmentIdx ss_id = edge.ss_id;
                myStreetSegment next_ss_info = mySegData[ss_id];
                // check for street segment st cul-de-sacs (dead end)
                if(next_ss_info.from == next_ss_info.to){ continue; }
                // create myPathSeg, check for turn penalty, and push to priority queue
                double next_node_time = myNodesVec[cur_node_id].bestTime + next_ss_info.segTime;
                if(isTurning(ss_id, cur_node.edgeFrom)){next_node_time += turn_penalty;}
                myPathQueue.emplace(DijkstraNode(node_id, ss_id, next_node_time));
            }
        }
    }
    return false;
}

//
bool pathOptimizer(std::vector<int>& bestPath, double& bestTime,
                   const std::vector<DeliveryInf>& deliveries,
                   const std::vector<double>& travelTimeMatrix, int matrixSize,
                   const std::multimap<IntersectionIdx,int>& intersectionsToIndex,
                   double weight[], int size) {
    int randFuncNum = weightedRandom(size, weight);
    std::vector<int> possiblePath;
    switch (randFuncNum) {
        case 0:
            possiblePath = partialShuffle(bestPath, true);  // reverse
            break;
        case 1:
            possiblePath = partialShuffle(bestPath, false); // random
            break;
        case 2:
            possiblePath = segmentShift(bestPath, true, true);  // segment shift left
            break;
        case 3:
            possiblePath = segmentShift(bestPath, true, false); // segment shift right
            break;
        case 4:
            possiblePath = segmentShift(bestPath, false, true); // or opt left
            break;
        case 5:
            possiblePath = segmentShift(bestPath, true, false); // or opt right
            break;
        default:
            printf("Hit default case in random optimization selector.\n");
    }
    double possiblePathTime = timeFromPath(possiblePath, deliveries, travelTimeMatrix, matrixSize, intersectionsToIndex);
    if(possiblePathTime < bestTime && possiblePathTime != -1){
        bestPath.clear();
        bestPath = possiblePath;
        bestTime = possiblePathTime;
        return true;
    }
    return false;
}

int weightedRandom(int n, double weights[]) {
    // Calculate the total weight
    double weightSum = 0.0;
    for (int i = 0; i < n; i++) {
        weightSum += weights[i];
    }

    // Generate a random number between 0 and the total weight
    std::uniform_real_distribution<double> dist(0.0, weightSum);
    double r = dist(g);

    // Select the integer based on its probability weight
    double weightCumulative = 0.0;
    for (int i = 0; i < n; i++) {
        weightCumulative += weights[i];
        if (r <= weightCumulative) {
            return i;
        }
    }

    // Should never get here, but just in case...
    return -1;
}

// reverse shuffle + random shuffle
std::vector<int> partialShuffle(std::vector<int> path, bool reverse_or_random) {
    std::uniform_int_distribution<int> dist(0, path.size() - 1);
    int startIdx = dist(rd), endIdx = dist(rd);
    while (startIdx == endIdx) { endIdx = dist(rd); }
    if (startIdx > endIdx) std::swap(startIdx, endIdx);
    // true for reverse shuffle
    if (reverse_or_random)
        std::reverse(path.begin() + startIdx, path.begin() + endIdx + 1);
        // false for random shuffle
    else
        std::shuffle(path.begin() + startIdx, path.begin() + endIdx + 1, g);
    return path;
}

bool nodeSwap(std::vector<int>& path, double& time, const std::vector<double>& travelTimeMatrix, int matrixSize) {
    if(path.size() <= 2){
        return false;
    }
    std::uniform_int_distribution<int> dist1(1, path.size() / 2 - 1);
    std::uniform_int_distribution<int> dist2(path.size() / 2 , path.size() - 1);
    int randNum1, randNum2, firstIdx, secondIdx;
    int counter = 0;
    randNum1 = dist1(rd);
    randNum2 = dist2(rd);
    firstIdx = path[randNum1];
    secondIdx = path[randNum2];
    while(firstIdx % 2 == 0){
        randNum1 = dist1(rd);
        firstIdx = path[randNum1];
        counter++;
        if(counter > path.size()/2 + 1){
            return false;
        }
    }
    counter = 0;
    while(secondIdx % 2 == 1){
        randNum2 = dist2(rd);
        secondIdx = path[randNum2];
        counter++;
        if(counter > path.size()/2 + 1){
            return false;
        }
    }
    if(randNum1 == 0){return false;}
    if(randNum2 == path.size()-1){return false;}
    int firstBefore = path[randNum1 - 1];
    int firstAfter = path[randNum1 + 1];
    int secondBefore = path[randNum2 - 1];
    int secondAfter = path[randNum2 + 1];
    double firstLinkBefore, firstLinkAfter, secondLinkBefore, secondLinkAfter;
    firstLinkBefore = travelTimeMatrix[firstBefore * matrixSize + secondIdx];
    firstLinkAfter = travelTimeMatrix[secondIdx * matrixSize + firstAfter];
    secondLinkBefore = travelTimeMatrix[secondBefore * matrixSize + firstIdx];
    secondLinkAfter = travelTimeMatrix[firstIdx * matrixSize + secondAfter];
    if(firstLinkBefore == -1 || firstLinkBefore == INT_MAX){return false;}
    if(firstLinkAfter == -1 || firstLinkAfter == INT_MAX){return false;}
    if(secondLinkBefore == -1 || secondLinkBefore == INT_MAX){return false;}
    if(secondLinkAfter == -1 || secondLinkAfter == INT_MAX){return false;}
    int newTime;
    std::vector<int> newPath = path;
    newPath[randNum1] = secondIdx;
    newPath[randNum2] = firstIdx;
    newTime = time + firstLinkBefore + firstLinkAfter + secondLinkBefore + secondLinkAfter;
    newTime = newTime - travelTimeMatrix[firstBefore * matrixSize + firstIdx]
                - travelTimeMatrix[firstIdx * matrixSize + firstAfter]
                - travelTimeMatrix[secondBefore * matrixSize + secondIdx]
                - travelTimeMatrix[secondIdx * matrixSize + secondAfter];
    if(newTime < time){
        time = newTime;
        path = newPath;
        return true;
    }
    return false;
}

// 2 segmentShifts + 2 or opts
std::vector<int> segmentShift(std::vector<int> path, bool shiftOneElement, bool rotateLeft) {
    // segment shift left equals to segment shift right, just differs by the segment length
    // here, only segment shift left is implemented
    int pathSize = path.size();
    std::uniform_int_distribution<int> dist(0, pathSize - 1);
    int startIdx = dist(rd);
    int endIdx = dist(rd) + 1;

    // check for corner cases
    while (startIdx == pathSize - 1) { startIdx = dist(rd); };
    while (startIdx == endIdx) { endIdx = dist(rd) + 1; }
    if (startIdx > endIdx) { std::swap(startIdx, endIdx); }
    if (startIdx == endIdx - 1) { std::swap(path[startIdx], path[endIdx]); return path; }

    // start, middle and end index are three different indices now
    int newFirstIdx;
    if (shiftOneElement) newFirstIdx = startIdx + 1;
    else newFirstIdx = startIdx + dist(rd) % (endIdx - startIdx - 1) + 1;
    if (rotateLeft) std::rotate(path.begin() + startIdx, path.begin() + newFirstIdx, path.begin() + endIdx);
    else std::rotate(path.rbegin() + startIdx, path.rbegin() + newFirstIdx, path.rbegin() + endIdx);
    return path;
}

double timeFromPath(const std::vector<int>& path, const std::vector<DeliveryInf>& deliveries,
                    const std::vector<double>& travelTimeMatrix, int matrixSize,
                    const std::multimap<IntersectionIdx,int>& intersectionsToIndex){
    int numOfDeliv = deliveries.size();
    std::vector<bool> picked(numOfDeliv, false);
    std::vector<bool> dropped(numOfDeliv, false);
    int droppedNum = 0, pickedNum = 0;
    double travelTime = 0.0;
    // handle simple special case: the first index is a drop off point
    // Or: the last index is a pick up point
    if((path.front() & 0x1) == 1 || (path.back() & 0x1) == 0){
        return -1;
    }
    for(int path_idx = 0; path_idx<path.size(); ++path_idx){
        if(droppedNum > pickedNum){
            // std::cout << "return: more drop than pick" << std::endl;
            return -1;
        }
        int cur_index = path[path_idx];
        IntersectionIdx cur_inter;
        if((cur_index & 0x1) == 0){ cur_inter = deliveries[cur_index/2].pickUp; }
        else { cur_inter = deliveries[(cur_index - 1)/2].dropOff; }
        auto range = intersectionsToIndex.equal_range(cur_inter);
        for(auto kv_itr = range.first; kv_itr != range.second; ++kv_itr) {
            int temp_index = kv_itr->second; 
            if((temp_index & 0x1) == 0){
                if(!picked[temp_index/2]){
                    picked[temp_index/2] = true;
                    pickedNum++;
                }
            } else {
                // if(picked[temp_index/2] == false){return -1;}
                if(picked[(temp_index - 1)/2] && !dropped[(temp_index - 1)/2]){
                    dropped[(temp_index - 1)/2] = true;
                    droppedNum++;
                } 
            }
        }
        if(path_idx != path.size() - 1){
            double newTravelTime = travelTimeMatrix[path[path_idx] * matrixSize + path[path_idx+1]];
            if(newTravelTime == -1 || newTravelTime == INT_MAX){
                return -1;
            } else {
                travelTime += newTravelTime;
            }
        }
    }
    // some parcels are not delivered
    if(droppedNum != numOfDeliv) {
        // std::cout << "Not all parcel deliveried. " << std::endl;
        // std::cout << "Dropped: " << droppedNum << " numOfDeliv: " << numOfDeliv << std::endl;
        return -1;
    }
    // std::cout << "return here" << std::endl;
    // std::cout << travelTime << std::endl;
    return travelTime;
}

std::vector<CourierSubPath> resultFromPath(const std::vector<int>& path, int numOfDeliv, int numOfDepot,
                                           const std::vector<double>& travelTimeMatrix,
                                           const std::vector<CourierSubPath>& subPathMatrix){
    // add staring depot, ending depot, and form the result here:
    // Add the starting depot:
    int temp_index = -1;
    double bestTime = INT_MAX;
    int matrixSize = numOfDeliv * 2 + numOfDepot;
    std::vector<CourierSubPath> result;
    for(int i=0; i<numOfDepot; i++){
        double newBestTime = travelTimeMatrix[(i + 2*numOfDeliv) * matrixSize + path.front()];
        if(newBestTime == -1 || newBestTime == INT_MAX){continue;}
        if(newBestTime < bestTime){
            bestTime = newBestTime;
            temp_index = i + 2*numOfDeliv;
        }
    }
    result.emplace_back(subPathMatrix[temp_index * matrixSize + path[0]]);
    // Add path btw pickups and dropoffs:
    for(int i=0; i<path.size()-1; i++){
        result.emplace_back(subPathMatrix[path[i] * matrixSize + path[i+1]]);
    }
    // Add path to the returning depot
    temp_index = -1;
    bestTime = INT_MAX;
    int last_index = path.back();
    for(int i=0; i<numOfDepot; i++){
        double newBestTime = travelTimeMatrix[last_index * matrixSize + i + 2*numOfDeliv];
        if(newBestTime == -1 || newBestTime == INT_MAX){continue;}
        if(newBestTime < bestTime){
            bestTime = newBestTime;
            temp_index = i + 2*numOfDeliv;
        }
    }
    result.emplace_back(subPathMatrix[last_index * matrixSize + temp_index]);
    return result;
}

std::vector<int> Simulated_Annealing(std::vector<int> initPath, double initTime, 
                                     const std::vector<DeliveryInf>& deliveries, std::vector<double>& travelTimeMatrix, 
                                     int matrixSize, const std::multimap<IntersectionIdx,int>& intersectionsToIndex,
                                     std::chrono::high_resolution_clock::time_point StartTime){
    double temperature = 2000; // init temp
    double end_Temp = 1e-20; // end temp  
    if(initPath.size()<10){
        temperature = 2000; // init temp
        end_Temp = 11; // end temp    
    }
    double dTemp = 0.9995;
    int count_temp = 0;
    
    std::vector<int> currentPath = initPath;
    std::vector<int> bestPath = initPath;
    double currentPathTime = initTime;
    double bestPathTime = initTime;

    // int pathSize = currentPath.size();
    
    // TODO: Do we need to set a maximum iteration?
    int max_iteration = 150; // change to time count?
    // double weight[9] = { 3, 3, 1, 1, 1, 3, 3, 5, 5};
    double weight[6] = {1, 1, 1, 1, 3, 3};
    while (temperature > end_Temp){
        count_temp++;
        currentPath = bestPath;
        currentPathTime = bestPathTime;
        for(int iteration = 0; iteration < max_iteration; iteration ++){
            double newPathTime = -1;
            std::vector<int> newPath;
            do{
                int randFuncNum = weightedRandom(6, weight);
                // std::uniform_int_distribution<int> dist(0, currentPath.size() - 1);
                // int num1, num2, temp;
                newPath.clear();
                switch (randFuncNum) {
                    case 0:
                        newPath = partialShuffle(currentPath, true);  // reverse
                        break;
                    case 1:
                        newPath = partialShuffle(currentPath, false); // random
                        break;
                    case 2:
                        newPath = segmentShift(currentPath, true, true);  // segment shift left
                        break;
                    case 3:
                        newPath = segmentShift(currentPath, true, false); // segment shift right
                        break;
                    case 4:
                        newPath = segmentShift(currentPath, false, true); // or opt left
                        break;
                    case 5:
                        newPath = segmentShift(currentPath, true, false); // or opt right
                        break;
                    default:
                        printf("Hit default case in random optimization selector.\n");
                }
                newPathTime = timeFromPath(newPath, deliveries, travelTimeMatrix, matrixSize, intersectionsToIndex);
            } while(newPathTime == -1);
            
            double acceptanceProbability = exp((currentPathTime - newPathTime) / temperature);
            double randomProbability = ((double) rand() / (RAND_MAX)); // need to modify?

            if ((newPathTime < currentPathTime) || (randomProbability < acceptanceProbability)){
                currentPathTime = newPathTime;
                currentPath = newPath;
                if(currentPathTime < bestPathTime){
                    bestPath = currentPath;
                    bestPathTime = currentPathTime;
                }
            }
        }
        temperature *= dTemp;
        auto CurTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>> (CurTime - StartTime);
        // if(temperature<0.1){
        //     break;
        // }
        if(duration.count() > 49.95){
            std::cout << "Return due to time constraint; Num of temperature iterations: " << count_temp 
            << " Ending temp: " << temperature << std::endl;
            return bestPath;
        }
    } 
    // auto CurTime = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::duration<double>> (CurTime - StartTime);
    // double newweight[9] = { 3, 3, 1, 1, 1, 3, 3, 3, 3};
    // int counterddddd = 0;
    // while(duration.count() < 49.95){
    //     CurTime = std::chrono::high_resolution_clock::now();
    //     duration = std::chrono::duration_cast<std::chrono::duration<double>> (CurTime - StartTime);
    //     pathOptimizer(bestPath, bestPathTime, deliveries, travelTimeMatrix, matrixSize, intersectionsToIndex, newweight, 9);
    //     counterddddd++;
    // }
    std::cout << "Return at the end of SA; Num of temperature iterations: " << count_temp << std::endl;
    return bestPath;
}
