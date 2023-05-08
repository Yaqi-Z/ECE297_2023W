#include "m1.h"
#include "m3.h"
#include "m1_extend.h"
#include "m2_extend.h"
#include "m3_extend.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"

extern std::vector<myStreetIntersection> streetIntersectionData;
extern std::vector<myStreetSegment> mySegData;
extern myMap current_map;

// Returns the time required to travel along the path specified, in seconds.
// The path is given as a vector of street segment ids, and this function can
// assume the vector either forms a legal path or has size == 0.  The travel
// time is the sum of the length/speed-limit of each street segment, plus the
// given turn_penalty (in seconds) per turn implied by the path.  If there is
// no turn, then there is no penalty. Note that whenever the street id changes
// (e.g. going from Bloor Street West to Bloor Street East) we have a turn.
double computePathTravelTime(const std::vector<StreetSegmentIdx>& path,
                             const double turn_penalty) {
    int path_size = path.size();
    double total_TravelTime_inSecond = 0;
    // check if input vector is empty
    if (path_size == 0) { return 0; }
    // corner case: only one street segment
    else if (path_size == 1) {
        total_TravelTime_inSecond = findStreetSegmentTravelTime(path[0]); 
        return total_TravelTime_inSecond;
    } else {
        // general case: traverse through al street segment and compute total travel time
        myStreetSegment ss_info = mySegData[path[0]];
        myStreetSegment prev_ss_info = mySegData[path[0]];
        total_TravelTime_inSecond += ss_info.segTime;
        for (int cur_seg = 1; cur_seg < path_size; cur_seg++) {
            ss_info = mySegData[path[cur_seg]];
            prev_ss_info = mySegData[path[cur_seg-1]];
            total_TravelTime_inSecond += ss_info.segTime;
            //check if there is a turn, compare the current one with pre
            if (ss_info.streetID != prev_ss_info.streetID){
                total_TravelTime_inSecond += turn_penalty;
            }
        }
        return total_TravelTime_inSecond;
    }
}

// Returns a path (route) between the start intersection (intersect_id.first)
// and the destination intersection (intersect_id.second), if one exists. 
// This routine should return the shortest path
// between the given intersections, where the time penalty to turn right or
// left is given by turn_penalty (in seconds).  If no path exists, this routine
// returns an empty (size == 0) vector.  If more than one path exists, the path
// with the shortest travel time is returned. The path is returned as a vector
// of street segment ids; traversing these street segments, in the returned
// order, would take one from the start to the destination intersection.
std::vector<StreetSegmentIdx> findPathBetweenIntersections(
        const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids,
        const double turn_penalty) {
    // ERROR msg if give invalid input intersections
    std::vector<StreetSegmentIdx> resPath;
    if(intersect_ids.first == intersect_ids.second){ return resPath; }
    if(intersect_ids.first > streetIntersectionData.size()){
        std::cout << "ERROE: Starting point id passed into searching function is invalid." << std::endl;
        return resPath;
    } else if(intersect_ids.second > streetIntersectionData.size()){
        std::cout << "ERROE: Ending point id passed into searching function is invalid." << std::endl;
        return resPath;
    }
    // initialize closed list for back tracking
    std::vector<myNode> myNodesVec(getNumIntersections(), myNode());
    // Use Astar search to find the route
    bool pathFound = AStarSearch(myNodesVec, intersect_ids, turn_penalty);
    // bool pathFound = DijkstraSearch(myNodesVec, intersect_ids, turn_penalty);
    if(pathFound) {
        IntersectionIdx cur_node = intersect_ids.second;
        StreetSegmentInfo SSInfo_;
        // Add path to vector in the reverse order
        while(myNodesVec[cur_node].fromEdge != -1) {
            resPath.emplace_back(myNodesVec[cur_node].fromEdge);
            SSInfo_ = mySegData[myNodesVec[cur_node].fromEdge];
            cur_node = (SSInfo_.from == cur_node) ? SSInfo_.to : SSInfo_.from;
        }
        std::reverse(resPath.begin(), resPath.end());
    } else {
        std::cout << "No path found: (fromEdge, bestTime, isClosed) = (";
        std::cout << myNodesVec[intersect_ids.second].fromEdge << ", ";
        std::cout << myNodesVec[intersect_ids.second].bestTime << ", ";
        std::cout << myNodesVec[intersect_ids.second].isClosed << ")" << std::endl;
    }

    // Can use M3 tester for MultiDestDijkstraSearch:

    // bool pathFound = MultiDestDijkstraSearch(myNodesVec, intersect_ids.first, {intersect_ids.second}, turn_penalty);
    // if(pathFound) {
    //     IntersectionIdx cur_node = intersect_ids.second;
    //     StreetSegmentInfo SSInfo_;
    //     // Add path to vector in the reverse order
    //     while(myNodesVec[cur_node].fromEdge != -1) {
    //         resPath.emplace_back(myNodesVec[cur_node].fromEdge);
    //         SSInfo_ = mySegData[myNodesVec[cur_node].fromEdge];
    //         cur_node = (SSInfo_.from == cur_node) ? SSInfo_.to : SSInfo_.from;
    //     }
    //     std::reverse(resPath.begin(), resPath.end());
    // } else {
    //     std::cout << "No path found: (fromEdge, bestTime, isClosed) = (";
    //     std::cout << myNodesVec[intersect_ids.second].fromEdge << ", ";
    //     std::cout << myNodesVec[intersect_ids.second].bestTime << ", ";
    //     std::cout << myNodesVec[intersect_ids.second].isClosed << ")" << std::endl;
    // }
    return resPath;
}

bool DijkstraSearch(
        std::vector<myNode>& myNodesVec,
        const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids,
        const double turn_penalty) {
    // Priority_Queue for Dijkstra optimization (min heap)
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> myPathQueue;
    IntersectionIdx inter_from = intersect_ids.first;
    IntersectionIdx inter_to = intersect_ids.second;
    bool pathFound = false;
    // corner case: from == to
    if(inter_from == inter_to) { return true; }
    // initialize the starting point (src), root's fromEdge is -1
    // Will put all adjacent nodes of the src to the heap
    myPathQueue.emplace(DijkstraNode(inter_from, -1, 0.0));

    // start searching
    while(!myPathQueue.empty()) {
        DijkstraNode cur_node = myPathQueue.top();
        myPathQueue.pop();
        int cur_node_id = cur_node.nodeID;
        myNode listNode = myNodesVec[cur_node_id];
        if(listNode.isClosed){continue;}
        if(cur_node.travelTime < listNode.bestTime) {
            myNode updatedNode(cur_node.edgeFrom, cur_node.travelTime, true);
            // myNodesVec[cur_node_id].bestTime = cur_node.travelTime;
            // myNodesVec[cur_node_id].fromEdge = cur_node.edgeFrom;
            // myNodesVec[cur_node_id].isClosed = true;
            myNodesVec[cur_node_id] = updatedNode;
            if(cur_node_id == inter_to){ pathFound = true; return true;}

            // std::vector<outgoingEdge> outEdges = streetIntersectionData[cur_node_id].outgoingEdges;

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
    return pathFound;
}

bool AStarSearch(
        std::vector<myNode>& myNodesVec,
        const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids,
        const double turn_penalty) {
    // open list for A* optimization (min heap)
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openList;
    IntersectionIdx inter_from = intersect_ids.first;
    IntersectionIdx inter_to = intersect_ids.second;
    bool pathFound = false;
    // initialize the open list with the src point
    openList.emplace(AStarNode(inter_from, -1, 0.0, heuristicFunc(inter_from, inter_to)));
    // while the open list is not empty;
    while(!openList.empty()){
        // get the node with least f value
        AStarNode cur_node = openList.top();
        openList.pop();
        IntersectionIdx cur_node_id = cur_node.nodeID;
        // check if have visited
        myNode listNode = myNodesVec[cur_node_id];
        if(listNode.isClosed){continue;}
        // explore the node if it have better travel time
        if(cur_node.GTravelTime < listNode.bestTime){
            myNode updatedNode(cur_node.edgeFrom, cur_node.GTravelTime, true);
            myNodesVec[cur_node_id] = updatedNode;
            // myNodesVec[cur_node_id].bestTime = cur_node.GTravelTime;
            // myNodesVec[cur_node_id].fromEdge = cur_node.edgeFrom;
            // myNodesVec[cur_node_id].isClosed = true;
            // if reach the dest return
            if(cur_node_id == inter_to){pathFound = true;return true;}
            // std::vector<outgoingEdge> outEdges = streetIntersectionData[cur_node_id].outgoingEdges;
            // traverse through all children nodes
            for(outgoingEdge edge : streetIntersectionData[cur_node_id].outgoingEdges){
                // check visited first
                IntersectionIdx next_node_id = edge.inter_id;
                if(myNodesVec[next_node_id].isClosed){continue;}
                StreetSegmentIdx ss_id = edge.ss_id;
                myStreetSegment next_ss_info = mySegData[ss_id];
                // check for street segment st cul-de-sacs (dead end)
                if(next_ss_info.from == next_ss_info.to){continue;}
                // general cases come here:
                AStarNode next_node;
                next_node.nodeID = next_node_id;
                next_node.edgeFrom = ss_id;
                next_node.GTravelTime = cur_node.GTravelTime + next_ss_info.segTime;
                if(isTurning(ss_id, cur_node.edgeFrom)){next_node.GTravelTime += turn_penalty;}
                next_node.HEstTime = heuristicFunc(next_node_id, inter_to);
                next_node.FVal = next_node.GTravelTime + next_node.HEstTime;
                openList.emplace(next_node);
            }
        }
    }
    return pathFound;
}

std::string showPathDetails(const std::vector<StreetSegmentIdx>& path, const double turn_penalty){
    int path_size = path.size();
    std::string output_Msg;
    // corner case: no valid path
    if(path_size == 0){
        output_Msg += "NO VALID PATH: Please use Path Mode to generate a path. \n";
        return output_Msg;
    } else if(path_size == 1){
        // only one street segment
        output_Msg = "--------- Here Is Your Trip Instruction ---------\n\n";
        std::string street_name = getStreetName(mySegData[path[0]].streetID);
        output_Msg = output_Msg + "Take " + streetNameToPrint(street_name) + " to reach your destination.\n";
        output_Msg = output_Msg + "Total length: " + formatTotalDistance(mySegData[path[0]].segLength) + "\n";
        // use 2 * min travel time here, can use other factor
        output_Msg = output_Msg + "Expected travel time: " + formatTotalTime(mySegData[path[0]].segTime*3) + "\n";
        output_Msg += "\n--------- Have a great one!!! ---------";
        return output_Msg;
    } else {
        double travel_distance = 0;
        double total_travel_distance = 0;
        double travel_time = 0;
        double total_travel_time = 0;
        IntersectionIdx curNode, prevNode;
        StreetSegmentIdx curSeg, prevSeg;
        prevSeg = path[0];
        curSeg = path[1];
        prevNode = current_map.startPoint;
        curNode = (prevNode == mySegData[prevSeg].from)? mySegData[prevSeg].to : mySegData[prevSeg].from;
        output_Msg = "--------- Here Is Your Trip Instruction ---------\n\n";
        // initialize the first street segment here
        std::string street_name = getStreetName(mySegData[prevSeg].streetID);
        output_Msg = output_Msg + "Start your trip on " + streetNameToPrint(street_name);
        output_Msg = output_Msg + ". The road has a speed limit of ";
        output_Msg += formatToTenth((double)mySegData[prevSeg].speedLimit*3.6);
        output_Msg += " km/h. \n";
        travel_distance += mySegData[prevSeg].segLength;
        total_travel_distance  += mySegData[prevSeg].segLength;
        travel_time += mySegData[prevSeg].segTime;
        total_travel_time += mySegData[prevSeg].segTime;
        // traverse through the list to generate travel instruction
        for (int index = 1; index < path_size; index++){
            prevSeg = path[index-1];
            curSeg = path[index];
            // consider if still travelling on the same street
            if(mySegData[curSeg].streetID == mySegData[prevSeg].streetID){
                // update travelling time if on the same street
                prevNode = curNode;
                curNode = (curNode == mySegData[curSeg].from) ? mySegData[curSeg].to : mySegData[curSeg].from;
                travel_distance += mySegData[prevSeg].segLength;
                total_travel_distance  += mySegData[prevSeg].segLength;
                travel_time += mySegData[prevSeg].segTime;
                total_travel_time += mySegData[prevSeg].segTime;
                continue;
            } else {
                // update travelling time and print output msg if make a turn
                travel_time += mySegData[prevSeg].segTime;
                total_travel_time += turn_penalty;
                travel_distance += mySegData[prevSeg].segLength;
                output_Msg = output_Msg + "Travel along for " + formatTotalDistance(travel_distance) + ". ";
                output_Msg = output_Msg + "Estimated travel time: " + formatTotalTime(travel_time*3) + "\n\n";
                travel_distance = 0.0;
                travel_time = 0.0;
                total_travel_distance  += mySegData[prevSeg].segLength;
                total_travel_time += mySegData[prevSeg].segTime;
                // get turning informaiton
                IntersectionIdx nextNode = (curNode == mySegData[curSeg].from)? mySegData[curSeg].to : mySegData[curSeg].from;
                std::string turning_dir = getTurningDir(prevNode, curNode, nextNode);
                prevNode = curNode;
                // curNode = (curNode == mySegData[curSeg].from) ? mySegData[curSeg].to : mySegData[curSeg].from;
                curNode = nextNode;
                output_Msg = output_Msg + turning_dir;
                street_name = getStreetName(mySegData[curSeg].streetID);
                output_Msg += streetNameToPrint(street_name) ;
                output_Msg = output_Msg + ". The road has a speed limit of ";
                output_Msg += formatToTenth((double)mySegData[curSeg].speedLimit*3.6);
                output_Msg += " km/h. \n";
            }
        }
        // append the information for the last street segment
        travel_distance += mySegData[curSeg].segLength;
        total_travel_distance  += mySegData[curSeg].segLength;
        travel_time += mySegData[curSeg].segTime;
        total_travel_time += mySegData[curSeg].segTime;
        output_Msg = output_Msg + "Travel along for " + formatTotalDistance(travel_distance) + ". ";
        output_Msg = output_Msg + "Estimated travel time: " + formatTotalTime(travel_time*3) + "\n";
        output_Msg += "\n--------- Trip Overview ---------\n";
        output_Msg = output_Msg + "Total travel distance: " + formatTotalDistance(total_travel_distance) + ".\n";
        output_Msg = output_Msg + "The trip will take you " + formatTotalTime(total_travel_time*3) + "\n";
        output_Msg += "\n--------- Have a great one!!! ---------";
        return output_Msg;
    }
}

// the function will format distance information; display in meters for short
// distance, in kilometers for long distance
std::string formatTotalDistance(double inputNum){
    double outNum;
    std::string total_distance_msg;
    // consider corner case: 995m < input < 1000m
    if(inputNum >= 997.5){
        outNum = inputNum/1000;
        outNum = round(outNum*10)/10;
        total_distance_msg = std::to_string(outNum);
        total_distance_msg.erase(total_distance_msg.find_last_not_of('0') + 1, std::string::npos);
        if(total_distance_msg.back() == '.'){
            total_distance_msg.pop_back();
        }
        total_distance_msg += " km";
        return total_distance_msg;
    } else {
        outNum = inputNum;
        outNum = round(outNum);
        outNum = round(outNum/5)*5;
        total_distance_msg = std::to_string(outNum);
        total_distance_msg.erase(total_distance_msg.find_last_not_of('0') + 1, std::string::npos);
        if(total_distance_msg.back() == '.'){
            total_distance_msg.pop_back();
        }
        total_distance_msg += " m";
        return total_distance_msg;
    }
}

// format travel time
std::string formatTotalTime(double inputNum){
    std::string outMsg = "";
    // in hours and mins if the time is more than 1 hour
    if(inputNum >= 3600){
        int outputNum = inputNum;
        // neglect seconds in long trip
        outputNum /= 60;
        int mins, hours;
        mins = outputNum % 60;
        outputNum /= 60;
        hours = outputNum;
        outMsg = std::to_string(hours) + " hour(s) and " + std::to_string(mins) + " mins. ";
        return outMsg;
    } else if(inputNum >= 60){
        // in num of mins for 1-59 mins
        inputNum /= 60;
        double outputNum = round(inputNum*10)/10;
        outMsg = std::to_string(outputNum);
        outMsg.erase(outMsg.find_last_not_of('0') + 1, std::string::npos);
        if(outMsg.back() == '.'){
            outMsg.pop_back();
        }
        outMsg += " mins. ";
        return outMsg;
    } else {
        // less than one min
        outMsg = " < 1 min. ";
        return outMsg;
    }
}

// format street name
std::string streetNameToPrint(std::string& streetName){
    if(streetName == "<unknown>"){
        return "street with unknown name";
    } else {
        return streetName;
    }
}

// round a number to keep only one decimal
std::string formatToTenth(double inputNum){
    double outputNum = round(inputNum*10)/10;
    std::string numMsg = std::to_string(outputNum);
    numMsg.erase(numMsg.find_last_not_of('0') + 1, std::string::npos);
    if(numMsg.back() == '.'){
        numMsg.pop_back();
    }
    return numMsg;
}

// return turning instruction give three sequential points(IntersectionIdx)
std::string getTurningDir(IntersectionIdx prev, IntersectionIdx cur, IntersectionIdx next){
    ezgl::point2d vecA, vecB, temp;
    ezgl::point2d prevPoint, curPoint, nextPoint;
    double dotP, crossP, lengthP;
    prevPoint = streetIntersectionData[prev].xy_loc;
    curPoint = streetIntersectionData[cur].xy_loc;
    nextPoint = streetIntersectionData[next].xy_loc;
    vecA = curPoint - prevPoint;
    vecB = nextPoint - curPoint;
    temp = vecA*vecB;
    dotP = temp.x + temp.y;
    crossP = vecA.x*vecB.y - vecB.x*vecA.y;
    lengthP = sqrt(vecA.x*vecA.x + vecA.y*vecA.y)*sqrt(vecB.x*vecB.x + vecB.y*vecB.y);
    double angle;
    angle = dotP/lengthP;
    angle = acos(angle)*180/M_PI;
    std::string msg;
    if(angle > 170){
        return "Make a U-turn to ";
    } else if(angle > 110) {
        msg = "Make a sharp ";
        if(crossP > 0){ msg += "left turn to "; }
        else{ msg += "right turn to ";}
    } else if(angle > 70) {
        msg = "Make a ";
        if(crossP > 0){ msg += "left turn to "; }
        else{ msg += "right turn to ";}
    } else {
        msg = "Make a small turn to ";
    }
    return msg;
}

// heuristic function for AStar(use straight length)
double heuristicFunc(IntersectionIdx point_from, IntersectionIdx point_to){
    ezgl::point2d pos_from, pos_to;
    pos_from = streetIntersectionData[point_from].xy_loc;
    pos_to = streetIntersectionData[point_to].xy_loc;
    double x1, y1, x2, y2;
    x1 = pos_from.x;
    y1 = pos_from.y;
    x2 = pos_to.x;
    y2 = pos_to.y;
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1))/30;
}

// check if two street segment is turning
bool isTurning(StreetSegmentIdx ss1, StreetSegmentIdx ss2){
    if(ss1 < 0){return false;}
    if(ss2 < 0){return false;}
    StreetSegmentInfo ss1_info = mySegData[ss1];
    StreetSegmentInfo ss2_info = mySegData[ss2];
    if(ss1_info.streetID == ss2_info.streetID){return false;}
    else {return true;}
}