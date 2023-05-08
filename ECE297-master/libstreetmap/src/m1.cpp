/*
 * Copyright 2023 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated
 * documentation files (the "Software") in course work at the University
 * of Toronto, or for personal use. Other uses are prohibited, in
 * particular the distribution of the Software either publicly or to third
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include <cctype>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <vector>
#include <unordered_set>
#include <string>
#include <utility>
#include <map>
#include <limits>
#include "m1.h"
#include "m1_extend.h"
#include "m3_extend.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <ctime>

// All global variables goes here:
myMap current_map;
std::vector<myStreetSegment> mySegData;
std::vector<myStreetIntersection> streetIntersectionData;
std::vector<StreetInfo> mapStreet;
std::map<OSMID, std::map<std::string, std::string>> tagPairOfOSMNodeIdx;
std::map<OSMID, std::map<std::string, std::string>> tagPairOfOSMWayIdx;
std::vector<myFeatureData> myFeatures;
Trie* trieMapOfStreet;
std::map<std::string, std::string> mapNames;

// extern ezgl::surface *restaurant_png;
// extern ezgl::surface *regular_png;
// extern ezgl::surface *school_png;
// extern ezgl::surface *health_care_png;
// extern ezgl::surface *transit_png;

ezgl::surface *restaurant_png = NULL;
ezgl::surface *regular_png = NULL;
ezgl::surface *school_png = NULL;
ezgl::surface *health_care_png = NULL;
ezgl::surface *transit_png = NULL;

// global variables for m2.cpp:
// Two set of color scheme for day and night mode
// first 11 codes are used for features [0:10]
// 12th is most common streets [11], 13th is for motorway streets [12]
// 14th is for trunk streets [13], 15th is for primary streets [14]
// 16th is for secondary streets [15], 17th is background  [16]
std::vector<std::vector<int>> color_list_light;
    // {{0xFF, 0xFF, 0xE7}, {0xC8, 0xFA, 0xCC}, {0xFF, 0xF1, 0xBA}, {0xAA, 0xD3, 0xDF}, 
    // {0xAA, 0xD3, 0xDF}, {0xF2, 0xEF, 0xE9}, {0xD7, 0xD0, 0xCA}, {0xAD, 0xD1, 0x9E},
    // {0xDE, 0xF6, 0xC0}, {0xAA, 0xD3, 0xDF}, {0xDD, 0xEC, 0xEC}, {255, 255, 255},
    // {220, 151, 162}, {238, 182, 159}, {245, 216, 169}, {247, 250, 196},
    // {241, 239, 233}};
std::vector<std::vector<int>> color_list_dark;
    // {{63, 76, 99}, {41, 96, 91}, {110, 112, 100}, {40, 53, 118}, 
    // {40, 53, 118}, {37, 87, 88}, {71, 89, 117}, {39, 91, 91},
    // {46, 108, 83}, {40, 53, 118}, {103, 131, 165}, {85, 96, 115},
    // {103, 113, 137}, {101, 110, 131}, {80, 90, 109}, {80, 90, 109},
    // {59, 71, 91}};
// active color scheme
std::vector<std::vector<int>> color_list;

// Util function for inserting a new element into an ascending vector, O(logn)
template<typename T>
void insert_sorted_noDupe(T &new_ele, std::vector<T> &cur_vector){
    // Standard Binary Search algorithm to find the index of the
    // largest element smaller or equal to the new_ele in the vector
    int left = 0;
    int right = cur_vector.size()-1;
    int result = -1;
    while (left <= right) {
        // update mid point at each loop
        int mid = (left + right) / 2;
        if (cur_vector[mid] <= new_ele) {
            result = mid;
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    // check if the vector is empty
    if(result == -1){
        cur_vector.insert(cur_vector.begin(), new_ele);
    } else if(cur_vector[result] == new_ele){
        // check if the element already exists
        return;
    } else if(result == cur_vector.size()){
        // check if the new_element is the larger than all elements in the vector
        cur_vector.emplace_back(new_ele);
    } else {
        // general case, insert the new_ele next to the result found
        cur_vector.insert(cur_vector.begin()+result+1, new_ele);
    }
}

// Util function for finding common member in two sorted vector, O(n+m)
template<typename T>
std::vector<T> find_common_member_in_sorted(std::vector<T> &first, std::vector<T> &second){
    // If either vector is empty, return empty vector
    if(first.empty()&&second.empty()){
        return std::vector<T> ();
    } else {
        int size_f = first.size();
        int size_s = second.size();
        int index_f = 0;
        int index_s = 0;
        std::vector<T> result(0);
        // iterate through the list from the beginning
        while(index_f < size_f && index_s < size_s){
            // if the element in the first vector is smaller, iterator for the 1st vector++
            if(first[index_f]<second[index_s]){
                index_f++;
                continue;
            }
            // if the element in the first vector is larger, iterator for the 2nd vector++
            else if(first[index_f]>second[index_s]){
                index_s++;
                continue;
            }
            // push back the common element to the result
            result.emplace_back(first[index_f]);
            index_f++, index_s++;
        }
        return result;
    }
}
// Constructor and destructor for TrieNode:
TrieNode::TrieNode(){
    is_end_of_word = false;
    id = {};
}
// Destructor will interate through all children to call their destructor
TrieNode::~TrieNode(){
    if(children.empty()){
        return;
    }
    for(auto iterator = children.begin(); iterator != children.end(); iterator++){
        delete iterator->second;
    }
}
// Constructor and destructor for Trie tree
Trie::Trie() {
    root = new TrieNode();
}
Trie::~Trie() {
    delete root;
}
// Insert a word (street name here), the ID of the street
// will be stored to all nodes the word passes through
void Trie::insert(std::string word, int id) {
    TrieNode *current = root;
    for (char c : word) {
        if (current->children.find(c) == current->children.end()) {
            current->children[c] = new TrieNode();
        }
        current = current->children[c];
        insert_sorted_noDupe(id, current->id);
    }
    current->is_end_of_word = true;
    insert_sorted_noDupe(id, current->id);
}
// Search for a specific word / first few characters of a word
// return all possible results in a vector
std::vector<int> Trie::search(std::string word) {
    TrieNode *current = root;
    for (char c : word) {
        if (current->children.find(c) == current->children.end()) {
            return {};
        }
        current = current->children[c];
    }
    return current->id;
}

// load map names
void loadMapNames(){
    mapNames.insert({"beijing","beijing_china"});
    mapNames.insert({"cairo","cairo_egypt"});
    mapNames.insert({"capetown","cape-town_south-africa"});
    mapNames.insert({"goldenhorseshoe","golden-horseshoe_canada"});
    mapNames.insert({"hamilton","hamilton_canada"});
    mapNames.insert({"hongkong","hong-kong_china"});
    mapNames.insert({"iceland","iceland"});
    mapNames.insert({"interlaken","interlaken_switzerland"});
    mapNames.insert({"kyiv","kyiv_ukraine"});
    mapNames.insert({"london","london_england"});
    mapNames.insert({"newdelhi","new-delhi_india"});
    mapNames.insert({"newyork","new-york_usa"});
    mapNames.insert({"riodejaneiro","rio-de-janeiro_brazil"});
    mapNames.insert({"sainthelena","saint-helena"});
    mapNames.insert({"singapore","singapore"});
    mapNames.insert({"sydney","sydney_australia"});
    mapNames.insert({"tehran","tehran_iran"});
    mapNames.insert({"tokyo","tokyo_japan"});
    mapNames.insert({"toronto","toronto_canada"});
}
// helper function comparing two myFeatureData type variabl based on feature area
bool compareTwoFeatures(const myFeatureData &feature1, const myFeatureData &feature2){
    return feature1.featureArea>feature2.featureArea;
}

// load feature data
void loadFeatureData(){
    // check if the info has already been loaded
    if(!myFeatures.empty()){
        return;
    }
    int numFeatures = getNumFeatures();
    myFeatures.resize(numFeatures);
    // traverse through all features
    double minx_, maxx_, miny_, maxy_;
    for(int feature_id = 0; feature_id < numFeatures; feature_id++){
        myFeatureData this_feature_;
        int numOfPoints = getNumFeaturePoints(feature_id);
        this_feature_.feature_id = feature_id;
        this_feature_.numOfPoints = numOfPoints;
        this_feature_.feature_type = getFeatureType(feature_id);

        minx_ = x_from_lon(getFeaturePoint(feature_id, 0).longitude());
        miny_ = y_from_lat(getFeaturePoint(feature_id, 0).latitude());
        maxx_ = minx_;
        maxy_ = miny_;
        // simple case for features with only one point
        if(numOfPoints == 1){
            this_feature_.minx_ = minx_;
            this_feature_.maxx_ = maxx_;
            this_feature_.miny_ = miny_;
            this_feature_.maxy_ = maxy_;
            this_feature_.featureArea = 0;
            this_feature_.openFeature = true;
            ezgl::point2d cur_point(minx_, miny_);
            this_feature_.point_list.emplace_back(cur_point);
            myFeatures[feature_id] = this_feature_;
            // myFeatures.emplace_back(this_feature_);
            continue;
        }
        // general case
        std::vector<ezgl::point2d> point_list;
        for (int i = 0; i < numOfPoints; i++) {
            // update max and min lat and lon respectively
            ezgl::point2d cur_point;
            cur_point.x = x_from_lon(getFeaturePoint(feature_id, i).longitude());
            cur_point.y = y_from_lat(getFeaturePoint(feature_id, i).latitude());
            point_list.emplace_back(cur_point);
            minx_ = std::min(minx_, cur_point.x);
            maxx_ = std::max(maxx_, cur_point.x);
            miny_ = std::min(miny_, cur_point.y);
            maxy_ = std::max(maxy_, cur_point.y);
        }
        this_feature_.minx_ = minx_;
        this_feature_.maxx_ = maxx_;
        this_feature_.miny_ = miny_;
        this_feature_.maxy_ = maxy_;
        this_feature_.point_list = point_list;
        this_feature_.featureArea = findFeatureArea(feature_id);
        this_feature_.openFeature = !((getFeaturePoint(feature_id, 0).longitude() == getFeaturePoint(feature_id, numOfPoints-1).longitude()) &&
                                    (getFeaturePoint(feature_id, 0).latitude() == getFeaturePoint(feature_id, numOfPoints-1).latitude()));
        myFeatures[feature_id] = this_feature_;
        //myFeatures.emplace_back(this_feature_);
        continue;
    }
    // sort all features based on feature area descending
    std::sort(myFeatures.begin(), myFeatures.end(), compareTwoFeatures);
}
// load tag pair of OSM ways
void loadOSMWayTagPair(){
    // check if the OSM info is already loaded
    if(!tagPairOfOSMWayIdx.empty()){
        return;
    }
    int numOfWay = getNumberOfWays();
    // travers through all OSMWays linearly
    for(int way_index=0; way_index<numOfWay; way_index++){
        const OSMWay* tempWay = getWayByIndex(way_index);
        OSMID tempID = tempWay->id();
        // create a map<OSMID, map> for each way with its OSMID
        std::map<std::string, std::string> tempTagPairs;
        tagPairOfOSMWayIdx[tempID] = tempTagPairs;

        // setup a map<string, string> for all <key, value> pairs the way has
        for(int item = 0; item<getTagCount(tempWay); item++) {
           std::string key,value;
           std::tie(key,value) = getTagPair(tempWay,item);
           tagPairOfOSMWayIdx[tempID][key] = value;
        }
    }
}
// load tag pair of OSM nodes
void loadOSMNodeTagPair(){
    // check if the OSM info is already loaded
    if(!tagPairOfOSMNodeIdx.empty()){
        return;
    }
    int numOfNode = getNumberOfNodes();
    // travers through all OSMNodes linearly
    for(int node_index=0; node_index<numOfNode; node_index++){
        const OSMNode* tempNode = getNodeByIndex(node_index);
        OSMID tempID = tempNode->id();
        // create a map<OSMID, map> for each node with its OSMID
        std::map<std::string, std::string> tempTagPairs;
        tagPairOfOSMNodeIdx[tempID] = tempTagPairs;

        // setup a map<string, string> for all <key, value> pairs the node has
        for(int item = 0; item<getTagCount(tempNode); item++) {
           std::string key,value;
           std::tie(key,value) = getTagPair(tempNode,item);
           tagPairOfOSMNodeIdx[tempID][key] = value;
        }
    }
}
// traverse through all street segments to load various info
void loadStreetSegment() {
    // check if the info has already been loaded
    if(!mapStreet.empty()){
        return;
    }
    int numStreetSegment = getNumStreetSegments();
    int numStreet = getNumStreets();
    // resize and initialize global variables this function is storing to
    mySegData.resize(numStreetSegment);
    mapStreet.resize(numStreet);
    ezgl::point2d posFrom, posTo;
    std::string tag;
    for (int street_segment_id = 0; street_segment_id < numStreetSegment; street_segment_id++) {
        // Extract basic information from mapStreet segments
        StreetSegmentInfo SSInfo_ = getStreetSegmentInfo(street_segment_id);
        StreetIdx street_id = SSInfo_.streetID;
        std::string name_ = getStreetName(street_id);
        IntersectionIdx from_ = SSInfo_.from;
        IntersectionIdx to_ = SSInfo_.to;
        LatLon previousPos = getIntersectionPosition(from_),
                finalPos = getIntersectionPosition(to_);
        // Use simple x,y than class if too slow
        posFrom.x = x_from_lon(previousPos.longitude());
        posFrom.y = y_from_lat(previousPos.latitude());
        posTo.x = x_from_lon(finalPos.longitude());
        posTo.y = y_from_lat(finalPos.latitude());
        mySegData[street_segment_id].posFrom = posFrom;
        mySegData[street_segment_id].posTo = posTo;

        // preprocess street name to be all lower case and remove spaces
        boost::to_lower(name_);
        name_.erase(std::remove_if(name_.begin(), name_.end(), ::isspace), name_.end());
        trieMapOfStreet->insert(name_, street_id);

        // find the length of the street segment
        double distance = 0;
        for (int j = 0; j < SSInfo_.numCurvePoints; j++) {
            LatLon currentPos = getStreetSegmentCurvePoint(street_segment_id, j);
            distance += findDistanceBetweenTwoPoints(previousPos, currentPos);
            previousPos = currentPos;
        }
        distance += findDistanceBetweenTwoPoints(previousPos, finalPos);

        // store intersection id from_ and to_ to the street containing the street segment
        insert_sorted_noDupe(from_, mapStreet[street_id].myIntersections);
        insert_sorted_noDupe(to_, mapStreet[street_id].myIntersections);

        // add the length of the street segment to the corresponding street
        mapStreet[street_id].length = mapStreet[street_id].length + distance;
        // store all relative infomation of the street segment:
        mySegData[street_segment_id].segLength = distance;
        mySegData[street_segment_id].streetID = street_id;
        mySegData[street_segment_id].segTime = distance / SSInfo_.speedLimit;
        mySegData[street_segment_id].from = from_;
        mySegData[street_segment_id].to = to_;
        mySegData[street_segment_id].numCurvePoints = SSInfo_.numCurvePoints;
        mySegData[street_segment_id].oneWay = SSInfo_.oneWay;
        mySegData[street_segment_id].speedLimit = SSInfo_.speedLimit;
        // store street level and weight
        tag = getOSMWayTagValue(SSInfo_.wayOSMID, "highway");
        mySegData[street_segment_id].street_tag = tag;
        if(tag == "motorway"){
            mySegData[street_segment_id].street_level = 12;
            mySegData[street_segment_id].weight = 4;
            current_map.motorway_street_seg.emplace_back(street_segment_id);
        }
        else if(tag == "trunk"){
            mySegData[street_segment_id].street_level = 13;
            mySegData[street_segment_id].weight = 3;
            current_map.trunk_street_seg.emplace_back(street_segment_id);
        }
        else if(tag == "primary"){
            mySegData[street_segment_id].street_level = 14;
            mySegData[street_segment_id].weight = 3;
            current_map.primary_street_seg.emplace_back(street_segment_id);
        }
        else if(tag == "secondary"){
            mySegData[street_segment_id].street_level = 15;
            mySegData[street_segment_id].weight = 2; 
            current_map.secondary_street_seg.emplace_back(street_segment_id);
        }
        else {
            mySegData[street_segment_id].street_level = 11;
            mySegData[street_segment_id].weight = 1;
            current_map.general_street_seg.emplace_back(street_segment_id);
        }
    }
}

// load (pos, name) and calculate (street segments, adjacent intersections) intersection data
void loadIntersections() {
    // check if the intersection info has been loaded
    if(!streetIntersectionData.empty()){
        return;
    }
    int numTotalIntersecs = getNumIntersections();
    streetIntersectionData.resize(numTotalIntersecs);
    for (int index = 0; index < numTotalIntersecs; index++) {
        // load street segments of each intersection
        LatLon pos = getIntersectionPosition(index);
        double lat = pos.latitude(), lon = pos.longitude();
        streetIntersectionData[index].latitude = lat;
        streetIntersectionData[index].longitude = lon;
        streetIntersectionData[index].intersecName = getIntersectionName(index);
        streetIntersectionData[index].xy_loc.x = x_from_lon(lon);
        streetIntersectionData[index].xy_loc.y = y_from_lat(lat);
        int numAtIntersection = getNumIntersectionStreetSegment(index);
        // store street segments of an intersection to streetIntersectionData
        // store adjacent intersections of an intersection to streetIntersectionData
        std::unordered_set<IntersectionIdx> tempAdjIntersecs;
        for(int i=0; i<numAtIntersection; i++){
            StreetSegmentIdx ss_id = getIntersectionStreetSegment(index, i);
            streetIntersectionData[index].street_segments_of_intersections.emplace_back(ss_id);
            if(getStreetSegmentInfo(ss_id).from == index){
                outgoingEdge temp;
                temp.ss_id = ss_id;
                temp.inter_id = getStreetSegmentInfo(ss_id).to;
                streetIntersectionData[index].outgoingEdges.emplace_back(temp);
                tempAdjIntersecs.emplace(temp.inter_id);
            } else if (!getStreetSegmentInfo(ss_id).oneWay){
                outgoingEdge temp;
                temp.ss_id = ss_id;
                temp.inter_id = getStreetSegmentInfo(ss_id).from;
                streetIntersectionData[index].outgoingEdges.emplace_back(temp);
                tempAdjIntersecs.emplace(temp.inter_id);
            }
        }
        std::vector<IntersectionIdx> tempAdjVec(tempAdjIntersecs.begin(), tempAdjIntersecs.end());
        streetIntersectionData[index].adjIntersecs = tempAdjVec;
    }
}

// load basic information of the map
void loadMapInfo() {
    current_map.maxLat = -INT_MAX;
    current_map.maxLon = -INT_MAX;
    current_map.minLat = INT_MAX;
    current_map.minLon = INT_MAX;
    current_map.latAvg = -INT_MAX;
    current_map.mapArea = -INT_MAX;
    double lat, lon;
    for (int index = 0; index < getNumIntersections(); ++index) {
        // load street segments of each intersection
        LatLon pos = getIntersectionPosition(index);
        lat = pos.latitude();
        lon = pos.longitude();
        // find map bounds
        current_map.maxLat = current_map.maxLat < lat ? lat : current_map.maxLat;
        current_map.maxLon = current_map.maxLon < lon ? lon : current_map.maxLon;
        current_map.minLat = current_map.minLat > lat ? lat : current_map.minLat;
        current_map.minLon = current_map.minLon > lon ? lon : current_map.minLon;
    }
    // must find latAvg first here
    current_map.latAvg = (current_map.minLat + current_map.maxLat) / 2;
    current_map.maxX = x_from_lon(current_map.maxLon);
    current_map.maxY = y_from_lat(current_map.maxLat);
    current_map.minX = x_from_lon(current_map.minLon);
    current_map.minY = y_from_lat(current_map.minLat);
    current_map.mapArea = abs((current_map.maxX - current_map.minX)*(current_map.maxY - current_map.minY));
    current_map.startPoint = -1;
    current_map.endPoint = -1;
    current_map.pathSegment.clear();
    current_map.cur_POI.clear();
    current_map.dark_mode = false;
    current_map.findingRange = false;
    current_map.pathFindingMode = false;
    current_map.total_range = 0.0;
    current_map.global_turn_penalty = 0.0;
    current_map.travelInstructions = "No valid path. Please use Path Mode to generate a path.";
}
float x_from_lon(float lon) { return kEarthRadiusInMeters * kDegreeToRadian * lon * cos(current_map.latAvg * kDegreeToRadian); }
float y_from_lat(float lat) { return kEarthRadiusInMeters * kDegreeToRadian * lat; }
float lon_from_x(float x) { return x / kEarthRadiusInMeters / kDegreeToRadian / cos(current_map.latAvg * kDegreeToRadian); }
float lat_from_y(float y) { return y / kEarthRadiusInMeters / kDegreeToRadian; }

bool loadMap(std::string map_streets_database_filename) {
    // for benchmarking, test the time to load map
    auto startTime = std::chrono::high_resolution_clock::now();
    bool load_successful = false; //Indicates whether the map has loaded successfully
    // preprocess filename given and generate the osm and street version of it respectively
    std::string streets_database_filename = map_streets_database_filename;
    std::string osm_database_filename = map_streets_database_filename;
    int index = osm_database_filename.find("streets");
    // a valid filename must contain streets, otherwise invalid and return
    if(index == -1){
        return false;
    }
    osm_database_filename.erase(index, 7);
    osm_database_filename.insert(index, "osm");
    // Start loading
    std::cout << "Loading mapStreet database: " << streets_database_filename << std::endl;
    std::cout << "Loading osm: " << osm_database_filename << std::endl;

    // load datasets and available functions
    bool street_load_successful = loadStreetsDatabaseBIN(map_streets_database_filename);
    bool osm_load_successful = loadOSMDatabaseBIN(osm_database_filename);
    // must loadMapInfo at first here as it compute most basic properties of the map
    // load streetSegments before intersections (loadIntersections will use information from loadStreetSegment)
    trieMapOfStreet = new Trie();
    loadMapInfo();
    loadOSMNodeTagPair();
    loadOSMWayTagPair();
    loadMapNames();

    loadStreetSegment();
    loadIntersections();

    loadFeatureData();
    if(restaurant_png == NULL){
        restaurant_png = ezgl::renderer::load_png("libstreetmap/resources/restaurant.png");
        regular_png = ezgl::renderer::load_png("libstreetmap/resources/regular.png");
        school_png = ezgl::renderer::load_png("libstreetmap/resources/school.png");
        health_care_png = ezgl::renderer::load_png("libstreetmap/resources/healthcare1.png");
        transit_png = ezgl::renderer::load_png("libstreetmap/resources/transit.png");
    }
    color_list_light = {{0xFF, 0xFF, 0xE7}, {0xC8, 0xFA, 0xCC}, {0xFF, 0xF1, 0xBA}, {0xAA, 0xD3, 0xDF}, 
                        {0xAA, 0xD3, 0xDF}, {0xF2, 0xEF, 0xE9}, {0xD7, 0xD0, 0xCA}, {0xAD, 0xD1, 0x9E},
                        {0xDE, 0xF6, 0xC0}, {0xAA, 0xD3, 0xDF}, {0xDD, 0xEC, 0xEC}, {255, 255, 255},
                        {220, 151, 162}, {238, 182, 159}, {245, 216, 169}, {247, 250, 196},
                        {241, 239, 233}};
    color_list_dark = {{63, 76, 99}, {41, 96, 91}, {110, 112, 100}, {40, 53, 118}, 
                        {40, 53, 118}, {37, 87, 88}, {71, 89, 117}, {39, 91, 91},
                        {46, 108, 83}, {40, 53, 118}, {103, 131, 165}, {85, 96, 115},
                        {103, 113, 137}, {101, 110, 131}, {80, 90, 109}, {80, 90, 109},
                        {59, 71, 91}};
    color_list = {{0xFF, 0xFF, 0xE7}, {0xC8, 0xFA, 0xCC}, {0xFF, 0xF1, 0xBA}, {0xAA, 0xD3, 0xDF}, 
                        {0xAA, 0xD3, 0xDF}, {0xF2, 0xEF, 0xE9}, {0xD7, 0xD0, 0xCA}, {0xAD, 0xD1, 0x9E},
                        {0xDE, 0xF6, 0xC0}, {0xAA, 0xD3, 0xDF}, {0xDD, 0xEC, 0xEC}, {255, 255, 255},
                        {220, 151, 162}, {238, 182, 159}, {245, 216, 169}, {247, 250, 196},
                        {241, 239, 233}};
    
    // load successful if both street and osm are loaded
    load_successful = street_load_successful && osm_load_successful;

    auto curTime = std::chrono::high_resolution_clock::now();
    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    std::cout << "Milestone 1: Loading map took " << wallClock.count() << " seconds." << std::endl;

    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    std::cout << "Closing map\n";
    if(restaurant_png != NULL){
        ezgl::renderer::free_surface(restaurant_png);
        ezgl::renderer::free_surface(regular_png);
        ezgl::renderer::free_surface(school_png);
        ezgl::renderer::free_surface(health_care_png);
        ezgl::renderer::free_surface(transit_png);
        restaurant_png = NULL;
        regular_png = NULL;
        school_png = NULL;
        health_care_png = NULL;
        transit_png = NULL;
    }
    mySegData.clear();
    streetIntersectionData.clear();
    mapStreet.clear();
    tagPairOfOSMNodeIdx.clear();
    tagPairOfOSMWayIdx.clear();
    myFeatures.clear();
    mapNames.clear();
    color_list_light.clear();
    color_list_dark.clear();
    color_list.clear();
    delete trieMapOfStreet;
    // reset some global variables (not necessary)
    current_map.maxLat = -INT_MAX, current_map.maxLon = -INT_MAX;
    current_map.minLat = INT_MAX, current_map.minLon = INT_MAX;
    current_map.latAvg = -INT_MAX; current_map.mapArea = -INT_MAX;
    current_map.startPoint = -1; current_map.endPoint = -1;
    current_map.dark_mode = false; current_map.findingRange = false;
    current_map.pathFindingMode = false;
    current_map.scale_factor = 1000;
    current_map.total_range = 0.0;
    current_map.global_turn_penalty = 0.0;
    current_map.current_map_name = "ERROR CURRENT MAP NAME";
    current_map.currentCity = "ERROR CURRENT CITY";
    current_map.general_street_seg.clear();
    current_map.motorway_street_seg.clear();
    current_map.trunk_street_seg.clear();
    current_map.primary_street_seg.clear();
    current_map.secondary_street_seg.clear();
    current_map.cur_POI.clear();
    current_map.pathSegment.clear();
    current_map.highlighted_inter.clear();
    current_map.range_finder_point_list.clear();
    current_map.street_pieces_list.clear();
    // close street and osm datasets
    closeStreetDatabase();
    closeOSMDatabase();
}

// Returns the distance between two (latitude,longitude) coordinates in meters
double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2) {
    // (x, y) = (R · lon · cos(latAvg), R · lat)
    double lon1 = point_1.longitude() * kDegreeToRadian,
           lat1 = point_1.latitude() * kDegreeToRadian;
    double lon2 = point_2.longitude() * kDegreeToRadian,
           lat2 = point_2.latitude() * kDegreeToRadian;
    double lat_avg = (lat1 + lat2) / 2;
    double x1 = kEarthRadiusInMeters * lon1 * cos(lat_avg),
           y1 = kEarthRadiusInMeters * lat1;
    double x2 = kEarthRadiusInMeters * lon2 * cos(lat_avg),
           y2 = kEarthRadiusInMeters * lat2;
    return sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
}

// Returns the length of the given street segment in meters
double findStreetSegmentLength(StreetSegmentIdx street_segment_id) {
    return mySegData[street_segment_id].segLength;
}

// Returns the travel time to drive from one end of a street segment
// to the other, in seconds, when driving at the speed limit
// Note: (time = distance/speed_limit)
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
    return mySegData[street_segment_id].segTime;
}

// Returns all intersections reachable by traveling down one street segment
// from the given intersection (hint: you can't travel the wrong way on a
// 1-way street)
// the returned vector should NOT contain duplicate intersections
// Corner case: cul-de-sacs can connect an intersection to itself
// (from and to intersection on street segment are the same). In that case
// include the intersection in the returned vector (no special handling needed).
// Speed Requirement --> high
std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id) {
    return streetIntersectionData[intersection_id].adjIntersecs;
}

// Returns the geographically nearest intersection (i.e. as the crow flies) to
// the given position
IntersectionIdx findClosestIntersection(LatLon my_position) {
    // do a linear search through every intersection
    // calculate each distance between my_position
    // return the idx of the smallest distance
    int n = getNumIntersections();
    IntersectionIdx minIdx = 0;
    double minDistance = INT_MAX;
    for (int i = 0; i < n; i++) {
        double curDistance = findDistanceBetweenTwoPoints(getIntersectionPosition(i), my_position);
        if (curDistance < minDistance) {
            minDistance = curDistance;
            minIdx = i;
        }
    }
    return minIdx;
}

// Returns the street segments that connect to the given intersection
std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id) {
    return streetIntersectionData[intersection_id].street_segments_of_intersections;
}

// Returns all intersections along a given street.
std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id) {
    return mapStreet[street_id].myIntersections;
}

// Return all intersection ids at which the two given streets intersect
std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(StreetIdx street_id1, StreetIdx street_id2) {
    // both intersection lists are sorted and with no duplicates
    std::vector<int> inter_id1 =  mapStreet[street_id1].myIntersections;
    std::vector<int> inter_id2 =  mapStreet[street_id2].myIntersections;
    return find_common_member_in_sorted(inter_id1, inter_id2);
}

// Returns all street ids corresponding to street names that start with the
// given prefix
std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix) {
    // lower all characters and remove spaces
    street_prefix.erase(std::remove_if(street_prefix.begin(), street_prefix.end(), ::isspace),
        street_prefix.end());
    boost::to_lower(street_prefix);
    return trieMapOfStreet->search(street_prefix);
}

// The length of the street is adding up all the streetsegment length with the same streetID.
double findStreetLength(StreetIdx street_id) {
    return mapStreet[street_id].length;
}

// The function is to return the nearest point of interest of the given type to the given position
POIIdx findClosestPOI(LatLon my_position, std::string POItype) {
    POIIdx closestPOI = 0; 
    //initialize the cloestPOI_distance to be a maximum value.
    double closestPOI_distance = std::numeric_limits<double>::max(); 
    //Traverse
    //let distanceBetweTwoPoints = closestPOI of the 1st loop; continue loop(to all POIs), if the distance is shorter than closest, replace.
    for (int i = 0; i < getNumPointsOfInterest() ; i++){

        if (POItype == getPOIType(POIIdx(i))){
            //return the nearest point of interset (type)
            LatLon point_2 = getPOIPosition(POIIdx(i));
            double DistanceBetweenTwoPoints = findDistanceBetweenTwoPoints(my_position, point_2);

            //compare and return
            if (DistanceBetweenTwoPoints < closestPOI_distance) {
                closestPOI_distance = DistanceBetweenTwoPoints;
                closestPOI = POIIdx(i);
            }
        }
    }
    return closestPOI;
}

// Using shoelace functionto find the 2D feature area, no edge case need to concern due to its algorithm.
double shoelaceFormula(const std::vector<std::pair<double, double>> &points) {
    int numberOfVertices = points.size(); //n = numberOfVertices
    double polygonArea = 0.0; // initialize polygonArea
    int PerviousVertices = numberOfVertices - 1; //j = perviousVertives
    for (int currentVertices = 0; currentVertices < numberOfVertices; currentVertices++) { // i = currentVertices 
        polygonArea += (points[PerviousVertices].first + points[currentVertices].first) * (points[PerviousVertices].second - points[currentVertices].second);
        PerviousVertices = currentVertices;
    }
    return polygonArea; // Note: this need to divide by 2.
}

// The function is to find the feature area of the polygon. (holes are not concered)
// Corner Case: check if the feature is close or not -> not close: return 0;
double findFeatureArea(FeatureIdx feature_id) {
    int numFeaturePts = getNumFeaturePoints(feature_id);
    // Check if the polygon is closed
    if (getFeaturePoint(feature_id,0).latitude() != getFeaturePoint(feature_id, numFeaturePts - 1).latitude() ||
        getFeaturePoint(feature_id,0).longitude() != getFeaturePoint(feature_id, numFeaturePts - 1).longitude()) {
        return 0;
    }
    
    // Use lat_max and lat_min to Calculate average latitude
    double lat_min = getFeaturePoint(feature_id,0).latitude();
    double lat_max = lat_min;
    for (int i = 0; i < numFeaturePts; i++) {
        double lat_current = getFeaturePoint(feature_id, i).latitude();
        lat_min = std::min(lat_min, lat_current);
        lat_max = std::max(lat_max, lat_current);
        
    }
    double lat_avg = ((lat_max + lat_min) / 2);    

    // Convert lat and lon to x and y in meters
    std::vector<std::pair<double, double>> points;
    for (int i = 0; i < numFeaturePts; i++) {
        double LontoX = (kEarthRadiusInMeters * (getFeaturePoint(feature_id, i).longitude() * cos(lat_avg * kDegreeToRadian) * kDegreeToRadian));
        double LattoY = (kEarthRadiusInMeters * (getFeaturePoint(feature_id, i).latitude() * kDegreeToRadian));
        points.emplace_back(LontoX,LattoY);
    }
    // Calculate the area using the shoelace formula
    double polygonArea = shoelaceFormula(points);
    // Using the shoelace function to find area could be neg depends on the direction of it traverse -> fabs.
    return fabs(polygonArea/2);   
}


// Return the value associated with this key on the specified OSMNode.
std::string getOSMNodeTagValue (OSMID OSMid, std::string key) {
    // return empty string if no node with 'OSMid' found
    if(tagPairOfOSMNodeIdx.find(OSMid)==tagPairOfOSMNodeIdx.end()){
        return "";
    } else {
        // return empty string if the node doesn't have a value with tag 'key'
        std::map<std::string, std::string> pairOfNode = tagPairOfOSMNodeIdx[OSMid];
        if(pairOfNode.find(key)==pairOfNode.end()){
            return "";
        }
        return pairOfNode[key];
    }
}

// Return the value associated with this key on the specified OSMWay.
std::string getOSMWayTagValue (OSMID OSMid, std::string key) {
    // return empty string if no node with 'OSMid' found
    if(tagPairOfOSMWayIdx.find(OSMid)==tagPairOfOSMWayIdx.end()){
        return "";
    } else {
        // return empty string if the node doesn't have a value with tag 'key'
        std::map<std::string, std::string> pairOfNode = tagPairOfOSMWayIdx[OSMid];
        if(pairOfNode.find(key)==pairOfNode.end()){
            return "";
        }
        return pairOfNode[key];
    }
}