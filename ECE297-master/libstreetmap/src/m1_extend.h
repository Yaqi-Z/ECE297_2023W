//
// Created by lihanli3 on 2/11/23.
//

#ifndef MAPPER_M1_EXTEND_H
#define MAPPER_M1_EXTEND_H
#include <vector>
#include <map>
#include <set>
#include <unordered_set>
#include <string>
#include "ezgl/application.hpp"

// Function and class prototype goes here:
// structure definition:
struct street_piece_info {
    double width, length;
    StreetSegmentIdx street_segment_id;
    ezgl::point2d point_to, point_from;
};

struct StreetInfo {
    double length;
    std::vector<IntersectionIdx> myIntersections;
};

struct myMap {
    double maxLat, maxLon, minLat, minLon;
    double maxX, maxY, minX, minY;
    double latAvg, mapArea;
    IntersectionIdx startPoint, endPoint;
    // mode of the mapping system, dark_mode for color scheme, findingRange for RangeFinder extra feature;
    // M3 path finding global flag
    bool dark_mode, findingRange, pathFindingMode;
    // scale_factor used to determine zooming level
    double scale_factor;
    // total range for the extra feature
    double total_range;
    double global_turn_penalty;

    // two version of current city name
    // first one after preprocessing, second one before preprocessing
    std::string current_map_name;
    std::string currentCity;
    std::string travelInstructions;
    std::vector<StreetSegmentIdx> general_street_seg;
    std::vector<StreetSegmentIdx> motorway_street_seg;
    std::vector<StreetSegmentIdx> trunk_street_seg;
    std::vector<StreetSegmentIdx> primary_street_seg;
    std::vector<StreetSegmentIdx> secondary_street_seg;
    std::unordered_set<POIIdx> cur_POI;
    std::vector<StreetSegmentIdx> pathSegment;
    // set of intersection to be displayed
    std::unordered_set<IntersectionIdx> highlighted_inter;
    // vector of points for range finder extra feature
    std::vector<ezgl::point2d> range_finder_point_list;
    // vector of street pieces for street information displaying
    std::vector<street_piece_info> street_pieces_list;
};

struct myStreetSegment : StreetSegmentInfo {
    double segLength, segTime;
    ezgl::point2d posFrom, posTo;
    std::string street_tag;
    int weight, street_level;
};

struct outgoingEdge {
    StreetSegmentIdx ss_id;
    IntersectionIdx inter_id;
};

struct myStreetIntersection {
    double longitude, latitude;
    ezgl::point2d xy_loc;
    std::string intersecName;
    std::vector<IntersectionIdx> adjIntersecs;
    std::vector<StreetSegmentIdx> street_segments_of_intersections;
    std::vector<outgoingEdge> outgoingEdges;
};
struct myFeatureData {
    double featureArea;
    double minx_, maxx_, miny_, maxy_;
    int numOfPoints;
    std::vector<ezgl::point2d> point_list;
    bool openFeature;
    FeatureIdx feature_id;
    FeatureType feature_type;
};

// template definitions
template<typename T>
void insert_sorted_noDupe(T &new_ele, std::vector<T> &cur_vector);
template<typename T>
std::vector<T> find_common_member_in_sorted(std::vector<T> &first, std::vector<T> &second);

// class definition
class TrieNode{
public:
    std::map<char, TrieNode*> children;
    bool is_end_of_word;
    std::vector<int> id;
    TrieNode();
    ~TrieNode();
};
class Trie{
public:
    TrieNode *root;
    Trie();
    ~Trie();
    void insert(std::string word, int id);
    std::vector<int> search(std::string word);
};

// LatLon to pixel conversion
float x_from_lon(float lon);
float y_from_lat(float lat);
float lon_from_x(float x);
float lat_from_y(float y);


void loadOSMNodeTagPair();
void loadOSMWayTagPair();
void loadStreetSegment();
void loadIntersections();
void loadGraph();
void loadMapInfo();
void loadMapNames();
void loadFeatureData();
double shoelaceFormula(const std::vector<std::pair<double, double>> &points);
bool compareTwoFeatures(const myFeatureData &feature1, const myFeatureData &feature2);
std::string getOSMWayTagValue (OSMID OSMid, std::string key);

#endif //MAPPER_M1_EXTEND_H

