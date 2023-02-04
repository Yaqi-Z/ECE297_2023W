/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m1.cpp"
#include "point.hpp"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <unordered_map>
#include <map>
#include "math.h"

#include <string>
#include <algorithm>

#include "globals.h"
#include "OSMDatabaseAPI.h"
#include "glib.h"


void drawPath(LatLon pointFrom, LatLon pointTo, ezgl::renderer *g); 
bool whatRoad(StreetIdx y);
void draw_main_streets(ezgl::renderer *g);
void draw_main_canvas(ezgl::renderer *g);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
void findButton(ezgl::renderer *g);
void pathWayButtonBetwn2Intersec_Perform(GtkWidget *, ezgl::application *application);
gboolean mouseClickedPathway(ezgl::renderer *g);
void act_on_findButton(GtkWidget */*widget*/, ezgl::application *application);
void on_dialog_response(GtkDialog *dialog, gint response_id, gpointer user_data);
void test_button(GtkWidget */*widget*/, ezgl::application *application);
void initial_setup(ezgl::application *application, bool /*new_window*/);
void draw_map_blank_canvas();
void act_on_Clear(GtkWidget */*widget*/, ezgl::application *application);
void Clear(ezgl::renderer *g);


IntersectionIdx startIntersection;
std::vector<IntersectionIdx> UserClickedIntersec, UserInputIntersec;
std::vector<StreetSegmentIdx> pathStreetSeg;


POIIdx lastHighlightedPOI = -1;
IntersectionIdx lastHighlightedIntersection = -1; //stores the id of the last highlighted intersection

//declare global variables to store information pertaining to the map, especially the streets
ezgl::rectangle t;
bool isResidential = false;
double max1_lat, max1_lon, min1_lat, min1_lon;
bool isPedestrian = false;
bool residentialZoom = false;
bool lstreetZoom = false;


//Global variable intersections which is vector of all intersection data(stores position in lat and long and stores name) )
std::vector<intersection_data> intersections;

double avg_lat;

//function to determine the type for road by fetching data from OSM Database

bool whatRoad(StreetIdx y) {

    OSMID ss = getStreetSegmentInfo(street_streetSegments[y][0]).wayOSMID;
    isHighway = false;
    isPedestrian = false;
    isResidential = false;
    for (unsigned j = 0; j < getTagCount(umap[ss]); j++) {
        std::pair<std::string, std::string> tagPair = getTagPair(umap[ss], j); //fetch the tagPair from OSM Database

        //determine the type of road by examining the tagPair provided by OSM
        if (tagPair.second == "motorway") {
            isHighway = true;
            return true;
        }
        if (tagPair.second == "trunk") {
            isHighway = true;
            return true;
        }
        if (tagPair.second == "residential") {
            isResidential = true;
            return true;
        }
        if (tagPair.second == "service") {
            isPedestrian = true;
            return true;
        }

    }
    return false;
}

//sub-function called by draw_main_canvas to draw the main streets

void draw_main_streets(ezgl::renderer *g) {
    for (StreetIdx streetid = 0; streetid < getNumStreets(); streetid++) { //iterate through all streets
        for (int sssid = 0; sssid < street_streetSegments[streetid].size(); sssid++) {

            //obtain the intersections from & to, and number of curve points
            IntersectionIdx intersectionFrom = getStreetSegmentInfo(street_streetSegments[streetid][sssid]).from;
            IntersectionIdx intersectionTo = getStreetSegmentInfo(street_streetSegments[streetid][sssid]).to;
            int curvepts = getStreetSegmentInfo(street_streetSegments[streetid][sssid]).numCurvePoints;
            bool check = false;
            if (whatRoad(streetid) == true) {
                check = true;
            }

            //the case where a street has no curve points
            if (curvepts == 0) {

                //convert the intersection lat and lon into x and y coordinates
                LatLon y = getIntersectionPosition(intersectionFrom);
                float x = x_from_lon(y.longitude());
                float y2 = y_from_lat(y.latitude());
                LatLon w = getIntersectionPosition(intersectionTo);
                float x1 = x_from_lon(w.longitude());
                float y1 = y_from_lat(w.latitude());

                
                if (check == true) {
                    if (isSegPartOfPath[street_streetSegments[streetid][sssid]] == true) {
                        g->set_line_width(6);
                        g->set_color(ezgl::BLUE);
                        g->draw_line({x, y2}, {x1, y1});
                    } else if (isHighway == true) { //if the street is a highway
                        g->set_line_width(6);
                        g->set_color(ezgl::color(245, 197, 66));
                        g->draw_line({x, y2}, {x1, y1});
                    } else if (isPedestrian == true) { //if the street is a pedestrian way
                        ezgl::rectangle l;
                        l = g->get_visible_world();
                        if (l.area() < t.area() / 24) {
                            g->set_line_width(2);
                            g->set_color(ezgl::BLACK);
                            g->draw_line({x, y2}, {x1, y1});
                        }
                    } else { //if the street is neither highway nor pedestrian way
                        ezgl::rectangle l;
                        l = g->get_visible_world();
                        if (l.area() < t.area() / 16) {
                            g->set_line_width(5);
                            g->set_color(ezgl::color(210, 200, 219, 255));
                            g->draw_line({x, y2}, {x1, y1});
                        }
                    }
                } else { //if the street is not labelled by a type
                    g->set_line_width(4);
                    g->set_color(ezgl::WHITE);
                    g->draw_line({x, y2}, {x1, y1});
                }
            } else {
                LatLon y = getIntersectionPosition(intersectionFrom);
                float x = x_from_lon(y.longitude());
                float f = y_from_lat(y.latitude());
                LatLon cp = getStreetSegmentCurvePoint((street_streetSegments[streetid][sssid]), 0);
                float x1 = x_from_lon(cp.longitude());
                float y1 = y_from_lat(cp.latitude());

                if (check == true) {
                    if (isSegPartOfPath[street_streetSegments[streetid][sssid]] == true) {
                        g->set_line_width(6);
                        g->set_color(ezgl::BLUE);
                        g->draw_line({x, f}, {x1, y1});
                    } else if (isHighway == true) {
                        g->set_line_width(6);
                        g->set_color(ezgl::color(245, 197, 66));
                        g->draw_line({x, f}, {x1, y1});
                    } else if (isPedestrian == true) {
                        ezgl::rectangle l;
                        l = g->get_visible_world();
                        if (l.area() < t.area() / 24) {
                            g->set_line_width(2);
                            g->set_color(ezgl::BLACK);
                            g->draw_line({x, f}, {x1, y1});
                        }
                    } else {
                        ezgl::rectangle l;
                        l = g->get_visible_world();
                        if (l.area() < t.area() / 16) {
                            g->set_line_width(5);
                            g->set_color(ezgl::color(210, 200, 219, 255));
                            g->draw_line({x, f}, {x1, y1});
                        }
                    }
                } else {
                    g->set_line_width(4);
                    g->set_color(ezgl::WHITE);
                    g->draw_line({x, f}, {x1, y1});
                }

                for (int k = 0; k < curvepts - 1; ++k) {
                    LatLon first = getStreetSegmentCurvePoint((street_streetSegments[streetid][sssid]), k);
                    LatLon second = getStreetSegmentCurvePoint((street_streetSegments[streetid][sssid]), k + 1);
                    float x2 = x_from_lon(first.longitude());
                    float y2 = y_from_lat(first.latitude());
                    float xa = x_from_lon(second.longitude());
                    float ya = y_from_lat(second.latitude());

                    if (check == true) {
                        if (isSegPartOfPath[street_streetSegments[streetid][sssid]] == true) {
                        g->set_line_width(6);
                        g->set_color(ezgl::BLUE);
                        g->draw_line({x2, y2}, {xa, ya});
                        } else if (isHighway == true) {
                            g->set_line_width(6);
                            g->set_color(ezgl::color(245, 197, 66));
                            g->draw_line({x2, y2}, {xa, ya});
                        } else if (isPedestrian == true) {
                            ezgl::rectangle l;
                            l = g->get_visible_world();
                            if (l.area() < t.area() / 24) {
                                g->set_line_width(2);
                                //  g->set_line_dash(ezgl::line_dash::asymmetric_5_3);
                                g->set_color(ezgl::BLACK);
                                g->draw_line({x2, y2}, {xa, ya});
                            }
                        } else {
                            ezgl::rectangle l;
                            l = g->get_visible_world();
                            if (l.area() < t.area() / 16) {
                                g->set_line_width(5);

                                g->set_color(ezgl::color(210, 200, 219, 255));
                                g->draw_line({x2, y2}, {xa, ya});
                            }
                        }
                    } else {
                        g->set_line_width(4);
                        g->set_color(ezgl::WHITE);
                        g->draw_line({x2, y2}, {xa, ya});
                    }


                }
                //
                LatLon third = getStreetSegmentCurvePoint((street_streetSegments[streetid][sssid]), curvepts - 1);
                LatLon fourth = getIntersectionPosition(intersectionTo);
                float x3 = x_from_lon(third.longitude());
                float y3 = y_from_lat(third.latitude());
                float xb = x_from_lon(fourth.longitude());
                float yb = y_from_lat(fourth.latitude());


                if (check == true) {
                    if (isSegPartOfPath[street_streetSegments[streetid][sssid]] == true) {
                        g->set_line_width(6);
                        g->set_color(ezgl::BLUE);
                        g->draw_line({x3, y3}, {xb, yb});
                    } else if (isHighway == true) {
                        g->set_line_width(6);

                        g->set_color(ezgl::color(245, 197, 66));
                        g->draw_line({x3, y3}, {xb, yb});
                    } else if (isPedestrian == true) {
                        ezgl::rectangle l;
                        l = g->get_visible_world();
                        if (l.area() < t.area() / 24) {
                            g->set_line_width(2);
                            // g->set_line_dash(ezgl::line_dash::asymmetric_5_3);
                            g->set_color(ezgl::BLACK);
                            g->draw_line({x3, y3}, {xb, yb});
                        }
                    } else {
                        ezgl::rectangle l;
                        l = g->get_visible_world();
                        if (l.area() < t.area() / 16) {
                            g->set_line_width(5);
                            g->set_color(ezgl::color(210, 200, 219, 255));
                            g->draw_line({x3, y3}, {xb, yb});
                        }
                    }

                } else {
                    g->set_line_width(4);
                    g->set_color(ezgl::WHITE);
                    g->draw_line({x3, y3}, {xb, yb});
                }
            }
        }
    }
    
    //Draw the highlighted path when when user asks for direction
    g->set_line_width(6);
    g->set_color(ezgl::BLUE);
    
    for (StreetSegmentIdx segId = 0; segId < isSegPartOfPath.size(); segId++) { 
        if (isSegPartOfPath[segId] == true) {
            IntersectionIdx intersectionFrom = getStreetSegmentInfo(segId).from;
            IntersectionIdx intersectionTo = getStreetSegmentInfo(segId).to;
            int curvepts = getStreetSegmentInfo(segId).numCurvePoints;

            if (curvepts == 0) {
                //convert the intersection lat and lon into x and y coordinates
                LatLon y = getIntersectionPosition(intersectionFrom);
                float x = x_from_lon(y.longitude());
                float y2 = y_from_lat(y.latitude());
                LatLon w = getIntersectionPosition(intersectionTo);
                float x1 = x_from_lon(w.longitude());
                float y1 = y_from_lat(w.latitude());
                
                g->draw_line({x, y2}, {x1, y1});
                
            } else {
                
                LatLon y = getIntersectionPosition(intersectionFrom);
                float x = x_from_lon(y.longitude());
                float f = y_from_lat(y.latitude());
                LatLon cp = getStreetSegmentCurvePoint(segId, 0);
                float x1 = x_from_lon(cp.longitude());
                float y1 = y_from_lat(cp.latitude());
                g->draw_line({x, f}, {x1, y1});

                for (int k = 0; k < curvepts - 1; ++k) {
                    LatLon first = getStreetSegmentCurvePoint(segId, k);
                    LatLon second = getStreetSegmentCurvePoint(segId, k + 1);
                    float x2 = x_from_lon(first.longitude());
                    float y2 = y_from_lat(first.latitude());
                    float xa = x_from_lon(second.longitude());
                    float ya = y_from_lat(second.latitude());
                    g->draw_line({x2, y2}, {xa, ya});
                }

            LatLon third = getStreetSegmentCurvePoint(segId, curvepts - 1);
            LatLon fourth = getIntersectionPosition(intersectionTo);
            float x3 = x_from_lon(third.longitude());
            float y3 = y_from_lat(third.latitude());
            float xb = x_from_lon(fourth.longitude());
            float yb = y_from_lat(fourth.latitude());
            g->draw_line({x3, y3}, {xb, yb});
            }
        }
    }
}

        
void draw_main_canvas(ezgl::renderer *g) {

    g->set_coordinate_system(ezgl::WORLD);

    g->set_color(ezgl::GREY_55);


    //draw features on the map

    ezgl::rectangle visible_world = g->get_visible_world();
    double current_screen_width = abs(visible_world.m_first.x - visible_world.m_second.x); //find screen width to determine zoom level

    for (size_t i = 0; i < features.size(); i++) {
        //set color of feature according to its type
        if ((features[i].type == PARK) || (features[i].type == ISLAND)
                || (features[i].type == GREENSPACE) || (features[i].type == GOLFCOURSE)) {
            g->set_color(ezgl::color(140, 200, 0, 255));
        } else if ((features[i].type == LAKE) || (features[i].type == RIVER) || (features[i].type == STREAM)) {
            g->set_color(ezgl::LIGHT_SKY_BLUE);
        } else {
            g->set_color(ezgl::color(255, 255, 228, 255));
        }

        if (features[i].type != BUILDING || current_screen_width < 1200) { //only display building at a higher zoom level
            int numPoints = features[i].positions.size();

            if ((numPoints > 2) && (features[i].positions[0] == features[i].positions[numPoints - 1])) { //fill in the feature if it is a closed polygon
                std::vector<ezgl::point2d> positions_noRepeat(features[i].positions);
                positions_noRepeat.erase(positions_noRepeat.end() - 1);
                g->fill_poly(positions_noRepeat);
            } else { //draw the lines connecting feature points if not closed polygon

                for (int pNum = 0; (pNum + 1) < numPoints; pNum++) {
                    ezgl::point2d pt1 = features[i].positions[pNum];
                    ezgl::point2d pt2 = features[i].positions[pNum + 1];
                    g->draw_line(pt1, pt2);
                }
            }
        }
    }


    draw_main_streets(g);

    
     
    for (int i = 0; i < intersections.size(); ++i) {
        if (intersections[i].highlight == true || current_screen_width < 10000) { //don't draw intersections at lowest zoom levels
            //Getting the x and y position in metres which are distance in the earth using conversion functions    
            float x = x_from_lon(intersections[i].position.longitude());
            float y = y_from_lat(intersections[i].position.latitude());
            float width = current_screen_width / 200; // In metres intersection width
            float height = width;

            //Draw based on state
            //Choose colour based on what the highlight variable is for every intersection

            //If highlight flag set->draw in red
            if (i == UserClickedIntersec[UserClickedIntersec.size() - 1]) {
                g->set_color(ezgl::RED);
                g->fill_rectangle({x - width * 2, y - height * 2}, {x + width * 2, y + height * 2});
            } else if (i == UserClickedIntersec[UserClickedIntersec.size() - 2]) {
                g->set_color(ezgl::GREEN);
                g->fill_rectangle({x - width * 2, y - height * 2}, {x + width * 2, y + height * 2});
            } else { //else draw in grey
                g->set_color(ezgl::GREY_55);
                g->fill_rectangle({x - width / 2, y - height / 2}, {x + width / 2, y + height / 2});
            }
        }
    }



    for (StreetIdx streetid = 0; streetid < getNumStreets(); streetid++) {
        for (int sssid = 0; sssid < street_streetSegments[streetid].size(); sssid++) {
            IntersectionIdx intersectionFrom = getStreetSegmentInfo(street_streetSegments[streetid][sssid]).from;
            IntersectionIdx intersectionTo = getStreetSegmentInfo(street_streetSegments[streetid][sssid]).to;
            std::string stname = getStreetName(streetid);
            if (sssid == 0) {
                //Printing street names according to first street segment
                LatLon y = getIntersectionPosition(intersectionFrom);
                float x = x_from_lon(y.longitude());
                float y2 = y_from_lat(y.latitude());

                LatLon a = getIntersectionPosition(intersectionTo);
                float x1 = x_from_lon(a.longitude());
                float y3 = y_from_lat(a.latitude());
                //Setting the font according to the city name with fonts allowing for different languages
                /*if (cityName == "beijing" || cityName == "hongkong") {
                    g->format_font("NotoSansCJKSC", ezgl::font_slant::normal, ezgl::font_weight::normal);
                }
                if (cityName == "tokyo") {
                    g->format_font("NotoSansCJKJP", ezgl::font_slant::normal, ezgl::font_weight::normal);
                }
                if (cityName == "cairo") {
                    //g->format_font("NotoSansArabic", ezgl::font_slant::normal, ezgl::font_weight::normal);
                }
                if (cityName == "newdelhi") {
                    //g->format_font("NotoSansDevanagari", ezgl::font_slant::normal, ezgl::font_weight::normal);
                }*/

                double pi = 3.14159;

                ezgl::rectangle l;
                l = g->get_visible_world();
                
                bool check = false;
                if (whatRoad(streetid) == true) {
                    check = true;
                }

                if (isHighway == true) {
                    //Display major roads at certain zoom levels
                    if (l.area() < t.area() / 2 && l.area() > t.area() / 24) {
                        std::string onewayName;

                        g->set_font_size(10);

                        //Set rotation angle
                        double fx = (x1 - x);
                        double fy = (y3 - y2);
                        double radangle = atan2(fy, fx);
                        double angle = (radangle * (180 / pi));
                        if (fx < 0 && fy > 0) {
                            angle = 180 - angle;
                        }
                        if (fx > 0 && fy < 0) {
                            angle = angle;
                        }
                        if (fx < 0 && fy < 0) {
                            angle = 180 + angle;
                        }
                        g->set_text_rotation(angle);

                        double xg = findStreetLength(streetid);

                        //Printing name
                        g->set_color(ezgl::BLACK);
                        g->draw_text({x, y2}, stname, xg / 2, xg / 8);
                    }
                }
                //For residential streets, dividing zoom levels according to if the segment has curved points or not
                if (isResidential == true) {
                    if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).numCurvePoints == 0) {
                        if (l.area() < t.area() / 72 && l.area() > t.area() / 108) {

                            double xg = findStreetLength(streetid);
                            double fx = (x1 - x);
                            double fy = (y3 - y2);

                            double radangle = atan2(fy, fx);
                            //Calculating angle
                            double angle = (radangle * (180 / pi));

                            if (fx > 0 && fy < 0) {
                                angle = angle;
                            }
                            if (fx < 0 && fy < 0) {
                                angle = angle + 180;
                            }
                            if (fx < 0 && fy > 0) {
                                angle = 180 - angle;
                            }

                            std::string oneWay;
                            g->set_text_rotation(angle);

                            //Check if street is one way
                            if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).oneWay == true) {
                                oneWay = stname + " " + "-oneWay";
                            }

                            g->set_font_size(11);

                            //Write the name
                            g->set_color(ezgl::BLACK);
                            if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).oneWay == true) {
                                g->draw_text({(x1 + x) / 2, (y2 + y3) / 2}, oneWay, xg / 2, 150);
                            } else {
                                g->draw_text({(x1 + x) / 2, (y2 + y3) / 2}, stname, xg / 2, 150);
                            }
                        }
                    } else {
                        float x2, y2, xa, ya;
                        double xg = findStreetLength(streetid);
                        //If street segment has 2 curve points
                        if (getStreetSegmentInfo(street_streetSegments[streetid][0]).numCurvePoints == 2) {
                            LatLon first = getStreetSegmentCurvePoint((street_streetSegments[streetid][0]), 0);
                            LatLon second = getStreetSegmentCurvePoint((street_streetSegments[streetid][0]), 1);
                            float xa, x2, ya, y2;
                            //Get the location of curve points
                            x2 = x_from_lon(first.longitude());
                            y2 = y_from_lat(first.latitude());
                            xa = x_from_lon(second.longitude());
                            ya = y_from_lat(second.latitude());

                            double fx = (xa - x2);
                            double fy = (ya - y2);
                           double radangle = atan2(fy, fx);
                            //Set angle
                            double angle = (radangle * (180 / pi));

                            if (fx > 0 && fy < 0) {
                                angle = angle;
                            }
                            if (fx < 0 && fy < 0) {
                                angle = angle + 180;
                            }
                            if (fx < 0 && fy > 0) {
                                angle = 180 - angle;
                            }
                            std::string oneWay1;
                            //Check if one way
                            if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).oneWay == true) {

                                oneWay1 = stname + " " + "-oneWay";
                            }

                            g->set_font_size(10);

                            //Check zoom level
                            g->set_color(ezgl::BLACK);
                            if (l.area() < t.area() / 108) {
                                //Print one way if its a one way street
                                if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).oneWay == true) {
                                    g->draw_text({(xa + x2) / 2, (y2 + ya) / 2}, oneWay1, xg / 2, 200);
                                } else {
                                    g->draw_text({(xa + x2) / 2, (y2 + ya) / 2}, stname, xg / 2, 200);
                                }
                            }
                        }//If street segment has more than 2 curve points
                        else if (getStreetSegmentInfo(street_streetSegments[streetid][0]).numCurvePoints > 2) {
                            float x2, y2, xa, ya;
                            LatLon first = getStreetSegmentCurvePoint((street_streetSegments[streetid][0]), 1);
                            LatLon second = getStreetSegmentCurvePoint((street_streetSegments[streetid][0]), 2);
                            //get location of second and third point
                            x2 = x_from_lon(first.longitude());
                            y2 = y_from_lat(first.latitude());
                            xa = x_from_lon(second.longitude());
                            ya = y_from_lat(second.latitude());
                            double fx = (xa - x2);
                            double fy = (ya - y2);
                            double radangle = atan2(fy, fx);
                            //Calculate angle
                            double pi = 3.14159;
                            double angle = (radangle * (180 / pi));

                            if (fx > 0 && fy < 0) {
                                angle = angle;
                            }
                            if (fx < 0 && fy < 0) {
                                angle = angle + 180;
                            }
                            if (fx < 0 && fy > 0) {
                                angle = 180 - angle;
                            }

                            g->set_text_rotation(angle);

                            std::string oneWay1;
                            //Check if street is one way or not
                            if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).oneWay == true) {
                                oneWay1 = stname + " " + "--oneWay";
                            }

                            g->set_font_size(11);
                            g->set_color(ezgl::BLACK);
                            //Check zoom level
                            if (l.area() < t.area() / 108) {
                                //Print one way if its one way or else print street name without one way
                                if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).oneWay == true) {
                                    g->draw_text({(x1 + x) / 2, (y2 + y3) / 2}, oneWay1, xg / 2, xg / 8);
                                } else {
                                    g->draw_text({(xa + x2) / 2, (y2 + ya) / 2}, stname, xg / 2, xg / 8);
                                }
                            }
                        } else {
                            //if street segments have no curve points
                            LatLon first = getIntersectionPosition(getStreetSegmentInfo((street_streetSegments[streetid][0])).from);
                            LatLon second = getStreetSegmentCurvePoint((street_streetSegments[streetid][0]), 0);
                            float xa, x2, ya, y2;
                            //get location of to and from intersections
                            x2 = x_from_lon(first.longitude());
                            y2 = y_from_lat(first.latitude());
                            xa = x_from_lon(second.longitude());
                            ya = y_from_lat(second.latitude());

                            std::string oneWay;
                            //check if street segment is one way
                            if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).oneWay == true) {
                                oneWay = stname + " " + "--oneWay";
                            }

                            double fx = (xa - x2);
                            double fy = (ya - y2);

                            //calculate angle
                            double radangle = atan2(fy, fx);
                            double pi = 3.14159;
                            double angle = (radangle * (180 / pi));

                            if (fx > 0 && fy < 0) {
                                angle = angle;
                            }
                            if (fx < 0 && fy < 0) {
                                angle = angle + 180;
                            }
                            if (fx < 0 && fy > 0) {
                                angle = 180 - angle;
                            }

                            g->set_text_rotation(angle);
                            g->set_font_size(11);
                            g->set_color(ezgl::BLACK);

                            double xg = findStreetLength(streetid);

                            //Check zoom level
                            if (l.area() < t.area() / 120) {
                                //Check if one way or not
                                if (getStreetSegmentInfo(street_streetSegments[streetid][sssid]).oneWay == true) {
                                    g->draw_text({(x2 + xa) / 2, (y2 + ya) / 2}, oneWay, xg, xg);
                                }
                            } else {
                                g->draw_text({(x2 + xa) / 2, (y2 + ya) / 2}, stname, xg, xg);
                            }
                        }
                    }
                }
            }
        }
    }



    // Draw POIs on the map
    g->set_text_rotation(0);

    if (current_screen_width < 2500) { //only display POIs at a relatively high zoom level    
        float width = 5 + current_screen_width / 250;
        float height = width;

        for (size_t i = 0; i < POIs.size(); i++) { //iterate through all POIs
            //draw POI by different colours depending their type
            if (POIs[i].type == "school" || POIs[i].type == "university") {
                g->set_color(ezgl::RED);
            } else if (POIs[i].type == "fast_food" || POIs[i].type == "cafe") {
                g->set_color(ezgl::CYAN);
            } else {
                g->set_color(ezgl::ORANGE);
            }

            double x = POIs[i].position->x;
            double y = POIs[i].position->y;
            g->fill_rectangle({x, y}, {x + width, y + height}); //draw the square for the POI
        }

        //display POI name
        for (size_t i = 0; i < POIs.size(); i++) { //iterate through all POIs
            g->set_font_size(10);
            g->set_color(ezgl::BLACK);

            //display POI name while considering the zoom level and the distance to the closest POI
            g->draw_text({POIs[i].position->x + width / 2, POIs[i].position->y + height / 2},
            POIs[i].name, POIs[i].closestPOIDistance * 750.00 / current_screen_width, 10);
        }

        //display the legends for the POIs

        //draw background box
        g->set_coordinate_system(ezgl::SCREEN);
        g->set_color(ezgl::YELLOW);
        g->fill_rectangle({550, 470}, {750, 570});

        //legend for schools & uni
        g->set_color(ezgl::RED);
        g->fill_rectangle({560, 475}, {570, 485});

        g->set_color(ezgl::BLACK);
        g->set_font_size(11);
        g->draw_text({645, 480}, "School & University");

        //legend for fast food & cafe
        g->set_color(ezgl::CYAN);
        g->fill_rectangle({560, 495}, {570, 505});

        g->set_color(ezgl::BLACK);
        g->draw_text({640, 500}, "Fast Food & Cafe");

        //legend for other POIs
        g->set_color(ezgl::ORANGE);
        g->fill_rectangle({560, 515}, {570, 525});

        g->set_color(ezgl::BLACK);
        g->draw_text({610, 520}, "Others");
    }


    //display the name of the city and/or country of current map

    //draw the background box
    g->set_coordinate_system(ezgl::SCREEN);
    g->set_color(ezgl::YELLOW);
    g->fill_rectangle({0, 500}, {135, 535});

    //display the location name
    g->set_font_size(15);
    g->set_color(ezgl::BLACK);
    g->draw_text({60, 520}, map_path.substr(26, map_path.length() - 38));
    
}



//Function takes a pointer to ezgl application,
// takes to a pointer to a data structure that stores some information about the event
//x,y are two parameters in world coordinates-tells us where the user clicked

void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y) {
    std::cout << "Mouse clicked at (" << x << "," << y << ")\n";

    //Print out what button is clicked
    std::cout << "Button clicked is " << event->button << std::endl;

    //Clicking mouse to find closest intersection
    LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));

    //Print the latlon position
    std::cout << " LatLon" << pos.latitude() << "," << pos.longitude() << ")\n";
    //Call m1 function to find closest intersection
    int id = findClosestIntersection(pos);
    POIIdx poId = findClosestPOI(pos, "Starbucks");

    //Print out the intersection we found
    std::cout << "Closest Intersection: " << intersections[id].name << "\n";
    std::cout << "Closest Starbucks: " << POIs[poId].name << "\n";

    std::cout << "Street Names of Intersection: " << "\n";
    std::vector<std::string> intersectionstreetnames = findStreetNamesOfIntersection(id);
    for (int a = 0; a < intersectionstreetnames.size(); a++) {
        std::cout << intersectionstreetnames[a] << "\n";
    }
    
    //For DisplayPath 2 intersec in M3 below 1 line added 
    
    startIntersection = id;
    
    //Highlight the closest intersection-setting highlight flag for that intersection to true
    
    
    //M3 - For DisplayPath 2 intersec in M3 added 
    //getting the clicked start intersections in the UserClickedIntersec vector
    
    UserClickedIntersec.push_back(startIntersection);
    
    //M3- Increment the counter of mouse pressed
    intersections[id].intersectionMouseClkCounter++;
    
    //highlight the last two clicked intersections and unhighlight 3rd last clicked intersection
    intersections[UserClickedIntersec[UserClickedIntersec.size() - 1]].highlight = true;
    if (UserClickedIntersec.size() > 1) {
        intersections[UserClickedIntersec[UserClickedIntersec.size() - 2]].highlight = true;
    }
    if (UserClickedIntersec.size() > 2) {
        intersections[UserClickedIntersec[UserClickedIntersec.size() - 3]].highlight = false;
    }

    //if (POIs[poId].name == "Starbucks") {
    POIs[poId].highlight = true; //}
    POIs[lastHighlightedPOI].highlight = false;
    lastHighlightedPOI = poId;


    lastHighlightedIntersection = id; //remember the currently highlighted intersection

    app->update_message(intersections[id].name);
    
    
    //redraw since highlight intersection has changed
    app->refresh_drawing();
}



//code for the button to highlight the intersections between the streets provided as input by the user

void findButton(ezgl::renderer *g) {

    //initialize variables 

    std::string St1;
    std::string St2;
    int st1Select = 0;
    int st2Select = 0;
    IntersectionIdx fromIntersection, toIntersection;
    
    for (int loopNum = 0; loopNum < 2; loopNum++) {

        
        //print prompt to the user to ask for inputs of street names
        if (loopNum == 0) {
            std::cout << "Enter two street names for the starting intersection (e.g. Warden Avenue & Ellesmere Road). Please use '&' in between" << std::endl;
        } else if (loopNum == 1) {
            std::cout << "Enter two street names for the destination intersection (e.g. Tilson Road & Boyton Road). Please use '&' in between" << std::endl;
        }

        //obtain inputs from the user
        std::getline(std::cin, St1, '&');
        std::getline(std::cin, St2);
        std::cin.clear();

        //print message to the user confirming the street names
        std::cout << "Street1 is: " << St1 << std::endl;
        std::cout << "Street2 is: " << St2 << std::endl;


        //fetch the street ids based on street name provided by the user
        std::vector<StreetIdx> st1Ids;
        std::vector<StreetIdx> st2Ids;
        st1Ids = findStreetIdsFromPartialStreetName(St1);
        st2Ids = findStreetIdsFromPartialStreetName(St2);

        st1Select = 0;
        st2Select = 0;

        //If vector empty-no street exists
        if (st1Ids.size() == 0) {
            std::cout << St1 << " is not a valid street name. Please try again by pressing find" << std::endl;
        }

        if (st2Ids.size() == 0) {
            std::cout << St2 << " is not a valid street name. Please try again by pressing find" << std::endl;
        }

        //a street id corresponding to the street name is found
        if (st1Ids.size() == 1) {
        } else { //List some street Id's and user chooses
            std::cout << "Select a street(1,2, etc.) " << std::endl;
            for (st1Select = 0; st1Select < st1Ids.size(); st1Select++) {
                std::cout << "Select: " << st1Select << " is " << getStreetName(st1Ids[st1Select]) << std::endl;
            }

            //Print to screen what user chose
            std::cin>>st1Select;
            std::cout << std::endl;
            std::cout << st1Select << std::endl;
            std::cout << "You have selected: " << getStreetName(st1Ids[st1Select]) << std::endl;
            std::cin.clear();
        }


        //the same set of procedures for the second street
        if (st2Ids.size() == 1) {
        } else { //List some street Id's and user chooses
            std::cout << "Select a street(1,2, etc.) " << std::endl;
            for (st2Select = 0; st2Select < st2Ids.size(); st2Select++) {
                std::cout << "Select: " << st2Select << " is " << getStreetName(st2Ids[st2Select]) << std::endl;
            }
            //Print to screen what user chose
            std::cin >> st2Select;
        }

        //pass the two street ids into findIntersectionsOfTwoStreets to obtain common intersections
        std::pair<StreetIdx, StreetIdx> twoStInterIds;
        twoStInterIds = std::make_pair(st1Ids[st1Select], st2Ids[st2Select]);

        std::vector<IntersectionIdx> highlightedIntersections;
        highlightedIntersections = findIntersectionsOfTwoStreets(twoStInterIds);
        if (loopNum == 0) {
            fromIntersection = highlightedIntersections[0];
        } else if (loopNum == 1) {
            toIntersection = highlightedIntersections[0];
        }
    }
    
    UserClickedIntersec.push_back(fromIntersection);
    UserClickedIntersec.push_back(toIntersection);
    intersections[fromIntersection].highlight = true;
    intersections[toIntersection].highlight = true;

    pathStreetSeg = findPathBetweenIntersections(fromIntersection, toIntersection, 0);
        for (int i = 0; i < pathStreetSeg.size(); i++) {
            int segId = pathStreetSeg[i];
            isSegPartOfPath[segId] = true;
        }
    printTravelDirections(pathStreetSeg);
}




//Below 2 functions are for user clicked intersection path display

gboolean mouseClickedPathway(ezgl::renderer *g){
    std::vector<StreetSegmentIdx> pathwayBetwnIntersecVec;
  
   
            
    int pathwayClickElements = UserClickedIntersec.size();
        
        while (pathwayClickElements < 2) {
            pathwayClickElements = UserClickedIntersec.size(); //wait for user to click on two intersections
        }
        
        if(pathwayClickElements < 2){
            return FALSE; //No. of Intersection has to be atleast 2 for giving a path
        }
        
        std::cout << "You have entered this intersection " << UserClickedIntersec[pathwayClickElements-2] << " and this " << UserClickedIntersec[pathwayClickElements-1] << std::endl; 
        
        //Using draw path Street seg to draw the lines
        pathStreetSeg = findPathBetweenIntersections(UserClickedIntersec[pathwayClickElements-2], UserClickedIntersec[pathwayClickElements-1], 0); 
        for (int i = 0; i < pathStreetSeg.size(); i++) {
            int segId = pathStreetSeg[i];
            isSegPartOfPath[segId] = true;
        }
        printTravelDirections(pathStreetSeg);
    
        return TRUE;
   }//end bracket
    
    
    
    

 void pathWayButtonBetwn2Intersec_Perform(GtkWidget *, ezgl::application *application){
    
    
    //gboolean chooseIntersections;
    
    std::cout << "To display path, choose 2 intersections by using 'Find' on interface or click on the intersections in the map." << std::endl;
    
    //chooseIntersections = mouseClickedPathway();
    
    // Redraw the graphics
    application->refresh_drawing(); 
    
    GtkWidget* PathwayWindow = (GtkWidget*)application->get_object("PathwayWindow");
    
    gtk_widget_show(PathwayWindow);
   
    if(!(mouseClickedPathway(application->get_renderer()))){
    // Update the status bar message
    application->update_message("You have not given the required no. of intersections. Could you try again?");
    
    }   
    application->refresh_drawing();
 }

 
 
 
 
//Find button implementation

void act_on_findButton(GtkWidget */*widget*/, ezgl::application *application) {

    GtkEntry* text_entry = (GtkEntry *) application->get_object("Type Path");
    // Get the text written in the widget
    const char* text = gtk_entry_get_text(text_entry);
    // Update the status bar message
    application->update_message("Find Button Clicked");

    findButton(application->get_renderer());
    // Redraw the graphics
    application->refresh_drawing();
}

/*Find Button Implementation upto here */



/*Draw path Implementation for M3*/


 void on_dialog_response(GtkDialog *dialog, gint response_id, gpointer user_data) {
    std::cout <<"response is " ; 
    switch(response_id) {
        case GTK_RESPONSE_ACCEPT:
            std::cout <<"GTK_RESPONSE_ACCEPT";
            break ;
        case GTK_RESPONSE_DELETE_EVENT:
            std::cout << "GTK_RESPONSE_DELETE_EVENT (i.e. 'X' button)";
            break;
        case GTK_RESPONSE_REJECT:
            std::cout << "GTK_RESPONSE_REJECT";
            break;  
        default:
            std::cout << "UNKNOWN";
            break;
        
    }
    std::cout << "(" << response_id << ")\n";
    
    gtk_widget_destroy(GTK_WIDGET(dialog));
    
}


void test_button(GtkWidget */*widget*/, ezgl::application *application)
{
    GObject *window; //the parent window over which to add the dialog
    GtkWidget *content_area;  //the content area of the dialog
    GtkWidget *label ; //the label we will create to display a message in the content area
    GtkWidget *dialog ; // the dialog box we will create
  // Update the status bar message
  
    window = application->get_object(application->get_main_window_id().c_str());
    dialog = gtk_dialog_new_with_buttons(
            "Instructions",
            (GtkWindow*) window, 
            GTK_DIALOG_MODAL, 
            ("OK"),
            GTK_RESPONSE_ACCEPT, 
            ("CANCEL"),
            GTK_RESPONSE_REJECT,
            NULL
            );
    
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    label = gtk_label_new("1. If you want to manually type in 2 intersections, press the Type Path button.\n"
            "2. If you want to click on 2 places in the map and find path between them, press the Click Path button\n"
            "3. The starting point of the path is displayed in green whereas the destination point of the path is displayed in red.\n"
            "4. If you want to clear the highlighted intersections and paths on the map, press the Clear button.\n"
            "Press OK or CANCEL to close.");
    gtk_container_add(GTK_CONTAINER(content_area),label);
    
    gtk_widget_show_all(dialog);
    
    g_signal_connect(
            GTK_DIALOG(dialog),
            "response",
            G_CALLBACK(on_dialog_response),
            NULL
            );
    
    
    application->update_message("Test Button Pressed");
  
  // Redraw the main canvas
  application->refresh_drawing();
}



void Clear(ezgl::renderer *g) {
    std::cout << "The highlighted intersections and paths on the map have been cleared.\n";
    
    for (IntersectionIdx interId = 0; interId < intersections.size(); interId++) {
        intersections[interId].highlight = false;
    }
    UserClickedIntersec.clear();
    
    for (StreetSegmentIdx segNum = 0; segNum < isSegPartOfPath.size(); segNum++) {
        isSegPartOfPath[segNum] = false;
    }
}

void act_on_Clear(GtkWidget */*widget*/, ezgl::application *application) {

    GtkEntry* text_entry = (GtkEntry *) application->get_object("Clear");
    // Get the text written in the widget
    //const char* text = gtk_entry_get_text(text_entry);
    // Update the status bar message
    application->update_message("Clear Button Clicked");

    Clear(application->get_renderer());
    // Redraw the graphics
    application->refresh_drawing();
}


void initial_setup(ezgl::application *application, bool /*new_window*/)
{
  // Update the status bar message
  application->update_message("EZGL Application");

  // Create a Test button and link it with test_button callback fn.
  application->create_button("Type Path", 6, act_on_findButton);
  application->create_button("Click Path", 7, pathWayButtonBetwn2Intersec_Perform);
  application->create_button("Clear", 8, act_on_Clear);
  application->create_button("Help", 9, test_button);

}

void draw_map_blank_canvas() {
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";

    ezgl::application application(settings);

    ezgl::rectangle initial_world({x_from_lon(min1_lon), y_from_lat(min1_lat)}, {x_from_lon(max1_lon), y_from_lat(max1_lat)});
    t = initial_world; //remember the rectangle coordinates of the initial world for later use

    application.add_canvas("MainCanvas", draw_main_canvas, initial_world, ezgl::color(240, 240, 240, 255));
    //Application running until user quits
    application.run(initial_setup, act_on_mouse_click, nullptr, nullptr);
}

void drawMap() {
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";

    ezgl::application application(settings);

    draw_map_blank_canvas();

    application.run(nullptr, nullptr, nullptr, nullptr);
}
