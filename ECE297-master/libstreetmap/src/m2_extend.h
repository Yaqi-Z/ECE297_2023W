//
// Created by lihanli3 on 3/12/23.
//

#ifndef MAPPER_M2_EXTEND_H
#define MAPPER_M2_EXTEND_H

#include "m1_extend.h"
#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <chrono>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <unordered_set>
#include <cmath>
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <curl/curl.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// UI design, button, and corresponding callback functions:
size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata);
void dialog_window_callback(GtkDialog *self, gint response_id, ezgl::application *app);
void draw_main_canvas (ezgl::renderer *g);
void act_on_mouse_click (ezgl::application* app, GdkEventButton* event, double x, double y);
void initial_setup (ezgl::application* application, bool /*new_window */);
void toggle_dark (GtkWidget* /*widget*/, ezgl::application* application);
void toggle_rangeFinder (GtkWidget* /*widget*/, ezgl::application* application);
void combo_box_change_map(GtkComboBoxText* self, ezgl::application *application);
gboolean search_from_changed(GtkWidget *searchEntry, gpointer data);
gboolean search_button_clicked(GtkWidget *searchButton, gpointer data);
gboolean searchCompletion_match_selected (GtkEntryCompletion* self, GtkTreeModel* model, GtkTreeIter* iter, gpointer *);
gboolean weather_button_clicked(GtkWidget *, gpointer ezgl_app);

// feature drawing functions:
void draw_features(ezgl::renderer *g);
void draw_closed_feature(ezgl::renderer *g, int list_id);
void draw_open_feature(ezgl::renderer *g, int list_id);
void draw_simple_feature(ezgl::renderer *g, int list_id);

// street segment and street information drawing functions:
void draw_streetSeg(ezgl::renderer *g);
void draw_street_pieces_info(ezgl::renderer *g);
void draw_street_seg_from_list(ezgl::renderer *g, std::vector<StreetSegmentIdx> SSid_list);
void draw_simple_street_seg_from_list(ezgl::renderer *g, std::vector<StreetSegmentIdx> SSid_list);
void draw_street_segment(ezgl::renderer *g, StreetSegmentIdx& street_segment_id);
void draw_simple_street_segment(ezgl::renderer *g, StreetSegmentIdx& street_segment_id);
void draw_street_piece_from_two_points(ezgl::renderer *g, StreetSegmentIdx& street_segment_id, ezgl::point2d& point1, ezgl::point2d& point2);
void draw_simple_street_piece_from_two_points(ezgl::renderer *g, StreetSegmentIdx& street_segment_id, ezgl::point2d& point1, ezgl::point2d& point2);

// intersection drawing functions:
void draw_intersection(ezgl::renderer *g);

// POI drawing functions:
void show_POI_name(ezgl::renderer *g);
void draw_single_poi(ezgl::renderer *g, POIIdx POI_id);
double getPOIIconScale();

// other helper functions:
bool outsideVisible(const ezgl::rectangle &visible_world, double minx_, double maxx_, double miny_, double maxy_);

// functions for extra features:
void draw_range_finder_points(ezgl::renderer *g);

// M3 path finding callback function
gboolean pathModeSwitch_toggled(GtkWidget *, gboolean state, ezgl::application *data);
void path_finding_draw_street_piece_from_two_points(ezgl::renderer *g, StreetSegmentIdx& street_segment_id, ezgl::point2d& point1, ezgl::point2d& point2);
void path_finding_draw_street_segment(ezgl::renderer *g, StreetSegmentIdx& street_segment_id);
void path_finding_draw_street_seg_from_list(ezgl::renderer *g, std::vector<StreetSegmentIdx> SSid_list);
gboolean help_button_clicked(GtkWidget *, gpointer data);
void printPath(GtkListStore *directionListStore);
#endif //MAPPER_M2_EXTEND_H
