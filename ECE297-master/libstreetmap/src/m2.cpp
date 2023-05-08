#include "m2.h"
#include "m1.h"
#include "m2_extend.h"
#include "m3.h"
#include "m3_extend.h"
#include <fstream>

// global variables from m1.cpp:
extern std::vector<myStreetSegment> mySegData;
extern std::vector<myStreetIntersection> streetIntersectionData;
extern myMap current_map;
extern std::map<std::string, std::string>mapNames;
extern std::vector<myFeatureData> myFeatures;

extern std::vector<std::vector<int>> color_list_light;
extern std::vector<std::vector<int>> color_list_dark;
extern std::vector<std::vector<int>> color_list;

// five pngs for POI icons
extern ezgl::surface *restaurant_png;
extern ezgl::surface *regular_png;
extern ezgl::surface *school_png;
extern ezgl::surface *health_care_png;
extern ezgl::surface *transit_png;

// drawMap: draw the application window & contents inside.
// sets the ui, window and canvas
// calls initial_setup to init the window
void drawMap() {
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    ezgl::application application(settings);

    ezgl::rectangle initial_world({current_map.minX, current_map.minY},
                                  {current_map.maxX, current_map.maxY});
    application.add_canvas("MainCanvas", draw_main_canvas, initial_world);

    application.run(initial_setup, act_on_mouse_click, nullptr, nullptr);
}

// major function controlling drawing of each picture
void draw_main_canvas(ezgl::renderer *g) {
    // for benchmarking, test the time to draw the main canvas (Time measurement disabled)
    auto startTime = std::chrono::high_resolution_clock::now();

    // determine light / dark color scheme and load to active color_list
    // also fill the background of the world
    if (!current_map.dark_mode) {
        for(int i=0; i<17; i++){
            for(int j=0; j<3; j++){
                color_list[i][j] = color_list_light[i][j];
            }
        }
        ezgl::rectangle visible_world = g->get_visible_world();
        g->set_color(color_list[16][0], color_list[16][1], color_list[16][2]);
        g->fill_rectangle(visible_world);
    } else {
        for(int i=0; i<17; i++){
            for(int j=0; j<3; j++){
                color_list[i][j] = color_list_dark[i][j];
            }
        }
        ezgl::rectangle visible_world = g->get_visible_world();
        g->set_color(color_list[16][0], color_list[16][1], color_list[16][2]);
        g->fill_rectangle(visible_world);
    }
    // set up scale factor
    current_map.scale_factor = sqrt(g->get_visible_world().area());

    // draw features, street segments, intersections, street info, and POIs
    // if using the range meter: also draw relative information
    draw_features(g);
    draw_streetSeg(g);
    draw_intersection(g);
    draw_street_pieces_info(g);
    show_POI_name(g);
    if(current_map.findingRange){draw_range_finder_points(g);}
    
    // Time measurement disabled
    auto curTime = std::chrono::high_resolution_clock::now();
    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    std::cout << "draw_main_canvas took " << wallClock.count() << " seconds" << std::endl;
    int numOfFrames = 1/wallClock.count();
    std::cout << "drew " << numOfFrames << " frames per second" << std::endl;
}

// Interface set up (buttons, combo boxes, search bar .etc)
void initial_setup(ezgl::application* application, bool /*new_window */) {
    // Create a combo box for switching between maps
    application->create_combo_box_text(
        "ChangeMapBox:",
        0,
        combo_box_change_map,
        {"Select a map", "Beijing", "Cairo", "Cape Town",
         "Golden Horseshoe", "Hamilton", "Hong Kong",
         "Iceland", "Interlaken", "Kyiv",
         "London", "New Delhi", "New York",
         "Rio De Janeiro", "Saint Helena", "Singapore",
         "Sydney", "Tehran", "Tokyo", "Toronto"}
    );

    // 0. Glade -> Add SearchEntry -> Add & Bind SearchEntryCompletion -> Add & Bind ListStore
    //    -> Edit SearchEntryCompletion -> Add Renderer to render 1st column in ListStore as Text
    // 1. Initial setup -> register signal "changed" callback on searchFrom & searchTo,
    //   which are obtained from searchFrom/To's ID configured in Glade

    // get the object's unique ID defined in Glade
    auto *searchFrom = application->get_object("SearchFrom");
    assert(searchFrom);
    g_signal_connect(searchFrom, "changed", G_CALLBACK(search_from_changed), application);
    auto *searchTo = application->get_object("SearchTo");
    assert(searchTo);
    g_signal_connect(searchTo, "changed", G_CALLBACK(search_from_changed), application);

    // similarly, register signal "clicked" callback to buttons below

    auto *searchButton = application->get_object("SearchButton");
    g_signal_connect(searchButton, "clicked", G_CALLBACK(search_button_clicked), application);

    auto *themeButton = application->get_object("ChangeTheme");
    g_signal_connect(themeButton, "clicked", G_CALLBACK(toggle_dark), application);

    auto *weatherButton = application->get_object("WeatherButton");
    g_signal_connect(weatherButton, "clicked", G_CALLBACK(weather_button_clicked), application);

    // button for activating range finder, binded with toggle_rangeFinder callback;
    auto *rangeFinderButton = application->get_object("RangeFinderButton");
    g_signal_connect(rangeFinderButton, "clicked", G_CALLBACK(toggle_rangeFinder), application);

    auto *helpButton = application->get_object("HelpButton");
    g_signal_connect(helpButton, "clicked", G_CALLBACK(help_button_clicked), application);

    auto *searchFromCompletion = application->get_object("SearchFromCompletion");
    auto *searchToCompletion = application->get_object("SearchToCompletion");

    // connect callbacks for two search boxes
    g_signal_connect(searchFromCompletion, "match-selected", G_CALLBACK(searchCompletion_match_selected), application);
    g_signal_connect(searchToCompletion, "match-selected", G_CALLBACK(searchCompletion_match_selected), application);
    g_signal_connect(searchFromCompletion, "cursor-on-match", G_CALLBACK(searchCompletion_match_selected), application);

    auto *pathModeSwitch = application->get_object("PathModeSwitch");
    g_signal_connect(pathModeSwitch, "state-set", G_CALLBACK(pathModeSwitch_toggled), application);

}
// search completion callback function, when a search result is clicked
gboolean searchCompletion_match_selected (GtkEntryCompletion* self, GtkTreeModel* model, GtkTreeIter* iter, gpointer *)
{
    std::cerr << "entry name=" << gtk_widget_get_name(gtk_entry_completion_get_entry(self)) << std::endl;
    StreetIdx streetIdx;

    // get the street index. row 0 contains road name, row 1 contains streetIdx, which is unique
    gtk_tree_model_get(model, iter, 1, &streetIdx, -1);

    std::cerr<<"street name = "<<getStreetName(streetIdx)<<std::endl;
    std::cerr<<"street id = "<<streetIdx<<std::endl;

    return false;
}


// Callback functions
// callback for mouse clicking
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y) {
    // print some fundamental information to the terminal
     printf("Mouse clicked at (%lf, %lf)\n",x, y);
     std::cout << "Button number was " << event->button << std::endl;
    // check mode
    if(current_map.findingRange){
        if(current_map.scale_factor >= sqrt(current_map.mapArea)){return;}
        // record new measuring point and update msg to diaplay
        ezgl::point2d temp_point(x,y);
        current_map.range_finder_point_list.emplace_back(temp_point);
        // initialize some helper variables
        int list_size = current_map.range_finder_point_list.size();
        std::string themeMsg = "Point ";
        themeMsg = themeMsg + std::to_string(list_size);
        themeMsg = themeMsg + " dropped;";
        double number_to_display;
        std::string total_distance_msg;
        // check for special case: only one point
        if(list_size == 1){
            themeMsg = themeMsg + " Total distance: 0.0 meters.";
            app->update_message(themeMsg);
            app->refresh_drawing();
            return;
        } else {
            // genearl case:
            // temp variable for distance calculating
            ezgl::point2d temp = current_map.range_finder_point_list[list_size-1]
                                 - current_map.range_finder_point_list[list_size-2];
            // update global total range
            current_map.total_range += sqrt(temp.x * temp.x + temp.y * temp.y);
            // if current_map.total_range > 1000m: display in kilometers, else in meters
            if(current_map.total_range >= 1000){
                // rounded to tenth of km and remove redundant zeros
                number_to_display = current_map.total_range/1000;
                number_to_display = round(number_to_display*10)/10;
                total_distance_msg = std::to_string(number_to_display);
                total_distance_msg.erase(total_distance_msg.find_last_not_of('0') + 1, std::string::npos);
                if(total_distance_msg.back() == '.'){
                    total_distance_msg.pop_back();
                }
                themeMsg = themeMsg + "Total distance: " + total_distance_msg + " kilometers.";
                app->update_message(themeMsg);
                app->refresh_drawing();
                return;
            } else {
                // rounded to tenth of m and remove redundant zeros
                number_to_display = round(current_map.total_range*10)/10;
                total_distance_msg = std::to_string(number_to_display);
                total_distance_msg.erase(total_distance_msg.find_last_not_of('0') + 1, std::string::npos);
                if(total_distance_msg.back() == '.'){
                    total_distance_msg.pop_back();
                }
                themeMsg = themeMsg + "Total distance: " + total_distance_msg + " meters.";
                app->update_message(themeMsg);
                app->refresh_drawing();
                return;
            }
        }
    }
    else {
        // general mode: select/unselect intersections
        LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));
        int inter_id = findClosestIntersection(pos);

        if (current_map.pathFindingMode) {
            auto *searchButton = app->get_object("SearchButton");
            // state 0
            // starting point is either not set or we are finding a new path
            // endPoint != -1 indicates we previously found a path
            // just set starting point to the closest intersection, clear the end point
            if (current_map.startPoint == -1 || current_map.endPoint != -1) {
                current_map.startPoint = inter_id;
                current_map.endPoint = -1;
                current_map.pathSegment.clear();
                gtk_button_set_label(reinterpret_cast<GtkButton *>(searchButton), "End");
                gtk_list_store_clear((GtkListStore *)app->get_object("DirectionListStore"));
            }
            // starting point is the same as the input: back to state 0
            else if (current_map.startPoint == inter_id) {
                current_map.startPoint = current_map.endPoint = -1;
                current_map.pathSegment.clear();
                gtk_button_set_label(reinterpret_cast<GtkButton *>(searchButton), "Begin");
            }

            // state 1: startPoint != -1 && endPoint == -1
            // starting point is set, and new point differs from starting point
            // we start finding paths
            else {
                current_map.endPoint = inter_id;
                gtk_button_set_label(reinterpret_cast<GtkButton *>(searchButton), "Begin");
                printPath((GtkListStore *)app->get_object("DirectionListStore"));
            }
        }
        else {
            auto iterator = current_map.highlighted_inter.find(inter_id);
            // update list of intersections to be highlighted
            if (iterator != current_map.highlighted_inter.end()) {
                current_map.highlighted_inter.erase(iterator);
            } else {
                current_map.highlighted_inter.emplace(inter_id);
            }
        }
        printf("IntersecIdx is %d, pos is (%lf, %lf).\n", inter_id, streetIntersectionData[inter_id].xy_loc.x, streetIntersectionData[inter_id].xy_loc.y);
        std::stringstream ss;
        ss << "Closest Intersection: "
           << streetIntersectionData[inter_id].intersecName << "\n";
        // output msg to bottom left of the screen
        app->update_message(ss.str());
        app->refresh_drawing();
    }

}
// SearchFrom (and searchTo) search bar callback function
gboolean search_from_changed(GtkWidget *searchEntry, gpointer data)
{
    // 2. cast "data" to `ezgl::application *` as it was registered when g_signal_connect() is called
    auto ezgl_app = static_cast<ezgl::application *>(data);

    // 3. Get searchEntryListStore from its ID configured in Glade
    auto * searchEntryListStore = (GtkListStore *)ezgl_app->get_object("SearchFromCompletionStore");
    gtk_list_store_clear(searchEntryListStore);

    // 4. declare "iter" for uses in gtk_list_store_append() & gtk_list_store_set()
    GtkTreeIter iter;
    const char * entryText = gtk_entry_get_text((GtkEntry *)searchEntry);

    auto streetIds = findStreetIdsFromPartialStreetName(entryText);
    for (int idx = 0; idx < streetIds.size() && idx < 5; idx++) {
        // 5a. Append row into list store
        gtk_list_store_append (searchEntryListStore, &iter);

        // 5b. Set content of the appended row
        gtk_list_store_set (searchEntryListStore, &iter,
                            0, getStreetName(streetIds[idx]).c_str(), // gtk only accepts C-string
                            1, streetIds[idx],
                            -1);
    }

    return TRUE;
}
// SearchButton callback function. After clicking search,
// function would highlight mutual intersections of the two streets
gboolean search_button_clicked(GtkWidget *searchButton, gpointer data) {

    // set up variables for later use, essentially the contents in the two search bar
    auto ezgl_app = static_cast<ezgl::application *>(data);
    auto *searchFromObj = (GtkSearchEntry *)ezgl_app->get_object("SearchFrom");
    auto * searchToObj = (GtkSearchEntry *)ezgl_app->get_object("SearchTo");
    const char * searchFromText = gtk_entry_get_text((GtkEntry *)searchFromObj);
    const char * searchToText = gtk_entry_get_text((GtkEntry *)searchToObj);

    // call m1 APIs to find mutual street intersections
    auto fromIds = findStreetIdsFromPartialStreetName(searchFromText);
    auto toIds = findStreetIdsFromPartialStreetName(searchToText);
    int fromSize = fromIds.size(), toSize = toIds.size();
    bool intersectionFound = false;

    // show all the intersections
    for (int fromIndex = 0; fromIndex < fromSize; fromIndex++) {
        for (int toIndex = 0; toIndex < toSize; toIndex++) {
            auto mutualIntersections = findIntersectionsOfTwoStreets(fromIds[fromIndex], toIds[toIndex]);
            int numNeutralStreet = mutualIntersections.size();
            intersectionFound = numNeutralStreet;
            if (!intersectionFound) continue;
            printf("mutualIntersections = %d\nstreetId are listed below\n", numNeutralStreet);

            if (!current_map.pathFindingMode) {
                for (size_t i = 0; i < numNeutralStreet; i++) {
                    current_map.highlighted_inter.emplace(mutualIntersections[i]);
                    printf("%d ", mutualIntersections[i]);
                }
                printf("\n");
                continue;
            }

            // path finding mode
            // state 0: fresh start or just finished finding
            if (current_map.startPoint == -1 || (current_map.startPoint != -1 && current_map.endPoint != -1)) {
                current_map.startPoint = mutualIntersections[0];
                current_map.endPoint = -1;
                current_map.pathSegment.clear();
                current_map.travelInstructions = "No valid path. Please use Path Mode to generate a path.";
            }
            // state 1: found starting point
            else if (current_map.startPoint != -1 && current_map.endPoint == -1) {
                current_map.endPoint = mutualIntersections[0];
                // corner case: start == end, we set back to state 0
                if (current_map.endPoint == current_map.startPoint) {
                    current_map.startPoint = current_map.endPoint = -1;
                    current_map.pathSegment.clear();
                } else {
                    printPath((GtkListStore *)ezgl_app->get_object("DirectionListStore"));
                }
            } else {
                std::cerr << "Unknown case. You fucked up. startPoint = " << current_map.startPoint <<
                ", endPoint = " << current_map.endPoint << std::endl;
            }
        }
    }
    if (!intersectionFound){ezgl_app->update_message("Invalid input. Check your spelling.");}

    if (current_map.pathFindingMode) {
        // update searchButton label
        if (current_map.startPoint == -1 || current_map.endPoint != -1) {
            gtk_button_set_label(reinterpret_cast<GtkButton *>(searchButton), "Begin");
        }
        else {
            gtk_list_store_clear((GtkListStore *) ezgl_app->get_object("DirectionListStore"));
            gtk_button_set_label(reinterpret_cast<GtkButton *>(searchButton), "End");
        }
    }
    ezgl_app->refresh_drawing();       // Force a redraw now
    return TRUE;
}
void printPath(GtkListStore *directionListStore) {
    double distanceEst = findDistanceBetweenTwoPoints(getIntersectionPosition(current_map.startPoint),
                                                      getIntersectionPosition(current_map.endPoint));
    if(distanceEst >= 5000){current_map.global_turn_penalty = 0;}
    else if(distanceEst >= 1000){current_map.global_turn_penalty = 5;}
    else {current_map.global_turn_penalty = 15;}
    current_map.pathSegment = findPathBetweenIntersections(
            {current_map.startPoint, current_map.endPoint}, current_map.global_turn_penalty);
    current_map.travelInstructions = showPathDetails(current_map.pathSegment, current_map.global_turn_penalty);
    std::cout << current_map.travelInstructions << std::endl;
    std::vector<std::string> directionString;
    directionString = {""};
    directionString.emplace_back(current_map.travelInstructions);

    GtkTreeIter iter;
    gtk_list_store_clear(directionListStore);

    for (int idx = 0; idx < directionString.size(); idx++) {
        // Append row into list store
        gtk_list_store_append(directionListStore, &iter);
        // Set content of the appended row
        gtk_list_store_set(directionListStore, &iter, 0, directionString[idx].c_str(), -1);
    }
}
// libcurl data writing callback function
size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata) {
    ((std::string *) userdata) ->append(ptr, size * nmemb);
    return size * nmemb;
}
// dialog window callback function, closes the dialog
void dialog_window_callback(GtkDialog *self, gint , ezgl::application *) {
    gtk_widget_destroy(GTK_WIDGET(self));
}
// WeatherButton callback function. Display the current city weather & humidity
gboolean weather_button_clicked(GtkWidget *, gpointer data) {

    // setting up fow variables
    auto ezgl_app = static_cast<ezgl::application *>(data);
    ezgl_app->update_message("Displaying local weather");
    std::string url;

    // corner cases
    if (current_map.currentCity == "" || current_map.currentCity == "Select a map") { current_map.currentCity = "Toronto"; }
    else std::cerr << "Current City is: " + current_map.currentCity << std::endl;

    // create the API with current city name
    url = "http://api.weatherapi.com/v1/current.json?key=1dc8f2a5427e4c8ab93205516231203&q=" + current_map.currentCity + "&aqi=yes";

    // initialize libcurl
    CURL *curlHandle = curl_easy_init();
    if (!curlHandle) {
        std::cerr << "Error: failed to initialize libcurl" << std::endl;
        return TRUE;
    }

    // set libcurl options
    curl_easy_setopt(curlHandle, CURLOPT_URL, url.c_str());

    // create a string to hold the response
    std::string response;

    // set the callback function to write the response to the string
    curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curlHandle, CURLOPT_WRITEDATA, &response);

    // perform the request
    CURLcode result = curl_easy_perform(curlHandle);
    if (result != CURLE_OK)
    {
        std::cerr << "Error: curl_easy_perform() failed: " << curl_easy_strerror(result) << std::endl;
        curl_easy_cleanup(curlHandle);
    }

    // cleanup libcurl
    curl_easy_cleanup(curlHandle);
    curlHandle = nullptr;

    // parse the response as JSON using boost property_tree
    boost::property_tree::ptree pt;
    std::istringstream iss(response);
    try {
        boost::property_tree::read_json(iss, pt);
    } catch (const std::exception &ex) {
        std::cerr << "Error: failed to parse response as JSON: " << ex.what() << std::endl;
    }

    // extract the city name and temperature from the JSON
    std::string city_name = "";
    double tempInCel = 0.0;
    double humidity = 0.0;

    // API may fail, do a try catch
    try {
        city_name = pt.get<std::string>("location.name");
        tempInCel = pt.get<double>("current.temp_c");
        humidity = pt.get<double>("current.humidity");
    }
    catch (const std::exception &ex) {
        std::cerr << "Error: failed to extract API data: " << ex.what() << std::endl;
    }

    // display data to window
    std::string stringTempCel = std::to_string(tempInCel).substr(0, 4);
    std::string string_humidity = std::to_string(humidity).substr(0, 2);
    std::string toPrint = "  Temperature: " + stringTempCel + " °C." + "\n\n" + "  Humidity: " + string_humidity + "%  " + "\n\n";
    ezgl_app->create_dialog_window(dialog_window_callback, city_name.c_str(), toPrint.c_str());
    return TRUE;
}
// callback function for flipping the color scheme of the system
void toggle_dark(GtkWidget* themeButton, ezgl::application* application) {
    // flip status
    current_map.dark_mode = !current_map.dark_mode;
    // create output message and print to bottom left corner
    std::string themeMsg;
    if (current_map.dark_mode) {
        themeMsg = "Switched to night theme";
        gtk_button_set_label(reinterpret_cast<GtkButton *>(themeButton), "Light Mode");
    }
    else {
        themeMsg = "Switched to day theme";
        gtk_button_set_label(reinterpret_cast<GtkButton *>(themeButton), "Dark Mode");
    }
    application->update_message(themeMsg);
    // force a picture refreshing
    application->refresh_drawing();
}
// callback for enable and diable rangefinder
void toggle_rangeFinder(GtkWidget* rangeFinderButton, ezgl::application* application) {
    // flip status
    current_map.findingRange = !current_map.findingRange;
    // create output message and print to bottom left corner
    std::string themeMsg;
    if (current_map.findingRange) {
        themeMsg = "Range Finder Mode: ON";
         gtk_button_set_label(reinterpret_cast<GtkButton *>(rangeFinderButton), "Finding");
    }
    else {
        themeMsg = "Range Finder Mode: OFF";
         gtk_button_set_label(reinterpret_cast<GtkButton *>(rangeFinderButton), "Range Finder");
    }
    // clear total range, point_list for upcoming measurement
    current_map.total_range = 0;
    current_map.range_finder_point_list.clear();
    application->update_message(themeMsg);
    application->refresh_drawing();
}

gboolean help_button_clicked(GtkWidget *, gpointer data) {
    auto ezgl_app = static_cast<ezgl::application *>(data);
    // display data to window
    std::string help_info = "./libstreetmap/resources/help_info.txt";
    std::ifstream file(help_info);
    if (!file.is_open())
        std::cerr << "Could not open file: " << help_info << std::endl;
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string help_text = buffer.str();
    ezgl_app->create_dialog_window(dialog_window_callback, "Help", help_text.c_str());

    return TRUE;
}

// display path of range finder
void draw_range_finder_points(ezgl::renderer *g){
    // setup offset_point for text drawing
    ezgl::point2d offset_point(0, 10);
    if(current_map.range_finder_point_list.empty()){
        return;
    } else if(current_map.range_finder_point_list.size() == 1){
        // setup point color for single point case;
        if(current_map.dark_mode) {g->set_color(5, 114, 248, 200);}
        else{g->set_color(255, 0, 0, 200);}
        g->fill_arc(current_map.range_finder_point_list[0], 4, 0, 360);
        // setup text color
        if(current_map.dark_mode) {g->set_color(ezgl::WHITE);}
        else{g->set_color(ezgl::BLACK);}
        g->draw_text(current_map.range_finder_point_list[0] + offset_point, "Point 1");
        return;
    }
    // initialize helper variables
    ezgl::point2d point_from, point_to, mid_point;
    double distance, deltax, deltay, angle;
    std::string info_to_display = "";
    // draw the first point
    if(current_map.dark_mode) {g->set_color(5, 114, 248, 200);}
    else {g->set_color(255, 0, 0, 200);}
    g->fill_arc(current_map.range_finder_point_list[0], 5, 0, 360);
    // draw links between consecutive points
    for(int i=0; i<current_map.range_finder_point_list.size()-1; i++){
        point_from = current_map.range_finder_point_list[i];
        point_to = current_map.range_finder_point_list[i+1];
        mid_point = point_from + point_to;
        mid_point.x /= 2;
        mid_point.y /= 2;
        deltay = point_to.y - point_from.y;
        deltax = point_to.x - point_from.x;
        distance = sqrt(deltax*deltax + deltay*deltay);
        // round distance to tenth;
        distance = round(distance*10)/10;
        // setup point color
        if(current_map.dark_mode) {g->set_color(5, 114, 248, 200);}
        else {g->set_color(255, 0, 0, 200);}
        g->set_line_width(5);
        g->draw_line(point_from, point_to);
        g->fill_arc(point_to, 4, 0, 360);
        // erase unecessary zeros from the text and setup rotation degree
        info_to_display = std::to_string(distance);
        info_to_display.erase(info_to_display.find_last_not_of('0') + 1, std::string::npos);
        info_to_display = "Distance: " + info_to_display + "m";
        angle = atan((point_to.y - point_from.y) / (point_to.x - point_from.x));
        angle = angle*180/M_PI;
        // finding the position of the text
        offset_point.x = -deltay/8;
        offset_point.y = deltax/8;
        // set up text color
        if(current_map.dark_mode) {g->set_color(ezgl::WHITE);}
        else {g->set_color(ezgl::BLACK);}
        if(angle>=-360 && angle<=360){g->set_text_rotation(angle);}
        else {g->set_text_rotation(0);}
        g->set_font_size(16);
        g->draw_text(mid_point + offset_point, info_to_display, distance*0.8, INT_MAX);
    }
    // draw all points
    g->set_font_size(14);
    offset_point = {0, 10};
    for(int i=0; i<current_map.range_finder_point_list.size(); i++){
        info_to_display = "Point " + std::to_string(i+1);
        g->set_text_rotation(0);
        if(current_map.dark_mode) {g->set_color(ezgl::WHITE);}
        else {g->set_color(ezgl::BLACK);}
        g->draw_text(current_map.range_finder_point_list[i] + offset_point, info_to_display);
    }
}

// draw small street pieces (street segments or segment btw two curve points)
// is also responsible for generating street/street segment information (oneWay, width, length)
void draw_street_piece_from_two_points(ezgl::renderer *g, StreetSegmentIdx& street_segment_id, ezgl::point2d& point1, ezgl::point2d& point2){
    // retrieve ss info
    myStreetSegment SSInfo_ = mySegData[street_segment_id];
    double x1 = point1.x;
    double y1 = point1.y;
    double x2 = point2.x;
    double y2 = point2.y;
    double length, delta_x, delta_y;
    double width_x, width_y, width_piece;
    // find street piece length and setup street piece width based on street level
    delta_x = (x2 - x1);
    delta_y = (y2 - y1);
    length = sqrt((delta_x * delta_x) + (delta_y * delta_y));
    ezgl::point2d point1_, point2_, point3_, point4_;
    // weight represents information about the street level
    width_x = SSInfo_.weight*2*delta_y/length;
    width_y = SSInfo_.weight*2*delta_x/length;
    width_piece = 2*sqrt((width_x * width_x) + (width_y * width_y));
    point1_.x = (x1 + width_x);
    point2_.x = (x1 - width_x);
    point3_.x = (x2 - width_x);
    point4_.x = (x2 + width_x);
    point1_.y = (y1 - width_y);
    point2_.y = (y1 + width_y);
    point3_.y = (y2 + width_y);
    point4_.y = (y2 - width_y);
    std::vector<ezgl::point2d> point_list;
    point_list.emplace_back(point1_);
    point_list.emplace_back(point2_);
    point_list.emplace_back(point3_);
    point_list.emplace_back(point4_);
    // draw street segment
    g->set_color(color_list[SSInfo_.street_level][0], color_list[SSInfo_.street_level][1], color_list[SSInfo_.street_level][2]);
    g->fill_poly(point_list);
    g->fill_arc(point2, width_piece/2, 0, 360);
    // insert street infomation to be drawn
    street_piece_info new_street_piece;
    new_street_piece.width = width_piece;
    new_street_piece.length = length;
    new_street_piece.street_segment_id = street_segment_id;
    new_street_piece.point_to = point2;
    new_street_piece.point_from = point1;
    current_map.street_pieces_list.emplace_back(new_street_piece);
    return;
}

// draw one street segment
void draw_street_segment(ezgl::renderer *g, StreetSegmentIdx& street_segment_id){
    myStreetSegment SSInfo_ = mySegData[street_segment_id];
    // check curve point
    if(SSInfo_.numCurvePoints == 0){
        // ss with no curve point, one piece: from - to
        draw_street_piece_from_two_points(g, street_segment_id, SSInfo_.posFrom, SSInfo_.posTo); // modify here
    } else {
        // ss with curve points, multiple pieces: looping through
        ezgl::point2d previousPos = SSInfo_.posFrom;
        ezgl::point2d finalPos = SSInfo_.posTo;
        ezgl::point2d currentPos;
        LatLon LLtemp;
        for(int j=0; j < SSInfo_.numCurvePoints; j++){
            LLtemp = getStreetSegmentCurvePoint(street_segment_id, j);
            currentPos.x = x_from_lon(LLtemp.longitude());
            currentPos.y = y_from_lat(LLtemp.latitude());
            draw_street_piece_from_two_points(g, street_segment_id, previousPos, currentPos);
            previousPos = currentPos;
        }
        draw_street_piece_from_two_points(g, street_segment_id, previousPos, finalPos);
    }
    return;
}
// taking a list of ssid and draw the street
void draw_street_seg_from_list(ezgl::renderer *g, std::vector<StreetSegmentIdx> SSid_list){
    ezgl::rectangle visible_world = g->get_visible_world();
    ezgl::point2d posFrom, posTo;
    double minx_, miny_, maxx_, maxy_;
    myStreetSegment SSInfo_;
    for(int list_id=0; list_id < SSid_list.size(); list_id++){
        SSInfo_ = mySegData[SSid_list[list_id]];
        // find the position of the street and check if the street segment is visible first
        posFrom = SSInfo_.posFrom;
        posTo = SSInfo_.posTo;
        if(posFrom.x > posTo.x) {minx_ = posTo.x; maxx_ = posFrom.x;}
        else {minx_ = posFrom.x; maxx_ = posTo.x;}
        if(posFrom.y > posTo.y) {miny_ = posTo.y; maxy_ = posFrom.y;}
        else {miny_ = posFrom.y; maxy_ = posTo.y;}
        if(outsideVisible(visible_world, minx_-10, maxx_+10, miny_-10, maxy_+10)){ continue; }
        // check for street seg length information
        if((int) (current_map.scale_factor) / SSInfo_.segLength > 400){
            continue;
        } else if((int) (current_map.scale_factor) / findStreetLength(SSInfo_.streetID) > 80){
            continue;
        }
        draw_street_segment(g, SSid_list[list_id]);
    }
}

void path_finding_draw_street_seg_from_list(ezgl::renderer *g, std::vector<StreetSegmentIdx> SSid_list){
    for(int list_id=0; list_id < SSid_list.size(); list_id++){
        StreetSegmentIdx street_segment_id = SSid_list[list_id];
        myStreetSegment SSInfo_ = mySegData[street_segment_id];
        // check curve point
        if(SSInfo_.numCurvePoints == 0){
            // ss with no curve point, one piece: from - to
            if(sqrt(current_map.mapArea) / current_map.scale_factor < 12){
                if(current_map.dark_mode) {g->set_color(5, 114, 248);}
                else {g->set_color(255, 0, 0);}
                g->set_line_width(5);
                g->draw_line(SSInfo_.posFrom, SSInfo_.posTo);
            } else if(sqrt(current_map.mapArea) / current_map.scale_factor < 7){
                if(current_map.dark_mode) {g->set_color(5, 114, 248);}
                else {g->set_color(255, 0, 0);}
                g->set_line_width(2);
                g->draw_line(SSInfo_.posFrom, SSInfo_.posTo);
            } else {
                path_finding_draw_street_piece_from_two_points(g, street_segment_id, SSInfo_.posFrom, SSInfo_.posTo);
            }
        } else {
            // ss with curve points, multiple pieces: looping through
            ezgl::point2d previousPos = SSInfo_.posFrom;
            ezgl::point2d finalPos = SSInfo_.posTo;
            ezgl::point2d currentPos;
            LatLon LLtemp;
            for(int j=0; j < SSInfo_.numCurvePoints; j++){
                LLtemp = getStreetSegmentCurvePoint(street_segment_id, j);
                currentPos.x = x_from_lon(LLtemp.longitude());
                currentPos.y = y_from_lat(LLtemp.latitude());
                if(sqrt(current_map.mapArea) / current_map.scale_factor < 12){
                    if(current_map.dark_mode) {g->set_color(5, 114, 248);}
                    else {g->set_color(255, 0, 0);}
                    g->set_line_width(5);
                    g->draw_line(previousPos, currentPos);
                } else if(sqrt(current_map.mapArea) / current_map.scale_factor < 7){
                if(current_map.dark_mode) {g->set_color(5, 114, 248);}
                else {g->set_color(255, 0, 0);}
                g->set_line_width(2);
                g->draw_line(previousPos, currentPos);
                } else {
                    path_finding_draw_street_piece_from_two_points(g, street_segment_id, previousPos, currentPos);
                }
                previousPos = currentPos;
            }
            if(sqrt(current_map.mapArea) / current_map.scale_factor < 12){
                if(current_map.dark_mode) {g->set_color(5, 114, 248);}
                else {g->set_color(255, 0, 0);}
                g->set_line_width(5);
                g->draw_line(previousPos, finalPos);
            } else if(sqrt(current_map.mapArea) / current_map.scale_factor < 7){
                if(current_map.dark_mode) {g->set_color(5, 114, 248);}
                else {g->set_color(255, 0, 0);}
                g->set_line_width(2);
                g->draw_line(previousPos, finalPos);
            } else {
                path_finding_draw_street_piece_from_two_points(g, street_segment_id, previousPos, finalPos);
            }
        }
    }
}

// m3 drawing
void path_finding_draw_street_piece_from_two_points(ezgl::renderer *g, StreetSegmentIdx& street_segment_id, ezgl::point2d& point1, ezgl::point2d& point2){
    // retrieve ss info
    myStreetSegment SSInfo_ = mySegData[street_segment_id];
    double x1 = point1.x;
    double y1 = point1.y;
    double x2 = point2.x;
    double y2 = point2.y;
    double length, delta_x, delta_y;
    double width_x, width_y, width_piece;
    // find street piece length and setup street piece width based on street level
    delta_x = (x2 - x1);
    delta_y = (y2 - y1);
    length = sqrt((delta_x * delta_x) + (delta_y * delta_y));
    ezgl::point2d point1_, point2_, point3_, point4_;
    // weight represents information about the street level
    width_x = SSInfo_.weight*2*delta_y/length*0.9;
    width_y = SSInfo_.weight*2*delta_x/length*0.9;
    width_piece = 2*sqrt((width_x * width_x) + (width_y * width_y));
    point1_.x = (x1 + width_x);
    point2_.x = (x1 - width_x);
    point3_.x = (x2 - width_x);
    point4_.x = (x2 + width_x);
    point1_.y = (y1 - width_y);
    point2_.y = (y1 + width_y);
    point3_.y = (y2 + width_y);
    point4_.y = (y2 - width_y);
    std::vector<ezgl::point2d> point_list;
    point_list.emplace_back(point1_);
    point_list.emplace_back(point2_);
    point_list.emplace_back(point3_);
    point_list.emplace_back(point4_);
    // draw street segment
    if(current_map.dark_mode) {g->set_color(5, 114, 248);}
    else {g->set_color(255, 0, 0);}
    g->fill_poly(point_list);
    g->fill_arc(point2, width_piece/2, 0, 360);
    // insert street infomation to be drawn
    street_piece_info new_street_piece;
    new_street_piece.width = width_piece;
    new_street_piece.length = length;
    new_street_piece.street_segment_id = street_segment_id;
    new_street_piece.point_to = point2;
    new_street_piece.point_from = point1;
    current_map.street_pieces_list.emplace_back(new_street_piece);
    return;
}

// simpler version of street segment drawing for high camera view (faster but less acurate)
void draw_simple_street_piece_from_two_points(ezgl::renderer *g, StreetSegmentIdx& street_segment_id, ezgl::point2d& point1, ezgl::point2d& point2){
    myStreetSegment SSInfo_ = mySegData[street_segment_id];
    g->set_color(color_list[SSInfo_.street_level][0], color_list[SSInfo_.street_level][1], color_list[SSInfo_.street_level][2]);
    g->set_line_width(0);
    g->draw_line(point1, point2);
    return;
}

// simpler version of drawing one street segment
void draw_simple_street_segment(ezgl::renderer *g, StreetSegmentIdx& street_segment_id){
    myStreetSegment SSInfo_ = mySegData[street_segment_id];
    if(SSInfo_.numCurvePoints == 0){
        // ss with no curve point
        draw_simple_street_piece_from_two_points(g, street_segment_id, SSInfo_.posFrom, SSInfo_.posTo); // modify here
    } else {
        // ss with curve points
        ezgl::point2d previousPos = SSInfo_.posFrom;
        ezgl::point2d finalPos = SSInfo_.posTo;
        ezgl::point2d currentPos;
        LatLon LLtemp;
        for(int j=0; j < SSInfo_.numCurvePoints; j++){
            LLtemp = getStreetSegmentCurvePoint(street_segment_id, j);
            currentPos.x = x_from_lon(LLtemp.longitude());
            currentPos.y = y_from_lat(LLtemp.latitude());
            draw_simple_street_piece_from_two_points(g, street_segment_id, previousPos, currentPos);
            previousPos = currentPos;
        }
        draw_simple_street_piece_from_two_points(g, street_segment_id, previousPos, finalPos);
    }
    return;
}

// simpler version of drawing from a list of ssid
void draw_simple_street_seg_from_list(ezgl::renderer *g, std::vector<StreetSegmentIdx> SSid_list){
    ezgl::rectangle visible_world = g->get_visible_world();
    ezgl::point2d posFrom, posTo;
    double minx_, miny_, maxx_, maxy_;
    myStreetSegment SSInfo_;
    for(int list_id=0; list_id < SSid_list.size(); list_id++){
        SSInfo_ = mySegData[SSid_list[list_id]];
        // find the position of the street and check if the street segment is visible first
        posFrom = SSInfo_.posFrom;
        posTo = SSInfo_.posTo;
        if(posFrom.x > posTo.x) {minx_ = posTo.x; maxx_ = posFrom.x;}
        else {minx_ = posFrom.x; maxx_ = posTo.x;}
        if(posFrom.y > posTo.y) {miny_ = posTo.y; maxy_ = posFrom.y;}
        else {miny_ = posFrom.y; maxy_ = posTo.y;}
        // make the condition more strict for getting outside of the visible world
        if(outsideVisible(visible_world, minx_-10, maxx_+10, miny_-10, maxy_+10)){ continue; }
        draw_simple_street_segment(g, SSid_list[list_id]);
    }
}

// draw all street segments
void draw_streetSeg(ezgl::renderer *g){
    // auto startTime = std::chrono::high_resolution_clock::now();
    // color setup
    g->set_line_dash(ezgl::line_dash::none);
    // clear street pieces list for direction and street name drawing
    current_map.street_pieces_list.clear();

    // check for the scale factor first: drawing streets based on street importance
    // only draw major street when showing more than 1/3 of the map (high camera)
    if(sqrt(current_map.mapArea) / current_map.scale_factor < 3){
        draw_simple_street_seg_from_list(g, current_map.trunk_street_seg);
        draw_simple_street_seg_from_list(g, current_map.motorway_street_seg);
        // auto curTime = std::chrono::high_resolution_clock::now();
        // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
        // std::cout << "draw_streetSeg took " << wallClock.count() << " seconds" << std::endl;
    } else if(sqrt(current_map.mapArea) / current_map.scale_factor < 7){
        draw_simple_street_seg_from_list(g, current_map.secondary_street_seg);
        draw_simple_street_seg_from_list(g, current_map.primary_street_seg);
        draw_simple_street_seg_from_list(g, current_map.trunk_street_seg);
        draw_simple_street_seg_from_list(g, current_map.motorway_street_seg);
        // auto curTime = std::chrono::high_resolution_clock::now();
        // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
        // std::cout << "draw_streetSeg took " << wallClock.count() << " seconds" << std::endl;
    } else if(sqrt(current_map.mapArea) / current_map.scale_factor < 10){
        draw_simple_street_seg_from_list(g, current_map.general_street_seg);
        draw_street_seg_from_list(g, current_map.secondary_street_seg);
        draw_street_seg_from_list(g, current_map.primary_street_seg);
        draw_street_seg_from_list(g, current_map.trunk_street_seg);
        draw_street_seg_from_list(g, current_map.motorway_street_seg);
        // auto curTime = std::chrono::high_resolution_clock::now();
        // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
        // std::cout << "draw_streetSeg took " << wallClock.count() << " seconds" << std::endl;
    } else {
        draw_street_seg_from_list(g, current_map.general_street_seg);
        draw_street_seg_from_list(g, current_map.secondary_street_seg);
        draw_street_seg_from_list(g, current_map.primary_street_seg);
        draw_street_seg_from_list(g, current_map.trunk_street_seg);
        draw_street_seg_from_list(g, current_map.motorway_street_seg);
        // auto curTime = std::chrono::high_resolution_clock::now();
        // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
        // std::cout << "draw_streetSeg took " << wallClock.count() << " seconds" << std::endl;;
    }
    if (current_map.pathFindingMode) path_finding_draw_street_seg_from_list(g, current_map.pathSegment);
}

// draw street pieces info: oneway, street name
void draw_street_pieces_info(ezgl::renderer *g){
    // auto startTime = std::chrono::high_resolution_clock::now();
    // check for camera first
    if(sqrt(current_map.mapArea) / current_map.scale_factor < 15){
        return;
    }
    // helper variable initialization
    street_piece_info piece_temp;
    myStreetSegment seg_info_temp;
    bool oneWay;
    double angle;
    std::string street_name;
    std::string text_to_display;
    ezgl::point2d center_pos;
    // traverse through all street_pieces
    for(int street_piece_id=0; street_piece_id<current_map.street_pieces_list.size(); street_piece_id++){
        piece_temp = current_map.street_pieces_list[street_piece_id];
        seg_info_temp = mySegData[piece_temp.street_segment_id];
        oneWay = seg_info_temp.oneWay;
        street_name = getStreetName(seg_info_temp.streetID);
        // udpate information to display for valid street names
        if(street_name != "<unknown>"){
            text_to_display = street_name;
            if(!oneWay){
                text_to_display = "<- " + text_to_display + " ->";
            } else {
                if(piece_temp.point_to.x - piece_temp.point_from.x < 0){
                    text_to_display = "<- " + text_to_display;
                } else {
                    text_to_display = text_to_display + " ->";
                }
            } 
            // setup correct roation from -90 to 270
            angle = atan((piece_temp.point_to.y - piece_temp.point_from.y) / (piece_temp.point_to.x - piece_temp.point_from.x));
            angle = angle*180/M_PI;
            if(angle>=-360 && angle<=360){g->set_text_rotation(angle);}
            else {g->set_text_rotation(0);}
            g->set_font_size(seg_info_temp.street_level);
            if(current_map.dark_mode){g->set_color(ezgl::WHITE);}
            else {g->set_color(ezgl::BLACK);}
            center_pos.x = (piece_temp.point_to.x + piece_temp.point_from.x)/2;
            center_pos.y = (piece_temp.point_to.y + piece_temp.point_from.y)/2;
            g->draw_text(center_pos, text_to_display, piece_temp.length*1.2, piece_temp.width*1.8);
            continue;
        } else {
            // udpate information to display for invalid street names
            angle = atan((piece_temp.point_to.y - piece_temp.point_from.y) / (piece_temp.point_to.x - piece_temp.point_from.x));
            angle = angle*180/M_PI;
            if(!oneWay){
                text_to_display = "<-  ->";
            } else {
                if(piece_temp.point_to.x - piece_temp.point_from.x < 0){
                    text_to_display = " <- ";
                } else {
                    text_to_display = " -> ";
                }
            }
            // setup correct roation from -90 to 270
            if(angle>=-360 && angle<=360){g->set_text_rotation(angle);}
            else {g->set_text_rotation(0);}
            g->set_font_size(seg_info_temp.street_level);
            if(current_map.dark_mode){g->set_color(ezgl::WHITE);}
            else {g->set_color(ezgl::BLACK);}
            center_pos.x = (piece_temp.point_to.x + piece_temp.point_from.x)/2;
            center_pos.y = (piece_temp.point_to.y + piece_temp.point_from.y)/2;
            g->draw_text(center_pos, text_to_display, piece_temp.length*1.1, piece_temp.width*1.8);
            continue;
        }
    }
    // disable timer
    // auto curTime = std::chrono::high_resolution_clock::now();
    // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    // std::cout << "draw street info took " << wallClock.count() << " seconds" << std::endl;
}

// draw intersections marked as highlighted
void draw_intersection(ezgl::renderer *g){
    // auto startTime = std::chrono::high_resolution_clock::now();
    ezgl::point2d inter_loc;
    float radius = 4;
    if(current_map.dark_mode) {g->set_color(5, 114, 248, 200);}
    else {g->set_color(255, 0, 0, 200);}

    if (current_map.pathFindingMode) {
        if (current_map.startPoint != -1) {
            g->set_color(0, 255, 0, 200);
            g->fill_arc(streetIntersectionData[current_map.startPoint].xy_loc, radius, 0, 360);
        }
        if (current_map.endPoint != -1) {
            g->set_color(0, 0, 255, 200);
            g->fill_arc(streetIntersectionData[current_map.endPoint].xy_loc, radius, 0, 360);
        }
        return;
    }

    for(IntersectionIdx inter_id : current_map.highlighted_inter){
        inter_loc = streetIntersectionData[inter_id].xy_loc;
        g->fill_arc(inter_loc, radius, 0, 360);
    }
    // auto curTime = std::chrono::high_resolution_clock::now();
    // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    // std::cout << "draw_intersection took " << wallClock.count() << " seconds" << std::endl;
}

// draw features
void draw_features(ezgl::renderer *g){
    // auto startTime = std::chrono::high_resolution_clock::now();
    ezgl::rectangle visible_world = g->get_visible_world();
    // feature list is sorted based on feature area descending
    for(int list_id = 0; list_id < myFeatures.size(); list_id++){
        myFeatureData feature_data = myFeatures[list_id];
        int numOfPoints = feature_data.numOfPoints;
        // don't draw single point features
        if(numOfPoints == 1){continue;}
        // don't draw feature smaller than 1/100 of the visible world
        // as the list is sorted, return the first time of encountering
        // feature < 1/100 of the area of the visible world
        double ratio = current_map.scale_factor/sqrt(feature_data.featureArea);
        if(ratio > 200){
            // auto curTime = std::chrono::high_resolution_clock::now();
            // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
            // std::cout << "draw_features took " << wallClock.count() << " seconds" << std::endl;
            return;
        }
        // skip features outside of the world
        // make the condition more strict for getting outside of the visible world
        if(outsideVisible(visible_world, feature_data.minx_-10, feature_data.maxx_+10, feature_data.miny_-10, feature_data.maxy_+10)){
            continue;
        }
        // check if using dot to represent features
        if(ratio > 60){ draw_simple_feature(g, list_id); continue;}
        if (feature_data.openFeature) {draw_open_feature(g, list_id);}
        else {draw_closed_feature(g, list_id);}
    }
    // auto curTime = std::chrono::high_resolution_clock::now();
    // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    // std::cout << "draw_features took " << wallClock.count() << " seconds" << std::endl;
}

// draw a dot to represent features at high camera
void draw_simple_feature(ezgl::renderer *g, int list_id){
    myFeatureData feature_data_temp = myFeatures[list_id];
    FeatureType feature_type = feature_data_temp.feature_type;
    g->set_color(color_list[feature_type][0], color_list[feature_type][1], color_list[feature_type][2]);
    ezgl::point2d cur_p;
    cur_p.x = (feature_data_temp.minx_ + feature_data_temp.maxx_) / 2;
    cur_p.y = (feature_data_temp.miny_ + feature_data_temp.maxy_) / 2;
    g->fill_arc(cur_p, 6, 0, 360);
}

// draw closed features
void draw_closed_feature(ezgl::renderer *g, int list_id){
    myFeatureData feature_data_temp = myFeatures[list_id];
    FeatureType feature_type = feature_data_temp.feature_type;
    g->set_color(color_list[feature_type][0], color_list[feature_type][1], color_list[feature_type][2]);
    g->fill_poly(feature_data_temp.point_list);
}

// draw open features
void draw_open_feature(ezgl::renderer *g, int list_id){
    myFeatureData feature_data_temp = myFeatures[list_id];
    FeatureType feature_type = feature_data_temp.feature_type;
    g->set_color(color_list[feature_type][0], color_list[feature_type][1], color_list[feature_type][2]);
    g->set_line_width(5);
    g->set_line_dash(ezgl::line_dash::none);
    int numOfPoints = feature_data_temp.numOfPoints;
    // travers through the list to draw the open feature
    ezgl::point2d previousPos = feature_data_temp.point_list[0];
    ezgl::point2d curPos;
    for(int p_index = 1; p_index<numOfPoints; p_index++){
        curPos = feature_data_temp.point_list[p_index];
        g->draw_line(previousPos, curPos);
        previousPos = curPos;
    }
}

// check if a rectangle defined by (minx, miny), (maxx, maxy) is visible
bool outsideVisible(const ezgl::rectangle &visible_world, double minx_, double maxx_, double miny_, double maxy_){
    if(minx_ > visible_world.right()){return true;}
    if(miny_ > visible_world.top()){return true;}
    if(maxx_ < visible_world.left()){return true;}
    if(maxy_ < visible_world.bottom()){return true;}
    return false;
}

// callback for loading another map 
void combo_box_change_map(GtkComboBoxText* self, ezgl::application *application){
    auto text = gtk_combo_box_text_get_active_text(self);
    current_map.currentCity = text;
    if(!text){
        free(text);
        text = NULL;
        return;
    } else {
        application->update_message(text);
        // first check select a map option
        std::string new_map = text;
        if(new_map == "Select a map"){ free(text); text = NULL; return; }
        // pre-process string for loadMap()
        boost::to_lower(new_map);
        new_map.erase(std::remove_if(new_map.begin(), new_map.end(), ::isspace), new_map.end());
        new_map = mapNames[new_map];
        // swtich to the same map as the current one?
        if(current_map.current_map_name == new_map){
            std::cout << "Same map, not switching..." << std::endl;
            free(text); return;
        }
        // general case: switch to a different map
        current_map.current_map_name = new_map;
        std::cout << "Switch to: " << text << std::endl;
        // clear data structures and reload a new one
        current_map.highlighted_inter.clear();
        free(text);
        closeMap();
        new_map = "/cad2/ece297s/public/maps/" + new_map + ".streets.bin";
        loadMap(new_map);
        std::cout << "Successfully loaded map '" << new_map << "'\n";
        ezgl::rectangle new_world({current_map.minX, current_map.minY},
                                  {current_map.maxX, current_map.maxY});
        // reset the pathFindingMode
        auto *pathModeSwitch = application->get_object("PathModeSwitch");
        gtk_switch_set_state(reinterpret_cast<GtkSwitch *>(pathModeSwitch), FALSE);
        application->change_canvas_world_coordinates("MainCanvas", new_world);
        application->refresh_drawing();
    }
}

// show all POI names
void show_POI_name(ezgl::renderer *g){
    // auto startTime = std::chrono::high_resolution_clock::now();
    // check if showing less than 1/12 of the whole map
    // if not(high camera), return directly
    if(sqrt(current_map.mapArea) / current_map.scale_factor < 12){
        // auto curTime = std::chrono::high_resolution_clock::now();
        // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
        // std::cout << "draw POI took " << wallClock.count() << " seconds" << std::endl;
        return;
    }
    int numOfPOI_display = 0;
    // define how many POI draw in different zoom level
    if (sqrt(current_map.mapArea) / current_map.scale_factor < 15) {numOfPOI_display = 10;}
    else if (sqrt(current_map.mapArea) / current_map.scale_factor < 20) {numOfPOI_display = 15;}
    else{numOfPOI_display = 20;}
    // helper variables initialization
    int POIdrawn = 0;
    ezgl::rectangle visible_world = g->get_visible_world();
    std::unordered_set<POIIdx> poi_temp;
    if(!current_map.cur_POI.empty()){
        for(auto itr = current_map.cur_POI.begin(); itr!=current_map.cur_POI.end(); itr++){
            LatLon ll_pos = getPOIPosition(*itr);
            ezgl::point2d poi_pos;
            poi_pos.x = x_from_lon(ll_pos.longitude());
            poi_pos.y = y_from_lat(ll_pos.latitude());
            if (outsideVisible(visible_world, poi_pos.x, poi_pos.x, poi_pos.y, poi_pos.y)) {
                continue;
            }
            draw_single_poi(g, *itr);
            poi_temp.insert(*itr);
            POIdrawn++;
        }
    }
    // A trivial shortcut (if none of the POI on the screen goes out of it)
    if(POIdrawn>=numOfPOI_display){ current_map.cur_POI = poi_temp; return; }

    // travers through all POIs but only draw part of them
    int numOfPOIs = getNumPointsOfInterest();
    for (int POI_id=0; POI_id<numOfPOIs; POI_id+=5) {
        // return if have drawn enough pois
        if(POIdrawn >= numOfPOI_display){
            // auto curTime = std::chrono::high_resolution_clock::now();
            // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
            // std::cout << "draw POI took " << wallClock.count() << " seconds" << std::endl;
            current_map.cur_POI = poi_temp;
            return;
        }
        // check if the poi has already been drawn
        if(poi_temp.find(POI_id) != poi_temp.end()){ continue; }
        LatLon cur_pos = getPOIPosition(POI_id);
        ezgl::point2d poi_pos;
        std::string poi_name;
        poi_pos.x = x_from_lon(cur_pos.longitude());
        poi_pos.y = y_from_lat(cur_pos.latitude());
        // check if visible
        if (outsideVisible(visible_world, poi_pos.x, poi_pos.x, poi_pos.y, poi_pos.y)) {
            continue;
        }
        draw_single_poi(g, POI_id);
        poi_temp.insert(POI_id);
        POIdrawn++;
    }
    
    // auto curTime = std::chrono::high_resolution_clock::now();
    // auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (curTime - startTime);
    // std::cout << "draw POI took " << wallClock.count() << " seconds" << std::endl;
    current_map.cur_POI = poi_temp;
    return;
}

void draw_single_poi(ezgl::renderer *g, POIIdx POI_id){
    std::string poi_type, poi_name;
    LatLon cur_pos = getPOIPosition(POI_id);
    ezgl::point2d poi_pos;
    poi_pos.x = x_from_lon(cur_pos.longitude());
    poi_pos.y = y_from_lat(cur_pos.latitude());
    poi_type = getPOIType(POI_id);
    poi_name = getPOIName(POI_id);
    // check type for icons
    if (poi_type == "restaurant" || poi_type == "fast_food" || poi_type == "cafe" ||
        poi_type == "ice_cream" || poi_type == "bar" || poi_type == "internet_cafe"){
        if(restaurant_png!=NULL){
            g->draw_surface(restaurant_png, poi_pos, getPOIIconScale());
        }
    } else if (poi_type == "prep_school" || poi_type == "school" || poi_type  == "college" ||
               poi_type == "music_school" || poi_type == "kindergarten" || poi_type == "library"){
        if(school_png!=NULL){
            g->draw_surface(school_png, poi_pos, getPOIIconScale());
        }
    } else if (poi_type == "dentist" || poi_type == "pharmacy" || poi_type == "clinic" || poi_type == "doctors" ) {
        if(health_care_png!=NULL){
            g->draw_surface(health_care_png, poi_pos, getPOIIconScale());
        }
    } else if (poi_type == "bus_station" || poi_type == "station" || poi_type == "subway" ) {
        if(transit_png!=NULL){
            g->draw_surface(transit_png, poi_pos, getPOIIconScale());
        }
    } else {
        if(regular_png!=NULL){
            g->draw_surface(regular_png, poi_pos, getPOIIconScale());
        }
    }
    // setup text drawing
    poi_pos.y = poi_pos.y - poi_name.size();
    if (current_map.dark_mode){ g->set_color(255, 128, 0, 244); }
    else { g->set_color(0, 100, 0, 244); }
    g->set_text_rotation(0);
    g->set_font_size(13);
    g->draw_text(poi_pos , poi_name, 200, 14);
}

double getPOIIconScale(){
    if(sqrt(current_map.mapArea)/current_map.scale_factor/30 < 0.9){ return 0.9; }
    if(sqrt(current_map.mapArea)/current_map.scale_factor/30 > 1.3){ return 1.3; }
    return sqrt(current_map.mapArea)/current_map.scale_factor/30;
}
// M3 path finding mode
gboolean pathModeSwitch_toggled(GtkWidget *, gboolean state, ezgl::application *data) {
    // update path finding mode
    current_map.pathFindingMode = state;
    auto *searchButton = data->get_object("SearchButton");
    if (!state) {
        // reset path finding mode data & flags
        current_map.pathSegment.clear();
        current_map.startPoint = current_map.endPoint = -1;
        gtk_button_set_label(reinterpret_cast<GtkButton *>(searchButton), "Search");
        gtk_list_store_clear((GtkListStore *)data->get_object("DirectionListStore"));
    }
    else {
        // change the search button text to "begin"
        gtk_button_set_label(reinterpret_cast<GtkButton *>(searchButton), "Begin");
    }
    data->refresh_drawing();
    return FALSE;
}

