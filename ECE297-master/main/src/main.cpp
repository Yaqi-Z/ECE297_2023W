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
#include <string>
#include <chrono>
#include <map>
#include <boost/algorithm/string.hpp>

#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m1_extend.h"
#include "m2_extend.h"
#include "m3_extend.h"

//Program exit codes
constexpr int SUCCESS_EXIT_CODE = 0;        //Everyting went OK
constexpr int ERROR_EXIT_CODE = 1;          //An error occured
constexpr int BAD_ARGUMENTS_EXIT_CODE = 2;  //Invalid command-line usage

//The default map to load if none is specified
std::string default_map_path = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";
//map names to file names
extern std::map<std::string, std::string> mapNames;
extern myMap current_map;

// The start routine of your program (main) when you are running your standalone
// mapper program. This main routine is *never called* when you are running 
// ece297exercise (the unit tests) -- those tests have their own main routine
// and directly call your functions in /libstreetmap/src/ to test them.
// Don't write any code in this file that you want run by ece297exerise -- it 
// will not be called!
int main(int argc, char** argv) {
    if(mapNames.empty()){
        loadMapNames();
    }

    std::string map_path;
    if(argc == 1) {
        //Use a default map
        map_path = default_map_path;
        current_map.current_map_name = "toronto_canada";
    } else if (argc == 2) {
        //Get the map from the command line
        // omit case, space, '-', and '_'
        map_path = argv[1];
        boost::to_lower(map_path);
        map_path.erase(std::remove_if(map_path.begin(), map_path.end(), ::isspace), map_path.end());
        map_path.erase(std::remove(map_path.begin(), map_path.end(), '-'), map_path.end());
        map_path.erase(std::remove(map_path.begin(), map_path.end(), '_'), map_path.end());
        if(mapNames.find(map_path) == mapNames.end()){
            std::cerr << "Failed to load map '" << map_path << "'\n";
            std::cout << "Please use command: 'mapper' or 'mapper [CityName]'; ";
            std::cout << "[CityName] must be without space; [CityName] is case insensitive; '-' or '_' will be ignored." << std::endl;
            return ERROR_EXIT_CODE;
        } else {
            //retrieve the correct map_path and udpate current_map
            map_path = mapNames[map_path];
            current_map.current_map_name = map_path;
            map_path = "/cad2/ece297s/public/maps/" + map_path + ".streets.bin";
        }
    } else {
        //Invalid arguments
        std::cerr << "Usage: " << argv[0] << " [map_file_path]\n";
        std::cerr << "  If no map_file_path is provided a default map is loaded.\n";
        return BAD_ARGUMENTS_EXIT_CODE;
    }

    //Load the map and related data structures
    bool load_success = loadMap(map_path);
    if(!load_success) {
        std::cerr << "Failed to load map '" << map_path << "'\n";
        return ERROR_EXIT_CODE;
    }
    std::cout << "Successfully loaded map '" << map_path << "'\n";

    //You can now do something with the map data
    drawMap();
    //Clean-up the map data and related data structures
    closeMap(); 
    
    return SUCCESS_EXIT_CODE;
}
