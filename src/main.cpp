#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

// Helper function to handle user input and check boundaries.
std::vector<float> getUserInput(float start_x, float start_y, float end_x, float end_y)
{
    bool isCorrect = false;

    while(!isCorrect)
    {
        std::cout << "\n";
        std::cout << "Insert the (x,y) coordinates relative to the inicial point. Only values between 0 - 100" << std::endl;
        std::cout << "x: ";
        std::cin >> start_x;
        std::cout << "y: ";
        std::cin >> start_y;

        std::cout << "Insert the (x,y) coordinates relative to the goal point. Only values between 0 - 100" << std::endl;
        std::cout << "x: ";
        std::cin >> end_x;
        std::cout << "y: ";
        std::cin >> end_y;
        std::cout << "\n";

        if( (start_x >= 0 && start_x <= 100) && (start_y >= 0 && start_y <= 100) && (end_x >= 0 && end_x <= 100) && (end_y >= 0 && end_y <= 100) )
        {
            isCorrect = true;
        }
        else
        {
            std::cout << "Please check your boundaries! Only values between 0 - 100" << std::endl;
            std::cout << "start_x: " << start_x << std::endl;
            std::cout << "start_y: " << start_y << std::endl;
            std::cout << "end_x: " << end_x << std::endl;
            std::cout << "end_y: " << end_y << std::endl;
        }
    }

    return std::vector<float>{start_x, start_y, end_x, end_y};
}

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    float start_x = -1.0f;
    float start_y = -1.0f;
    float end_x = -1.0f;
    float end_y = -1.0f;

    auto userInput = getUserInput(start_x, start_y, end_x, end_y);
    start_x = userInput[0];
    start_y = userInput[1];
    end_x = userInput[2];
    end_y = userInput[3];
    // std::cout << start_x << ", " << start_y << ", " << end_x << ", " << end_y << "\n";

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
