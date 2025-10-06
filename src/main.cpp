// Argument input
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/program_options.hpp>
#include <cstdlib>

// For printing to terminal
#include <filesystem>
#include <iostream>

// ----------------------------------------------------------- GLOBAL PARAMETERS -----------------------------------------------------------

// OSRM
#include "OSRM_Engine.h"
#include "OSRMParameters.h"

// Termination handling
#include <csignal>

// for easier use
using std::cout;
using std::endl;
using std::string;

// --------------------------------------------------------------------- MAIN ---------------------------------------------------------------------
int main(int argumentCount, char *argumentVariables[]) {
    // Define program options
    boost::program_options::options_description argumentDescription("Allowed options:");
    argumentDescription.add_options() // description of the arguments
        ("help", "Produces help message.")
        ("osrm-path", boost::program_options::value<std::string>(), "Path to OSRM data, this should end with '.osrm' (e.g. '/osrm/belgium/belgium.osrm').")
        ("coordinates-path", boost::program_options::value<std::string>(), "Path to coordinates, this should be a .txt file (e.g. '/data/coordinates.txt').")
    ;

    // variables to read in the program options
    boost::program_options::variables_map variableMap;
    boost::program_options::store(boost::program_options::parse_command_line(argumentCount, argumentVariables, argumentDescription), variableMap);
    boost::program_options::notify(variableMap);

    // help function
    if (variableMap.count("help")) {
        cout << argumentDescription << endl;
        return 1;
    }

    // osrm struct
    osrm_params OSRM;

    // OSRM path
    if (variableMap.count("osrm-path")) {
        OSRM.pathTo_OSM_data = variableMap["osrm-path"].as<string>();
    }
    else throw std::invalid_argument("No path to OSRM data provided, use --osrm-path to provide it.");

    // coordinates path
    if(variableMap.count("coordinates-path")) {
        OSRM.pathTO_coordinates = variableMap["coordinates-path"].as<string>();
        cout << "-------- Loading coordinates from file: " << OSRM.pathTO_coordinates << endl;
    }   
    else {
        cout << "-------- No path to coordinates provided, using random sampling." << endl;
        OSRM.sample_locations_in_belgium(100); // sample 100 random locations in Belgium if no coordinates file is provided
        // Save sampled coordinates for reproducibility
        if (OSRM.save_coordinates_to_file("results/coordinates.txt")) {
            cout << "Sampled coordinates written to results/coordinates.txt" << endl;
        }
    }

    // start the engine
    OSRM.start_engine();

    // Do osrm calculations
    calculate_osrm_metrics(OSRM);

    // Flush standard streams to ensure output is written.
    std::cout.flush();
    std::cerr.flush();

    // On some macOS setups libtbb's static destructors still crash during process teardown.
    // To avoid that, terminate the process immediately without running remaining static
    // destructors using std::_Exit(0). This avoids the libtbb destructor crash at the cost
    // of not running further C++ destructors (which we've already run for OSRM above).
    std::_Exit(0);
}
