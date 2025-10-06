#ifndef OSRM_PARAMS_H
#define OSRM_PARAMS_H

// osrm libs
#include "osrm/engine_config.hpp"
#include "osrm/osrm.hpp"

// std libs
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <sstream>
#include <utility>
#include <random>
#include <functional>
#include <filesystem>
#include <iomanip>

struct osrm_params {
    // ---------------------------------------------------------- OSRM VARS ----------------------------------------------------
    std::string pathTo_OSM_data = ""; // Path to OSRM data (if given in argument, this is overridden)
    std::string pathTO_coordinates = ""; // Path to coordinates (if given in argument, this is overridden)

    osrm::EngineConfig config;          // Global Osrm configuration
    std::unique_ptr<osrm::OSRM> engine; // Global Osrm engine (pointer)

    int max_threads = 1;                           // maximum number of threads, will be determined later, initialized at 1
    const int equal_max_distance_havesine = 100; // Max haversine distance we consider two coordinates to be the same place

    int Number_of_locations = 0; // Number of locations
    int **TravelTimes = nullptr;       // Travel times between needed locations
    int **TravelDistances = nullptr;   // Travel distances between needed locations

    // Parsed coordinates (latitude, longitude) read from the file at `pathTO_coordinates`.
    std::vector<std::pair<double, double>> coordinates;

    bool sampledCoordinates = false; // whether the coordinates were sampled (true) or loaded from file (false)

    // Constructor
    osrm_params() {};

    // Destructor
    ~osrm_params() {
        // Free TravelTimes
        if (TravelTimes != nullptr) {
            for (int i = 0; i < Number_of_locations; ++i) {
                if (TravelTimes[i] != nullptr) {
                    delete[] TravelTimes[i];
                    TravelTimes[i] = nullptr;
                }
            }
            delete[] TravelTimes;
            TravelTimes = nullptr;
        }

        // Free TravelDistances
        if (TravelDistances != nullptr) {
            for (int i = 0; i < Number_of_locations; ++i) {
                if (TravelDistances[i] != nullptr) {
                    delete[] TravelDistances[i];
                    TravelDistances[i] = nullptr;
                }
            }
            delete[] TravelDistances;
            TravelDistances = nullptr;
        }

        // Reset engine unique_ptr to release OSRM internal resources before static destructors run
        if (engine) {
            engine.reset();
        }

        std::cout << "OSRM resources cleaned up." << std::endl;
    }
 
    // Load coordinates from a text file. Each line should contain two whitespace-separated numbers
    // (latitude and longitude). If `path` is empty, `pathTO_coordinates` member is used.
    // Returns true on success, false otherwise. On success `coordinates` is populated and
    // `Number_of_locations` is updated.
    inline bool load_coordinates_from_file(const std::string &path = "") {
        const std::string file = path.empty() ? pathTO_coordinates : path;
        if (file.empty()) {
            std::cerr << "No coordinates file path provided\n";
            return false;
        }

        std::ifstream in(file);
        if (!in.is_open()) {
            std::cerr << "Failed to open coordinates file: " << file << std::endl;
            return false;
        }

        coordinates.clear();
        std::string line;
        while (std::getline(in, line)) {
            // Skip empty lines
            if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;

            std::istringstream ss(line);
            double a, b;
            if (!(ss >> a >> b)) {
                std::cerr << "Skipping malformed coordinate line: '" << line << "'" << std::endl;
                continue;
            }
            coordinates.emplace_back(a, b);
        }

        return true;
    }


    // Simple ray-casting point-in-polygon test (returns true if point is inside polygon).
    inline bool point_in_polygon(double lon, double lat, const std::vector<std::pair<double,double>> &poly) {
        bool inside = false;
        int n = static_cast<int>(poly.size());
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = poly[i].first, yi = poly[i].second;
            double xj = poly[j].first, yj = poly[j].second;

            bool intersect = ((yi > lat) != (yj > lat)) &&
                            (lon < (xj - xi) * (lat - yi) / (yj - yi + 0.0) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }
    // Sample `count` random points inside Belgium and populate OSRM.coordinates.
    // This uses a simplified Belgium polygon and a bounding box for generation.
    // Note: OSRM.coordinates stores pairs as (longitude, latitude) to match the rest of the code.
    inline void sample_locations_in_belgium(int count) {
        if (count <= 0) {
            std::cerr << "sample_locations_in_belgium: count must be > 0" << std::endl;
            return;
        }

        // Smaller central-Belgium polygon (lon, lat) to keep sampled points within a reliable area.
        // This covers the central region (around Brussels / Antwerp corridor) and is intentionally
        // smaller than the full-country polygon to reduce out-of-bounds samples.
        const std::vector<std::pair<double,double>> belgium_poly = {
            {3.8, 50.8}, {4.6, 50.8}, {5.1, 50.95}, {4.9, 51.25}, {4.2, 51.25}, {3.7, 51.05}
        };

        // Tight bounding box around the smaller polygon (lon, lat)
        const double lon_min = 3.7, lon_max = 5.1;
        const double lat_min = 50.7, lat_max = 51.3;

        std::random_device rd;
        unsigned int seed = rd();
    
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> lon_dist(lon_min, lon_max);
        std::uniform_real_distribution<double> lat_dist(lat_min, lat_max);

        coordinates.clear();
        Number_of_locations = 0;

        const int max_attempts = std::max(10000, count * 1000);
        int attempts = 0;

        while ((int)coordinates.size() < count && attempts < max_attempts) {
            ++attempts;
            double lon = lon_dist(rng);
            double lat = lat_dist(rng);

            if (!point_in_polygon(lon, lat, belgium_poly)) continue; // reject points outside polygon
            
            // Avoid near-duplicates using a small epsilon
            bool dup = false;
            for (const auto &p : coordinates) {
                if (std::abs(p.first - lon) < 1e-6 && std::abs(p.second - lat) < 1e-6) {
                    dup = true;
                    break;
                }
            }
            if (dup) continue;
            

            // Store as (longitude, latitude)
            coordinates.emplace_back(lon, lat);

        }

        if ((int)coordinates.size() < count) {
            std::cerr << "Warning: only sampled " << coordinates.size() << " points after " << attempts << " attempts. Try increasing max_attempts or use replacement." << std::endl;
        }

        Number_of_locations = static_cast<int>(coordinates.size());
        std::cout << "Sampled " << Number_of_locations << " coordinates inside Belgium." << std::endl;

        sampledCoordinates = true;
    }

        // Save current coordinates to a whitespace-separated text file. Each line: <longitude> <latitude>
        inline bool save_coordinates_to_file(const std::string &filename) {
            try {
                std::filesystem::path p(filename);
                auto dir = p.parent_path();
                if (!dir.empty() && !std::filesystem::exists(dir)) {
                    std::filesystem::create_directories(dir);
                }
            }
            catch (const std::exception &e) {
                std::cerr << "Failed to create parent directory for: " << filename << " -> " << e.what() << std::endl;
                return false;
            }

            std::ofstream out(filename);
            if (!out.is_open()) {
                std::cerr << "Failed to open coordinates output file: " << filename << std::endl;
                return false;
            }

            out << std::setprecision(10);
            for (const auto &p : coordinates) {
                out << p.first << ' ' << p.second << '\n';
            }

            out.close();
            return true;
        }

    // start osrm engine
    void start_engine() {
        // Load coordinates from file
        if(!sampledCoordinates) load_coordinates_from_file(pathTO_coordinates);

        // If Number_of_locations wasn't set explicitly, use the number of loaded coordinates.
        if (Number_of_locations == 0) {
            Number_of_locations = static_cast<int>(coordinates.size());
        }

        if (Number_of_locations <= 0) {
            std::cerr << "No locations available to start engine. Ensure coordinates are loaded.\n";
            exit(EXIT_FAILURE);
        }

        TravelTimes = new int *[Number_of_locations];
        TravelDistances = new int *[Number_of_locations];
        for (int i = 0; i < Number_of_locations; i++) {
            TravelTimes[i] = new int[Number_of_locations];
            TravelDistances[i] = new int[Number_of_locations];
        }

        // Set the number of threads to the maximum available
        max_threads = static_cast<int>(std::thread::hardware_concurrency());

        // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore
        config.storage_config = {pathTo_OSM_data};
        config.use_shared_memory = false;

        // We support two routing speed up techniques:
        // - Contraction Hierarchies (CH): requires extract+contract pre-processing
        // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
        config.algorithm = osrm::EngineConfig::Algorithm::CH; // or MLD
        // config.algorithm = osrm::EngineConfig::Algorithm::MLD;

        engine = std::make_unique<osrm::OSRM>(config);
    }
};

#endif
