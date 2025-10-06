// std libs
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <variant>
#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <filesystem>

// OSRM core headers used by this file
#include "osrm/trip_parameters.hpp"

// project OSRM parameter struct and helpers
#include "OSRMParameters.h"

// ********************************* LOCAL PARAMETERS ************************************
// Define earth's radius for haversine distance
#define EARTH_RADIUS 6371.0

// ++++++++++++++++++++++++++++++++++++++ FUNCTIONS ++++++++++++++++++++++++++++++++++++++

// Degrees transpormed into radians
inline double degreesToRadians(const double &degrees) {
    double res = degrees * M_PI / 180.0;
    return res;
}

// Haversie travel calculation (vogelvlucht)
inline double haversine(const double &lat1, const double &lon1, const double &lat2, const double &lon2) {
    double Lat1 = degreesToRadians(lat1);
    double Lon1 = degreesToRadians(lon1);
    double Lat2 = degreesToRadians(lat2);
    double Lon2 = degreesToRadians(lon2);

    double dlon = Lon2 - Lon1;
    double dlat = Lat2 - Lat1;

    double a = pow(sin(dlat / 2), 2) + cos(Lat1) * cos(Lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return EARTH_RADIUS * c * 1000; // Multiply by 1000 to get the result in meters
}

// Fill in with haversine
inline void haversineEngineParallel(int **&depotTravelDistancesHaversine, const int &coordinates1Size, const int &coordinates2Size,
                                    double **&coordinates1, double **&coordinates2, osrm_params& OSRM) {
    int num_threads = OSRM.max_threads;
    auto run_parallel = [coordinates1Size, coordinates2Size, num_threads](auto proc) {
        int payload_size = (coordinates1Size * coordinates2Size + num_threads - 1) / num_threads;
        std::vector<std::thread> threads;
        for (int i = 0; i < num_threads; ++i) {
            int start_i = i * payload_size;
            int end_i = std::min(coordinates1Size * coordinates2Size, (i + 1) * payload_size);
            threads.emplace_back([start_i, end_i, &proc]() { proc(start_i, end_i); });
        }
        for (auto &t : threads) {
            t.join();
        }
        threads.clear();
    };

    auto haversine_proc = [&](int start_i, int end_i) {
        for (int i = start_i; i < end_i; ++i) {
            int i1 = i / coordinates2Size;
            int i2 = i % coordinates2Size;
            double lat1 = coordinates1[i1][1];
            double lon1 = coordinates1[i1][0];
            double lat2 = coordinates2[i2][1];
            double lon2 = coordinates2[i2][0];
            depotTravelDistancesHaversine[i1][i2] = static_cast<int>(haversine(lat1, lon1, lat2, lon2));
        }
    };

    run_parallel(haversine_proc);
}

// Osrm engine to calculate the routing data
inline void osrmEngine(int **&travel_distances, int **&travel_times, const int &coordinates1Size, const int &coordinates2Size,
                       double **&coordinates1, double **&coordinates2, osrm_params& OSRM) {
    // map loactions (index of travel matrices) to transport numbers
    std::unique_ptr<int[]> haversineDistances = std::make_unique<int[]>(coordinates1Size * coordinates2Size);
    
    int num_threads = OSRM.max_threads;
    // lambda function for running parallel
    auto run_parallel = [coordinates1Size, coordinates2Size, num_threads](auto proc) {
        int payload_size = (coordinates1Size * coordinates2Size + num_threads - 1) / num_threads;
        std::vector<std::thread> threads;
        for (int i = 0; i < num_threads; ++i) {
            int start_i = i * payload_size;
            int end_i = std::min(coordinates1Size * coordinates2Size, (i + 1) * payload_size);
            threads.emplace_back([start_i, end_i, &proc]() { proc(start_i, end_i); });
        }

        for (auto &t : threads) {
            t.join();
        }
        threads.clear();
    };

    // Haversine calculation
    auto harvestine_proc = [&](int start_i, int end_i) {
        for (int i = start_i; i < end_i; ++i) {
            int i1 = i / coordinates2Size;
            int i2 = i % coordinates2Size;
            auto haversineDistance = haversine(coordinates1[i1][1], coordinates1[i1][0], coordinates2[i2][1], coordinates2[i2][0]);
            haversineDistances[i] = haversineDistance;
        }
    };

    // Execute harvestine in parallel
    run_parallel(harvestine_proc);

    // OSRM calculation
    auto osrm_proc = [&](int start_i, int end_i) {
        osrm::RouteParameters params;
        params.overview = osrm::RouteParameters::OverviewType::False;
        // params.generate_hints = false;

        for (int i = start_i; i < end_i; ++i) {
            int i1 = i / coordinates2Size;
            int i2 = i % coordinates2Size;

            auto &result_distance = travel_distances[i1][i2];
            auto &result_time = travel_times[i1][i2];
            const auto &haversineDistance = haversineDistances[i];

            // if(i1 == 73 && i2 == 102) std::cout << haversineDistance / 1000.0 << std::endl;

            if (result_time == INT32_MAX) {
                // Route
                params.coordinates.clear();
                params.coordinates.push_back({osrm::util::FloatLongitude{coordinates1[i1][0]}, osrm::util::FloatLatitude{coordinates1[i1][1]}});
                params.coordinates.push_back({osrm::util::FloatLongitude{coordinates2[i2][0]}, osrm::util::FloatLatitude{coordinates2[i2][1]}});

                // Response is in JSON format
                osrm::engine::api::ResultT result = osrm::json::Object();

                // Execute routing request, this does the heavy lifting
                const auto status = OSRM.engine->Route(params, result);

                auto &json_result = std::get<osrm::json::Object>(result);
                if (status == osrm::Status::Ok) {
                    auto &routes = std::get<osrm::json::Array>(json_result.values["routes"]);

                    // Let's just use the first route
                    auto &route = std::get<osrm::json::Object>(routes.values.at(0));
                    auto route_distance = std::get<osrm::json::Number>(route.values["distance"]).value;
                    auto route_time = std::get<osrm::json::Number>(route.values["duration"]).value;
                    // if(i1 == 73 && i2 == 102) std::cout << route_time/60.0 << std::endl;

                    // Warn users if extract does not contain the default coordinates from above
                    //*
                    if (route_distance == 0 || route_time == 0) {
                        if (static_cast<int>(coordinates1[i1][0] * 100) == static_cast<int>(coordinates2[i2][0] * 100) && static_cast<int>(coordinates1[i1][1] * 100) == static_cast<int>(coordinates2[i2][1] * 100)) {
                            result_distance = haversineDistance * 1.5;
                            result_time = result_distance / 14.0;
                            // if(i1 == 73 && i2 == 102) std::cout << result_time / 60.0 << std::endl;
                        }
                        else {
                            std::cout << "Note: distance or duration is zero. " << std::flush;
                            std::cout << "You are probably doing a query outside of the OSM extract.\n"
                                      << std::endl;
                            std::cout << "Coord. 1: " << coordinates1[i1][1] << ", " << coordinates1[i1][0] << std::endl;
                            std::cout << "Coord. 2: " << coordinates2[i2][1] << ", " << coordinates2[i2][0] << std::endl;

                            result_distance = haversineDistance * 1.5;
                            result_time = result_distance / 14.0;
                            std::cout << " Havcersine time: " << result_time << " s " << std::endl;

                            // if(i1 == 73 && i2 == 102) std::cout << result_time / 60.0 << std::endl;
                        }
                    }
                    else {
                        result_distance = route_distance;
                        result_time = route_time;
                        // cout << result_time / 60.0 << endl;
                        // if(i1 == 73 && i2 == 102) std::cout << result_time / 60.0 << std::endl;
                    }
                    //*/

                    // cout << "Distance: " << result_distance << " meter\n";
                    // cout << "Duration: " << result_time << " seconds\n";
                }
                else if (status == osrm::Status::Error) {
                    const auto &code = std::get<osrm::json::String>(json_result.values.at("code")).value;
                    const auto &message = std::get<osrm::json::String>(json_result.values.at("message")).value;

                    std::cout << "Code: " << code << std::endl;
                    std::cout << "Message: " << message << std::endl;
                    result_distance = haversineDistance * 2;
                    result_time = result_distance / 12.0;
                }
                // else if(i1 == 73 && i2 == 102) std::cout << " What ?" << std::endl;
            }
        }
    };

    // Execute OSRM calculations in parallel
    run_parallel(osrm_proc);

    // cout << "Number of calls " << c_times << endl;
}

// Write matrices to CSV files
inline void write_matrix_csv(osrm_params& OSRM) {
    auto matrix_csv = [](const std::string &filename, int **matrix, int n) -> bool {
        try {
            std::filesystem::path p(filename);
            auto dir = p.parent_path();
            if (!dir.empty() && !std::filesystem::exists(dir)) {
                std::filesystem::create_directories(dir);
            }
        }
        catch (const std::exception &e) {
            std::cerr << "Failed to create output directory for: " << filename << " -> " << e.what() << std::endl;
            return false;
        }

        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << filename << std::endl;
            return false;
        }

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                out << matrix[i][j];
                if (j + 1 < n) out << ',';
            }
            out << '\n';
        }
        out.close();
        return true;
    };

    const std::string dist_file = "/app/results/travel_distances.csv";
    const std::string time_file = "/app/results/travel_times.csv";

    if (matrix_csv(dist_file, OSRM.TravelDistances, OSRM.Number_of_locations)) {
        std::cout << " - Travel distances written to: " << dist_file << std::endl;
    }
    else {
        std::cerr << " - Failed to write travel distances to CSV." << std::endl;
    }

    if (matrix_csv(time_file, OSRM.TravelTimes, OSRM.Number_of_locations)) {
        std::cout << " - Travel times written to: " << time_file << std::endl;
    }
    else {
        std::cerr << " - Failed to write travel times to CSV." << std::endl;
    }
}

// Calculate travel times and distances
void calculate_osrm_metrics(osrm_params& OSRM) {
    
    // Start the engine once
    OSRM.start_engine();

    std::cout << "OSRM calculations started ...\n - Number of threads being used: " << OSRM.max_threads << std::endl;

    // ++++++++++++++++++++ Client locations ++++++++++++++++++++
    
    double** coordinates = new double *[OSRM.Number_of_locations];
    // store coordinates in raw pointers for better performance
    for (int i = 0; i < OSRM.Number_of_locations; i++) {
        coordinates[i] = new double[2];
        coordinates[i][0] = OSRM.coordinates[i].first;   // longitude
        coordinates[i][1] = OSRM.coordinates[i].second; // latitude
        for (int j = 0; j < OSRM.Number_of_locations; j++) {
            if (i == j) {
                OSRM.TravelTimes[i][j] = 0;     // going to the same place gives zero
                OSRM.TravelDistances[i][j] = 0; // going to the same place gives zero
            }
            else {
                OSRM.TravelTimes[i][j] = INT32_MAX;
                OSRM.TravelDistances[i][j] = INT32_MAX;
            }
        }
    }

    osrmEngine(OSRM.TravelDistances, OSRM.TravelTimes, OSRM.Number_of_locations, OSRM.Number_of_locations, coordinates, coordinates, OSRM);
    std::cout << " - Osrm calculations done." << std::endl;

    // Write matrices to CSV files
    write_matrix_csv(OSRM);

    // delete raw pointers
    for (int i = 0; i < OSRM.Number_of_locations; i++) {
        delete[] coordinates[i];
    }
    delete[] coordinates;
}