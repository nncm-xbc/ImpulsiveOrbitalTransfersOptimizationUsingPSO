#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

// Structure to hold PSO optimization results for visualization
struct PSOOrbitTransferResult {
    // Initial orbit parameters
    double initial_radius;
    double initial_inclination;
    double initial_raan;
    double initial_eccentricity;
    double initial_arg_periapsis;
    
    // Target orbit parameters
    double target_radius;
    double target_inclination;
    double target_raan;
    double target_eccentricity;
    double target_arg_periapsis;
    
    // Optimal transfer parameters
    double initial_true_anomaly;    // Where first impulse occurs
    double final_true_anomaly;      // Where second impulse occurs
    double transfer_time;           // Time of flight for the transfer
    
    // For three-impulse transfers
    bool is_three_impulse;
    double intermediate_radius;     // For three-impulse transfers
    
    // Delta-V information
    std::vector<double> delta_v_magnitudes;
    std::vector<double> plane_change;
    
    // PSO convergence data
    std::vector<double> iteration_objectives;
    
    // Default constructor
    PSOOrbitTransferResult() : 
        initial_radius(0.0), initial_inclination(0.0), initial_raan(0.0), 
        initial_eccentricity(0.0), initial_arg_periapsis(0.0),
        target_radius(0.0), target_inclination(0.0), target_raan(0.0),
        target_eccentricity(0.0), target_arg_periapsis(0.0),
        initial_true_anomaly(0.0), final_true_anomaly(0.0), transfer_time(0.0),
        is_three_impulse(false), intermediate_radius(0.0) {}
};

// Function to load PSO results from a file
bool loadPSOResultsFromFile(const std::string& filename, PSOOrbitTransferResult& result) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open PSO results file: " << filename << std::endl;
        return false;
    }
    
    std::string line;
    std::string section;
    
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#')
            continue;
        
        // Check for section headers
        if (line[0] == '[' && line.back() == ']') {
            section = line.substr(1, line.size() - 2);
            continue;
        }
        
        // Parse key-value pairs
        std::istringstream iss(line);
        std::string key;
        std::string equals;
        std::string value;
        
        if (std::getline(iss, key, '=') && std::getline(iss, value)) {
            // Trim whitespace
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            // Process based on section
            if (section == "InitialOrbit") {
                if (key == "radius" || key == "semi_major_axis")
                    result.initial_radius = std::stod(value);
                else if (key == "inclination")
                    result.initial_inclination = std::stod(value);
                else if (key == "raan")
                    result.initial_raan = std::stod(value);
                else if (key == "eccentricity")
                    result.initial_eccentricity = std::stod(value);
                else if (key == "arg_periapsis")
                    result.initial_arg_periapsis = std::stod(value);
            }
            else if (section == "TargetOrbit") {
                if (key == "radius" || key == "semi_major_axis")
                    result.target_radius = std::stod(value);
                else if (key == "inclination")
                    result.target_inclination = std::stod(value);
                else if (key == "raan")
                    result.target_raan = std::stod(value);
                else if (key == "eccentricity")
                    result.target_eccentricity = std::stod(value);
                else if (key == "arg_periapsis")
                    result.target_arg_periapsis = std::stod(value);
            }
            else if (section == "OptimalTransfer") {
                if (key == "initial_true_anomaly")
                    result.initial_true_anomaly = std::stod(value);
                else if (key == "final_true_anomaly")
                    result.final_true_anomaly = std::stod(value);
                else if (key == "transfer_time")
                    result.transfer_time = std::stod(value);
                else if (key == "is_three_impulse")
                    result.is_three_impulse = (value == "true" || value == "1");
                else if (key == "intermediate_radius")
                    result.intermediate_radius = std::stod(value);
            }
            else if (section == "DeltaV") {
                if (key == "magnitude") {
                    std::istringstream vss(value);
                    std::string item;
                    result.delta_v_magnitudes.clear();
                    while (std::getline(vss, item, ',')) {
                        result.delta_v_magnitudes.push_back(std::stod(item));
                    }
                }
                if (key == "plane_change") {
                    std::istringstream vss(value);
                    std::string item;
                    result.plane_change.clear();
                    while (std::getline(vss, item, ',')) {
                        result.delta_v_magnitudes.push_back(std::stod(item));
                    }
                }
            }
        }
    }
    
    file.close();
    return true;
}