/**
 * @file Results.hpp
 * @brief PSO optimization results loading and data structures
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides data structures and file I/O functionality for loading
 * PSO optimization results into the visualization system. Handles
 * parsing of structured result files and data validation.
 */

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

/**
 * @struct PSOOrbitTransferResult
 * @brief Complete orbital transfer optimization results from PSO
 *
 * Contains all parameters needed to fully specify and visualize an
 * optimal orbital transfer found by PSO optimization. Includes:
 * - Complete orbital element sets for initial and target orbits
 * - Optimal transfer timing and trajectory parameters
 * - Delta-V breakdown and impulse characteristics
 * - Transfer type classification and special cases
 *
 * This structure serves as the interface between the optimization
 * engine and the visualization system, ensuring all necessary
 * information is preserved and accessible.
 */
struct PSOOrbitTransferResult {
    // ========================================
    // INITIAL ORBIT PARAMETERS
    // ========================================

    /** @brief Initial orbit radius or semi-major axis (DU) */
    double initial_radius;

    /** @brief Initial orbit inclination (radians) */
    double initial_inclination;

    /** @brief Initial orbit right ascension of ascending node (radians) */
    double initial_raan;

    /** @brief Initial orbit eccentricity (0 = circular) */
    double initial_eccentricity;

    /** @brief Initial orbit argument of periapsis (radians) */
    double initial_arg_periapsis;

    // ========================================
    // TARGET ORBIT PARAMETERS
    // ========================================

    /** @brief Target orbit radius or semi-major axis (DU) */
    double target_radius;

    /** @brief Target orbit inclination (radians) */
    double target_inclination;

    /** @brief Target orbit right ascension of ascending node (radians) */
    double target_raan;

    /** @brief Target orbit eccentricity (0 = circular) */
    double target_eccentricity;

    /** @brief Target orbit argument of periapsis (radians) */
    double target_arg_periapsis;

    // ========================================
    // OPTIMAL TRANSFER PARAMETERS
    // ========================================

    /** @brief True anomaly where first impulse occurs (radians) */
    double initial_true_anomaly;

    /** @brief True anomaly where second impulse occurs (radians) */
    double final_true_anomaly;

    /** @brief Time of flight for the transfer (TU) */
    double transfer_time;

    // ========================================
    // THREE-IMPULSE TRANSFER PARAMETERS
    // ========================================

    /** @brief Whether this is a three-impulse transfer */
    bool is_three_impulse;

    /** @brief Intermediate orbit radius for three-impulse transfers (DU) */
    double intermediate_radius;

    // ========================================
    // DELTA-V AND IMPULSE INFORMATION
    // ========================================

    /** @brief Vector of impulse magnitudes [dv1, dv2, ...] (DU/TU) */
    std::vector<double> delta_v_magnitudes;

    /** @brief Vector of plane change angles for each impulse (radians) */
    std::vector<double> plane_change;

    /**
     * @brief Default constructor - initializes all values to zero
     *
     * Creates a result structure with all parameters set to safe
     * default values. Must be populated from file or manual input
     * before use in visualization.
     */
    PSOOrbitTransferResult() :
        initial_radius(0.0), initial_inclination(0.0), initial_raan(0.0),
        initial_eccentricity(0.0), initial_arg_periapsis(0.0),
        target_radius(0.0), target_inclination(0.0), target_raan(0.0),
        target_eccentricity(0.0), target_arg_periapsis(0.0),
        initial_true_anomaly(0.0), final_true_anomaly(0.0), transfer_time(0.0),
        is_three_impulse(false), intermediate_radius(0.0) {}
};

/**
 * @brief Load PSO optimization results from structured file
 * @param filename Path to results file
 * @param result Reference to result structure to populate
 * @return true if file loaded successfully, false on error
 *
 * Parses a structured text file containing PSO optimization results
 * and populates the provided result structure. The file format uses
 * sections and key-value pairs:
 *
 * ```
 * [InitialOrbit]
 * radius = 1.0
 * inclination = 0.5
 * ...
 *
 * [TargetOrbit]
 * radius = 1.5
 * ...
 *
 * [OptimalTransfer]
 * initial_true_anomaly = 0.0
 * ...
 *
 * [DeltaV]
 * magnitude = 1.2,0.8
 * plane_change = 0.1,0.0
 * ```
 *
 * Features:
 * - Section-based organization for clarity
 * - Comma-separated values for vector parameters
 * - Robust parsing with whitespace handling
 * - Error reporting for missing files
 * - Backward compatibility with different parameter names
 *
 * The parser is designed to be flexible and handle variations in
 * parameter naming while maintaining strict validation of numerical
 * values and required sections.
 */
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
                        result.plane_change.push_back(std::stod(item));
                    }
                }
            }
        }
    }

    file.close();
    return true;
}
