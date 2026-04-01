//Script is AI generated, will test on drone to ensure it works as intended, will need to fuse with data stack for rocket.

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include "AB_Struct_Storage.h"

// TODO: Include your sensor header here, e.g., #include "AB_Struct_Storage.h"

// Placeholder function: Replace this with your actual hardware reading logic
// For example, reading from your AB_Snsrs struct's Magnetometer values
Eigen::Vector3f readMagnetometer() {
    // Return the raw X, Y, Z magnetometer readings
    return Eigen::Vector3f(0.0f, 0.0f, 0.0f); 
}

int main() {
    // --- Configuration (Adjustable) ---
    const int START_DELAY_SECONDS = 10;
    const int CALIBRATION_DURATION_SECONDS = 60; 
    const int SENSOR_POLL_RATE_MS = 20; // 50 Hz polling rate
    const std::string OUTPUT_FILE = "mag_calibration_output.txt";

    // Wait 10 seconds before starting
    std::cout << "Starting calibration in " << START_DELAY_SECONDS << " seconds. Prepare to spin..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(START_DELAY_SECONDS));

    std::cout << "Calibration started! Spin the object around all 3 axes..." << std::endl;
    
    std::vector<Eigen::Vector3f> mag_data;
    
    // Set time limits
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds(CALIBRATION_DURATION_SECONDS);

    // Collect data for the configured duration (e.g., 60 seconds)
    while (std::chrono::steady_clock::now() < end_time) {
        mag_data.push_back(readMagnetometer());
        std::this_thread::sleep_for(std::chrono::milliseconds(SENSOR_POLL_RATE_MS)); 
    }

    std::cout << "Data collection complete. Calculating biases..." << std::endl;

    if (mag_data.empty()) {
        std::cerr << "Error: No data collected." << std::endl;
        return 1;
    }

    // --- Calculate Hard and Soft Iron Biases ---
    // Using the Min/Max bounds approach to estimate the ellipsoid

    Eigen::Vector3f min_val = mag_data[0];
    Eigen::Vector3f max_val = mag_data[0];

    for (const auto& reading : mag_data) {
        min_val = min_val.cwiseMin(reading);
        max_val = max_val.cwiseMax(reading);
    }

    // 1. Hard Iron Bias (The center offset of the bounding box)
    Eigen::Vector3f hard_iron_bias = (max_val + min_val) / 2.0f;

    // 2. Soft Iron Matrix (The scaling required to make the ellipsoid a sphere)
    Eigen::Vector3f chord = (max_val - min_val) / 2.0f;
    float avg_chord = (chord.x() + chord.y() + chord.z()) / 3.0f;

    Eigen::Matrix3f soft_iron_matrix = Eigen::Matrix3f::Identity();
    
    // Protect against division by zero in case of identical dummy data
    if (chord.x() != 0 && chord.y() != 0 && chord.z() != 0) {
        soft_iron_matrix(0, 0) = avg_chord / chord.x();
        soft_iron_matrix(1, 1) = avg_chord / chord.y();
        soft_iron_matrix(2, 2) = avg_chord / chord.z();
    }

    // --- Output to Text File ---
    std::ofstream outfile(OUTPUT_FILE);
    if (outfile.is_open()) {
        outfile << "--- Magnetometer Calibration Results ---\n\n";
        
        outfile << "Hard Iron Bias Vector (Offset):\n";
        outfile << hard_iron_bias.x() << " " << hard_iron_bias.y() << " " << hard_iron_bias.z() << "\n\n";
        
        outfile << "Soft Iron Bias Matrix (Scale):\n";
        outfile << soft_iron_matrix << "\n\n";
        
        outfile << "Usage: Corrected_Mag = Soft_Iron_Matrix * (Raw_Mag - Hard_Iron_Bias)\n";
        
        outfile.close();
        std::cout << "Calibration successfully saved to " << OUTPUT_FILE << std::endl;
    } else {
        std::cerr << "Failed to open output file." << std::endl;
    }

    return 0;
}