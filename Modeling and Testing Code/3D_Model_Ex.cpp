#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>

#include "AB_Filter_Main.h"
#include "AB_Struct_Storage.h"
#include "AB_Deployment.h"

// --- EXTERNS FROM FILTER MAIN ---
extern AB_Snsrs Sensors;
extern AB_Attitude_State AttState;
extern AB_Vertical_State VertState;
extern AB_Horizontal_State HorizState;

// Import logic flags
extern bool apogee; 
extern bool burning;
extern bool burnout;

// --- CONFIGURATION ---
const float DEG_TO_RAD = 3.1415926535f / 180.0f;
const float GRAVITY_CONST = 9.802f;
const float SEA_LEVEL_PRESSURE_PA = 101325.0f; 

// --- ROD-ISOLATION CALIBRATION CONSTANTS (MATCHING FILTER MAIN) ---
extern Eigen::Matrix3f SOFT_IRON;
extern Eigen::Vector3f BIAS_ROCKET;
extern Eigen::Vector3f BIAS_ROD;

std::ofstream mag_diag_file;

// --- HELPER FUNCTIONS ---
float get_altitude_from_pressure(float pressure_pa) {
    if (pressure_pa < 0.1f) return 0.0f;
    return 44330.0f * (1.0f - std::pow(pressure_pa / SEA_LEVEL_PRESSURE_PA, 1.0f / 5.255f));
}

struct FlightLogPoint {
    float time_ms; float pressure_pa;
    float ax, ay, az; float gx, gy, gz;     
    float hgx, hgy, hgz; float mx, my, mz;     
};

std::vector<FlightLogPoint> load_csv(const std::string& filename) {
    std::vector<FlightLogPoint> data;
    std::ifstream file(filename);
    if (!file.is_open()) return data;
    std::string line;
    std::getline(file, line); 
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val_str;
        std::vector<float> row_vals;
        while (std::getline(ss, val_str, ',')) {
            try { row_vals.push_back(std::stof(val_str)); } catch (...) { row_vals.push_back(0.0f); }
        }
        if (row_vals.size() < 15) continue;
        FlightLogPoint p;
        p.time_ms = row_vals[0];
        p.pressure_pa = row_vals[1] * 100.0f;
        p.ax = row_vals[3] * GRAVITY_CONST;
        p.ay = row_vals[4] * GRAVITY_CONST;
        p.az = row_vals[5] * GRAVITY_CONST;
        p.gx = row_vals[6] * DEG_TO_RAD;
        p.gy = row_vals[7] * DEG_TO_RAD;
        p.gz = row_vals[8] * DEG_TO_RAD;
        p.hgx = row_vals[9] * GRAVITY_CONST;
        p.hgy = row_vals[10] * GRAVITY_CONST;
        p.hgz = row_vals[11] * GRAVITY_CONST;
        p.mx = row_vals[12];
        p.my = row_vals[13];
        p.mz = row_vals[14];
        data.push_back(p);
    }
    return data;
}

int main() {
    // Load the raw CSV data
    std::vector<FlightLogPoint> flight_log = load_csv("brute_test_flight_2025-11-23_SRAD_flight-only.csv"); 
    if (flight_log.size() < 20) return -1;

    // --- INITIALIZATION ---
    // Use the first data point to zero altitude
    float base_altitude = get_altitude_from_pressure(flight_log[0].pressure_pa);
    
    // Initialize filter states and set mag calibration matrix/vectors
    AB_Filter_Initialize(); 
    
    // --- SETUP OUTPUT FILE ---
    std::ofstream out_file("flight_3d_state.csv");
    out_file << "Time,PE,PN,PU,VE,VN,VU,Qw,Qx,Qy,Qz\n";

    std::cout << "--- PROCESSING 3D EXPORT (Rod-Isolation Mode) ---" << std::endl;

    // --- Magnetometer Diagnostic File ---
    mag_diag_file.open("mag_diagnostics.csv");
    mag_diag_file << "Time,Raw_Norm,Cal_Norm,Is_Burning,Is_Burnout\n";

    std::cout << "--- PROCESSING WITH MAG DIAGNOSTICS ---" << std::endl;

    for (size_t i = 1; i < flight_log.size(); i++) {
        FlightLogPoint& curr = flight_log[i];
        float dt = (curr.time_ms - flight_log[i-1].time_ms) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f; 
        Sensors.dt = dt;
        
        // 1. Load Sensors
        Sensors.Accelerometer << curr.ax, curr.ay, curr.az;
        Sensors.AccelerometerHG << -curr.hgx, -curr.hgy, curr.hgz; // Axis Flip
        Sensors.Gyroscope << curr.gx, curr.gy, curr.gz;
        Sensors.Magnetometer << curr.mx, curr.my, curr.mz; // Pass RAW to filter
        
        // 2. Barometer
        Sensors.Barometer = get_altitude_from_pressure(curr.pressure_pa) - base_altitude; 
        Sensors.GPS.setZero(); 

        // 3. Run Filter
        // The calibration logic (cal_mag = SOFT_IRON * (raw_mag - BIAS_ROCKET - BIAS_ROD)) 
        // is now handled INSIDE AB_loop() based on the burning/burnout flags.
        AB_loop();

        // 4. EXPORT STATE
        float time_sec = curr.time_ms / 1000.0f;
        
        // Only log active flight portion (Post-Initialization)
        if (time_sec > 14780.0f) {
            out_file << std::fixed << std::setprecision(4)
                     << time_sec << ","
                     << HorizState.Position_East << ","
                     << HorizState.Position_North << ","
                     << VertState.Altitude << ","
                     << HorizState.Velocity_East << ","
                     << HorizState.Velocity_North << ","
                     << VertState.Velocity_Up << ","
                     << AttState.Quaternion_Body_To_ENU.w() << ","
                     << AttState.Quaternion_Body_To_ENU.x() << ","
                     << AttState.Quaternion_Body_To_ENU.y() << ","
                     << AttState.Quaternion_Body_To_ENU.z() << "\n";
            
            // Console Feedback
            if (i % 500 == 0) {
                std::cout << "Processed T: " << time_sec << "s | Alt: " << VertState.Altitude << "m" << std::endl;
            }
        }
    }
    
    out_file.close();
    mag_diag_file.close();
    std::cout << "Done! 3D State saved to 'flight_3d_state.csv'" << std::endl;
    return 0;
}