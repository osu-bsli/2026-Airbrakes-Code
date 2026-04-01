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
extern float apogee; 
extern bool burning;
extern bool burnout;

// --- CONFIGURATION ---
const float DEG_TO_RAD = 3.1415926535f / 180.0f;
const float GRAVITY_CONST = 9.802f;
const float SEA_LEVEL_PRESSURE_PA = 101325.0f; 

// --- DUAL-PHASE MAG CALIBRATION CONSTANTS ---
const Eigen::Vector3f HARD_IRON_ASCENT(-20.7248f, -30.8167f, -38.5946f);
Eigen::Matrix3f SOFT_IRON_ASCENT;
const Eigen::Vector3f HARD_IRON_DESCENT(-20.6671f, -30.6290f, -38.1985f);
Eigen::Matrix3f SOFT_IRON_DESCENT;

// --- AERO CONFIGURATION ---
const float ROCKET_MASS_KG  = 20.0f;    
const float ROCKET_DIAM_M   = 0.1524f; // 6 inches
const float REF_AREA        = 3.14159265f * pow(ROCKET_DIAM_M / 2.0f, 2);

// Atmosphere Physics
const float GAS_CONSTANT_R  = 287.05f;
const float GAMMA           = 1.40f;    
const float TEMP_LAPSE_RATE = 0.0065f;  
const float TEMP_GROUND_K   = 273.15f + 9.0f; 

// --- HELPER FUNCTIONS ---
void SetupCalibration() {
    SOFT_IRON_ASCENT << 0.9091f, -0.0004f, 0.0032f, -0.0058f, 0.9412f, 0.0145f, 0.0022f, -0.0077f, 0.8800f;
    SOFT_IRON_DESCENT << 0.9108f, 0.0016f, 0.0018f, -0.0047f, 0.9402f, 0.0033f, -0.0000f, 0.0045f, 0.8923f;
}

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
    SetupCalibration(); 

    std::vector<FlightLogPoint> flight_log = load_csv("brute_test_flight_2025-11-23_SRAD_flight-only.csv"); 
    if (flight_log.size() < 20) return -1;

    // --- INITIALIZATION ---
    // NOTE: Pre-flight calibration loop REMOVED. 
    // We rely on AB_Filter_Main hardcoded biases or filter initialization.
    
    float base_altitude = get_altitude_from_pressure(flight_log[0].pressure_pa);
    Sensors.dt = 0.01f; 
    
    AB_Filter_Initialize(); 
    
    std::cout << "--- AERO ANALYSIS (3D Velocity) ---" << std::endl;
    std::cout << "Time  | Alt(m) | Vel(m/s) | Drag(N) | Cd    | Mach | Rho" << std::endl;

    // --- EXPORT SETUP ---
    std::ofstream out_file("flight_aero_data.csv");
    out_file << "Time,Alt,Vel,Drag,Cd,Mach,Rho\n"; 

    for (size_t i = 10; i < flight_log.size(); i++) {
        FlightLogPoint& curr = flight_log[i];
        float dt = (curr.time_ms - flight_log[i-1].time_ms) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f; 
        Sensors.dt = dt;
        
        // 1. Load IMU Sensors
        Sensors.Accelerometer << curr.ax, curr.ay, curr.az;
        Sensors.AccelerometerHG << -curr.hgx, -curr.hgy, curr.hgz; // Axis Flip
        // Removed: Sensors.AccelerometerHG += high_g_bias; (Handled by filter)
        Sensors.Gyroscope << curr.gx, curr.gy, curr.gz;
        
        // 2. Barometer
        Sensors.Barometer = get_altitude_from_pressure(curr.pressure_pa) - base_altitude; 
        Sensors.GPS.setZero(); 

        // 3. APPLY DUAL-PHASE MAG CALIBRATION
        Eigen::Vector3f raw_mag(curr.mx, curr.my, curr.mz);
        
        if (apogee == true) { 
            Sensors.Magnetometer = SOFT_IRON_DESCENT * (raw_mag - HARD_IRON_DESCENT);
        } else {
            Sensors.Magnetometer = SOFT_IRON_ASCENT * (raw_mag - HARD_IRON_ASCENT);
        }

        // 4. Run Filter
        AB_loop();

        // 5. Aero Calculation & Export
        float time_sec = curr.time_ms / 1000.0f;
        
        // LOGIC: We want the Coast Phase.
        // Coast = Burnout has happened AND We haven't reached apogee yet.
        if (burnout && !apogee) {
            
            // Atmosphere
            float T_curr = TEMP_GROUND_K - (TEMP_LAPSE_RATE * VertState.Altitude);
            float Rho = curr.pressure_pa / (GAS_CONSTANT_R * T_curr);
            float speed_of_sound = sqrt(GAMMA * GAS_CONSTANT_R * T_curr);

            // --- 3D VELOCITY CALCULATION ---
            float Vn = HorizState.Velocity_North;
            float Ve = HorizState.Velocity_East;
            float Vu = VertState.Velocity_Up;
            
            float V_sq = (Vn*Vn) + (Ve*Ve) + (Vu*Vu);
            float V_total = sqrt(V_sq);
            float Mach = V_total / speed_of_sound;

            // Drag Force (Use Low-G Accel Z magnitude for now, assuming axis alignment is handled by filter)
            // Note: Since filter cleans bias, Accel Z (body) is roughly Drag/Mass when thrust is 0.
            // Using Low-G sensor as it has better resolution during coast.
            float drag_accel = abs(Sensors.Accelerometer.z());
            float Drag_Force = ROCKET_MASS_KG * drag_accel;

            // Cd Calculation
            float Cd = 0.0f;
            // Avoid divide by zero and low velocity noise
            if (V_sq > 100.0f) { 
                Cd = (2.0f * Drag_Force) / (Rho * V_sq * REF_AREA);
            }

            // Console Output (Reduced Rate)
            if (i % 25 == 0) {
                std::cout << std::fixed << std::setprecision(4)
                          << time_sec << " | " 
                          << std::setw(6) << VertState.Altitude << " | "
                          << std::setw(8) << V_total << " | "
                          << std::setw(7) << Drag_Force << " | "
                          << std::setw(5) << Cd << " | "
                          << std::setw(4) << Mach << " | "
                          << std::setw(5) << Rho
                          << std::endl;
            }

            out_file << std::fixed << std::setprecision(4)
                    << time_sec << ","
                    << VertState.Altitude << ","
                    << V_total << ","
                    << Drag_Force << ","
                    << Cd << ","
                    << Mach << ","
                    << Rho << "\n";
        }
    }
    return 0;
}