#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <windows.h>
#include <psapi.h>

#include "AB_Filter_Main.h"
#include "AB_Struct_Storage.h"
#include "AB_Deployment.h"

// --- CONFIGURATION ---
const float DEG_TO_RAD = 3.1415926535f / 180.0f;
const float RAD_TO_DEG = 180.0f / 3.1415926535f;
const float GRAVITY_CONST = 9.802f;
const float SEA_LEVEL_PRESSURE_PA = 101325.0f; 

// --- DUAL-PHASE MAG CALIBRATION CONSTANTS ---
const Eigen::Vector3f HARD_IRON_ASCENT(-20.7248f, -30.8167f, -38.5946f);
Eigen::Matrix3f SOFT_IRON_ASCENT;
const Eigen::Vector3f HARD_IRON_DESCENT(-20.6671f, -30.6290f, -38.1985f);
Eigen::Matrix3f SOFT_IRON_DESCENT;

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

int main() {
    SetupCalibration(); 

    // Instantiate Master Structs to pre-allocate all memory on the stack
    AB_Filter_Main_Variables M; 
    AB_Predict_Deployment_Angle_Variables DA_Vars;
    AB_Predict_Apogee_Variables AP_Vars;

    // Initialize internal filter states and workspace variables
    AB_Filter_Initialize(M); 
    PredictDeploymentAngleInitialize(DA_Vars);

    // Hardcoded data points
    std::vector<std::string> hardcoded_lines = {
        "14773088,982.210022,20.480000,-0.079853,-0.055311,1.015629,0.000000,0.000000,0.121951,0.000000,0.441000,2.058000,-43.931999,-31.542000,-92.147995,39.7285309,-83.7083282,320.000,44.000,7613,7",
        "14773096,982.210022,20.480000,-0.079243,-0.053236,1.015873,-0.060976,0.000000,0.060976,0.049000,0.490000,2.058000,-43.931999,-31.542000,-92.147995,39.7285309,-83.7083282,320.000,44.000,7613,7"
    };

    FlightLogPoint prev_point = {};
    int row_count = 0;
    float base_altitude = 0.0f;
    
    std::cout << "--- FLIGHT STATE ANALYSIS (HARDCODED 2-LINE MODE) ---" << std::endl;
    std::cout << "  Time   |   Alt    |  Pos N   |  Pos E   |   V_Up   |   V_N    |   V_E    | ZenithDeg|  Brake%  " << std::endl;
    std::cout << "-------------------------------------------------------------------------------------------------" << std::endl;

    for (const std::string& line : hardcoded_lines) {
        std::stringstream ss(line);
        std::string val_str;
        std::vector<float> row_vals;
        
        while (std::getline(ss, val_str, ',')) {
            try { row_vals.push_back(std::stof(val_str)); } catch (...) { row_vals.push_back(0.0f); }
        }
        if (row_vals.size() < 15) continue;

        FlightLogPoint curr;
        curr.time_ms = row_vals[0];
        curr.pressure_pa = row_vals[1] * 100.0f;
        curr.ax = row_vals[3] * GRAVITY_CONST;
        curr.ay = row_vals[4] * GRAVITY_CONST;
        curr.az = row_vals[5] * GRAVITY_CONST;
        curr.gx = row_vals[6] * DEG_TO_RAD;
        curr.gy = row_vals[7] * DEG_TO_RAD;
        curr.gz = row_vals[8] * DEG_TO_RAD;
        curr.hgx = row_vals[9] * GRAVITY_CONST;
        curr.hgy = row_vals[10] * GRAVITY_CONST;
        curr.hgz = row_vals[11] * GRAVITY_CONST;
        curr.mx = row_vals[12];
        curr.my = row_vals[13];
        curr.mz = row_vals[14];

        row_count++;

        if (row_count == 1) {
            base_altitude = get_altitude_from_pressure(curr.pressure_pa);
            prev_point = curr;
            continue; 
        }

        // Calculate dt using previous point
        float dt = (curr.time_ms - prev_point.time_ms) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f; 
        M.Sensors.dt = dt; //
        
        // Update sensor data in Master Struct
        M.Sensors.Accelerometer << curr.ax, curr.ay, curr.az;
        M.Sensors.AccelerometerHG << -curr.hgx, -curr.hgy, curr.hgz; 
        M.Sensors.Gyroscope << curr.gx, curr.gy, curr.gz;
        M.Sensors.Barometer = get_altitude_from_pressure(curr.pressure_pa) - base_altitude; 
        M.Sensors.GPS.setZero(); 

        Eigen::Vector3f raw_mag(curr.mx, curr.my, curr.mz);
        if (M.apogee == true) { 
            M.Sensors.Magnetometer = SOFT_IRON_DESCENT * (raw_mag - HARD_IRON_DESCENT);
        } else {
            M.Sensors.Magnetometer = SOFT_IRON_ASCENT * (raw_mag - HARD_IRON_ASCENT);
        }

        // Run primary filter loop
        AB_loop(M);

        float time_sec = curr.time_ms / 1000.0f;
        float v_horiz = sqrt(M.HorizState.Velocity_North * M.HorizState.Velocity_North + 
                             M.HorizState.Velocity_East * M.HorizState.Velocity_East);
        float zenith_deg = atan2(v_horiz, M.VertState.Velocity_Up) * RAD_TO_DEG;
        
        float airbrake_pct = 0.0f;
        if ((M.burning || M.burnout) && !M.apogee) {
            float apogeeIC[] = {M.VertState.Altitude, M.VertState.Velocity_Up, (float)(zenith_deg * DEG_TO_RAD), 0.0f};
            // Run prediction using workspace structures to avoid runtime allocations
            float predicted_val = (float)PredictDeploymentAngle(apogeeIC, DA_Vars, AP_Vars); 
            airbrake_pct = predicted_val * 100.0f;
            if (airbrake_pct < 0.0f) airbrake_pct = 0.0f;
            if (airbrake_pct > 100.0f) airbrake_pct = 100.0f;
        }

        // Log analysis results for the processed point
        std::cout << std::fixed << std::setprecision(2)
                  << std::setw(8) << time_sec << " | " 
                  << std::setw(8) << M.VertState.Altitude << " | "
                  << std::setw(8) << M.HorizState.Position_North << " | "
                  << std::setw(8) << M.HorizState.Position_East << " | "
                  << std::setw(8) << M.VertState.Velocity_Up << " | "
                  << std::setw(8) << M.HorizState.Velocity_North << " | "
                  << std::setw(8) << M.HorizState.Velocity_East << " | "
                  << std::setw(8) << zenith_deg << " | "
                  << std::setw(8) << airbrake_pct
                  << std::endl;

        prev_point = curr;
    }
    
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        std::cout << "\n=======================================\n";
        std::cout << "Windows OS Memory Report (HARDCODED):\n";
        std::cout << "Peak RAM Used: " << pmc.PeakWorkingSetSize << " bytes (" 
                  << pmc.PeakWorkingSetSize / 1024 << " KB)\n";
        std::cout << "=======================================\n";
    }
    return 0;
}