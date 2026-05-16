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
    SOFT_IRON_ASCENT << 0.9091f, -0.0004f, 0.0032f,
                         -0.0058f, 0.9412f, 0.0145f,
                          0.0022f, -0.0077f, 0.8800f;

    SOFT_IRON_DESCENT << 0.9108f, 0.0016f, 0.0018f,
                         -0.0047f, 0.9402f, 0.0033f,
                         -0.0000f, 0.0045f, 0.8923f;
}

float get_altitude_from_pressure(float pressure_pa) {
    if (pressure_pa < 0.1f) return 0.0f;

    return 44330.0f *
           (1.0f - std::pow(pressure_pa / SEA_LEVEL_PRESSURE_PA,
                            1.0f / 5.255f));
}

struct FlightLogPoint {
    float time_ms;
    float pressure_pa;

    float ax, ay, az;
    float gx, gy, gz;

    float hgx, hgy, hgz;

    float mx, my, mz;
};

int main() {
    SetupCalibration();

    AB_Filter_Main_Variables M;
    AB_Predict_Deployment_Angle_Variables DA_Vars;
    AB_Predict_Apogee_Variables AP_Vars;

    AB_Filter_Initialize(M);
    PredictDeploymentAngleInitialize(DA_Vars);

    // Input CSV
    std::ifstream file("C:\\Users\\reill\\Documents\\BSLI\\fial\\2026-Airbrakes-Code\\Data\\novflight.csv");

    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open novflight.csv" << std::endl;
        return 1;
    }

    // Output CSV
    std::ofstream outFile("C:\\Users\\reill\\Documents\\BSLI\\fial\\output_results.csv");

    if (!outFile.is_open()) {
        std::cerr << "ERROR: Could not open output CSV file" << std::endl;
        return 1;
    }

    // CSV header
    outFile << "time_sec,altitude,pos_north,pos_east,vel_up,vel_north,vel_east,zenith_deg,airbrake_pct\n";

    FlightLogPoint prev_point = {};
    int row_count = 0;
    float base_altitude = 0.0f;

    std::string line;

    std::cout << "--- FLIGHT STATE ANALYSIS (LIMITED TO 20000 ROWS) ---\n";

    while (std::getline(file, line) && row_count < 20000) {

        if (line.empty())
            continue;

        std::stringstream ss(line);
        std::string val_str;
        std::vector<float> row_vals;

        while (std::getline(ss, val_str, ',')) {
            try {
                row_vals.push_back(std::stof(val_str));
            }
            catch (...) {
                row_vals.push_back(0.0f);
            }
        }

        if (row_vals.size() < 15)
            continue;

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

        float dt = (curr.time_ms - prev_point.time_ms) / 1000.0f;
        
        // --- FIX: dt CLAMP ---
        // Prevents the massive integration jump on the very first cycle
        if (dt <= 0.0f || dt > 1.0f) {
            dt = 0.01f; 
        }

        M.Sensors.dt = dt;

        M.Sensors.Accelerometer << curr.ax, curr.ay, curr.az;
        M.Sensors.AccelerometerHG << -curr.hgx, -curr.hgy, curr.hgz;
        M.Sensors.Gyroscope << curr.gx, curr.gy, curr.gz;

        M.Sensors.Barometer =
            get_altitude_from_pressure(curr.pressure_pa) - base_altitude;

        M.Sensors.GPS.setZero();

        Eigen::Vector3f raw_mag(curr.mx, curr.my, curr.mz);

        if (M.apogee == true) {
            M.Sensors.Magnetometer =
                SOFT_IRON_DESCENT * (raw_mag - HARD_IRON_DESCENT);
        } else {
            M.Sensors.Magnetometer =
                SOFT_IRON_ASCENT * (raw_mag - HARD_IRON_ASCENT);
        }

        // Run the filter loop
        AB_loop(M);

        float time_sec = curr.time_ms / 1000.0f;

        float v_horiz =
            sqrt(M.HorizState.Velocity_North * M.HorizState.Velocity_North +
                 M.HorizState.Velocity_East * M.HorizState.Velocity_East);

        float zenith_deg =
            atan2(v_horiz, M.VertState.Velocity_Up) * RAD_TO_DEG;

        float airbrake_pct = 0.0f;

        if ((M.burning || M.burnout) && !M.apogee) {

            float apogeeIC[] = {
                M.VertState.Altitude,
                M.VertState.Velocity_Up,
                zenith_deg * DEG_TO_RAD,
                0.0f
            };

            airbrake_pct = (float)PredictDeploymentAngle(apogeeIC, DA_Vars, AP_Vars);

            // Bound the output to 0-100% instead of artificially clamping at 1.0
            if (airbrake_pct < 0.0f) airbrake_pct = 0.0f;
            if (airbrake_pct > 100.0f) airbrake_pct = 100.0f;
        }

        // Write to CSV
        outFile << std::fixed << std::setprecision(6)
                << time_sec << ","
                << M.VertState.Altitude << ","
                << M.HorizState.Position_North << ","
                << M.HorizState.Position_East << ","
                << M.VertState.Velocity_Up << ","
                << M.HorizState.Velocity_North << ","
                << M.HorizState.Velocity_East << ","
                << zenith_deg << ","
                << airbrake_pct
                << "\n";

        prev_point = curr;
    }

    file.close();
    outFile.close();

    std::cout << "Done. Output written to CSV.\n";

    return 0;
}
