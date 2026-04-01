#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include "C:\Users\reill\Documents\BSLI\Eigen\eigen-5.0.0\Eigen\Dense"
using namespace Eigen;

// --- CONFIGURATION ---
const float DEG_TO_RAD = 3.1415926535f / 180.0f;
const float GRAVITY_CONST = 9.802f;

// --- MAG CALIBRATION ---
const Eigen::Vector3f HARD_IRON_ASCENT(-20.7248f, -30.8167f, -38.5946f);
Eigen::Matrix3f SOFT_IRON_ASCENT;
const Eigen::Vector3f HARD_IRON_DESCENT(-20.6671f, -30.6290f, -38.1985f);
Eigen::Matrix3f SOFT_IRON_DESCENT;

// --- REFERENCE VECTORS ---
Vector3f Mag_Reference; // Will be calculated on pad!
Vector3f Vertical_Up(0.0f, 0.0f, 1.0f); 

struct FlightLogPoint {
    float time_ms; float pressure_pa;
    float ax, ay, az; float gx, gy, gz;     
    float hgx, hgy, hgz; float mx, my, mz;     
};

// --- HELPER FUNCTIONS ---

void SetupCalibration() {
    SOFT_IRON_ASCENT << 0.9091f, -0.0004f, 0.0032f, -0.0058f, 0.9412f, 0.0145f, 0.0022f, -0.0077f, 0.8800f;
    SOFT_IRON_DESCENT << 0.9108f, 0.0016f, 0.0018f, -0.0047f, 0.9402f, 0.0033f, -0.0000f, 0.0045f, 0.8923f;
    
    // Default (will be overwritten by Calibrate_Reference_On_Pad)
    Mag_Reference << -2.3296f, 20.3722f, -47.1803f;
    Mag_Reference.normalize(); 
}

// --- NEW FUNCTION: Auto-Zero on Pad ---
void Calibrate_Reference_On_Pad(const std::vector<FlightLogPoint>& log) {
    Vector3f sum(0, 0, 0);
    int count = 0;

    // We scan the first few points to find stable pad data
    // Usually the first 1-2 seconds are fine.
    for (size_t i = 0; i < 50; i++) { 
        if (i >= log.size()) break;

        // SKIP invalid zeros
        if (log[i].mx == 0 && log[i].my == 0 && log[i].mz == 0) continue;

        // Apply ASCENT calibration (Hard/Soft Iron)
        Vector3f raw(log[i].mx, log[i].my, log[i].mz);
        Vector3f cal = SOFT_IRON_ASCENT * (raw - HARD_IRON_ASCENT);
        
        sum += cal;
        count++;
        
        // Stop after 10 good samples
        if (count >= 10) break;
    }

    if (count > 0) {
        Mag_Reference = sum / (float)count;
        Mag_Reference.normalize(); // Crucial!
        
        std::cout << "--- AUTO-CALIBRATION COMPLETE ---" << std::endl;
        std::cout << "Captured Pad Reference: [" 
                  << Mag_Reference.x() << ", " 
                  << Mag_Reference.y() << ", " 
                  << Mag_Reference.z() << "]" << std::endl;
    } else {
        std::cout << "ERROR: Could not capture mag reference! Using default." << std::endl;
    }
}

Quaternionf rotationFromVectors(Vector3f u, Vector3f v) {
    u.normalize();
    v.normalize();
    float dot = u.dot(v);
    if (dot > 0.99999f) return Quaternionf::Identity();
    if (dot < -0.99999f) return Quaternionf(0, 1, 0, 0);

    Vector3f cross = u.cross(v);
    Quaternionf q;
    q.w() = 1.0f + dot;
    q.vec() = cross;
    q.normalize();
    return q;
}

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

// --- MAIN ---
int main() {
    SetupCalibration();
    
    // 1. Load Data
    std::vector<FlightLogPoint> flight_log = load_csv("brute_test_flight_2025-11-23_SRAD_flight-only.csv");
    if (flight_log.size() < 10) return -1;

    // 2. AUTO-CALIBRATE REFERENCE (The Fix!)
    Calibrate_Reference_On_Pad(flight_log);

    // 3. Setup Export
    std::ofstream out_file("flight_att_compare.csv");
    out_file << "Time,G_w,G_x,G_y,G_z,M_w,M_x,M_y,M_z,Nx,Ny,Nz\n";

    Quaternionf q_gyro; q_gyro.setIdentity(); 
    Quaternionf q_mag;  q_mag.setIdentity();

    bool apogee_detected = false;
    float max_alt = -9999.0f;

    std::cout << "Processing " << flight_log.size() << " points..." << std::endl;

    for (size_t i = 1; i < flight_log.size(); i++) {
        FlightLogPoint& curr = flight_log[i];
        FlightLogPoint& prev = flight_log[i-1];
        
        float dt = (curr.time_ms - prev.time_ms) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f;

        // GYRO INTEGRATION
        Vector3f w(curr.gx, curr.gy, curr.gz); 
        Quaternionf q_delta;
        Vector3f half_theta = w * dt * 0.5f;
        q_delta.w() = 1.0f;
        q_delta.vec() = half_theta;
        q_delta.normalize();
        q_gyro = q_gyro * q_delta;
        q_gyro.normalize();

        // MAG VECTOR ROTATION
        float alt = 44330.0f * (1.0f - pow(curr.pressure_pa / 101325.0f, 0.1903f));
        if (alt > max_alt) max_alt = alt;
        if (max_alt > (alt + 20.0f)) apogee_detected = true;

        // Apply Calibration
        Vector3f raw_mag(curr.mx, curr.my, curr.mz);
        Vector3f cal_mag;
        if (apogee_detected) cal_mag = SOFT_IRON_DESCENT * (raw_mag - HARD_IRON_DESCENT);
        else                 cal_mag = SOFT_IRON_ASCENT  * (raw_mag - HARD_IRON_ASCENT);
        
        // Calculate Q that rotates Reference -> Measured
        // Since Reference is now == Start, this angle starts at 0!
        q_mag = rotationFromVectors(Mag_Reference, cal_mag);

        // Apply to Up Vector
        Vector3f Nose_Vector = q_mag * Vertical_Up;

        float time_sec = curr.time_ms / 1000.0f;
        out_file << std::fixed << std::setprecision(4) 
                 << time_sec << ","
                 << q_gyro.w() << "," << q_gyro.x() << "," << q_gyro.y() << "," << q_gyro.z() << ","
                 << q_mag.w()  << "," << q_mag.x()  << "," << q_mag.y()  << "," << q_mag.z()  << ","
                 << Nose_Vector.x() << "," << Nose_Vector.y() << "," << Nose_Vector.z() << "\n";
    }

    out_file.close();
    std::cout << "Done." << std::endl;
    return 0;
}