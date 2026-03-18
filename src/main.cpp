#include <Arduino.h>
#include <Eigen/Dense>
#include "LittleFS.h"

#undef B0
#undef B1
#undef B2
#undef B3

#include "AB_Filter_Main.h"
#include "AB_Struct_Storage.h"
#include "AB_Deployment.h"

// --- CONFIGURATION ---
const float GRAVITY_CONST = 9.802f;
const float SEA_LEVEL_PRESSURE_PA = 101325.0f;

// --- DUAL-PHASE MAG CALIBRATION ---
const Eigen::Vector3f HARD_IRON_ASCENT(-20.7248f, -30.8167f, -38.5946f);
Eigen::Matrix3f SOFT_IRON_ASCENT;
const Eigen::Vector3f HARD_IRON_DESCENT(-20.6671f, -30.6290f, -38.1985f);
Eigen::Matrix3f SOFT_IRON_DESCENT;

// --- STATIC FILTER & PREDICTOR STRUCTS ---
static AB_Filter_Main_Variables M; 
static AB_Predict_Deployment_Angle_Variables DA_Vars;
static AB_Predict_Apogee_Variables AP_Vars;

void SetupCalibration() {
    SOFT_IRON_ASCENT << 0.9091f, -0.0004f, 0.0032f, -0.0058f, 0.9412f, 0.0145f, 0.0022f, -0.0077f, 0.8800f;
    SOFT_IRON_DESCENT << 0.9108f, 0.0016f, 0.0018f, -0.0047f, 0.9402f, 0.0033f, -0.0000f, 0.0045f, 0.8923f;
}

float get_altitude(float pressure_pa) {
    if (pressure_pa < 0.1f) return 0.0f;
    return 44330.0f * (1.0f - std::pow(pressure_pa / SEA_LEVEL_PRESSURE_PA, 1.0f / 5.255f));
}

void log_memory() {
    Serial.println("\n=======================================");
    Serial.println("ESP32-S3 Memory Report:");
    Serial.printf("Internal Free Heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM Free Heap:    %u bytes\n", ESP.getFreePsram());
    Serial.printf("Peak Stack Used:    %u bytes\n", 32768 - uxTaskGetStackHighWaterMark(NULL));
    Serial.println("=======================================\n");
}

void setup() {
    Serial.begin(115200);
    while(!Serial);
    delay(2000);

    if(!LittleFS.begin(true)) {
        Serial.println("!!! LittleFS Mount Failed !!!");
        return;
    }

    SetupCalibration();
    M.Sensors.GPS.setZero(); 
    AB_Filter_Initialize(M);
    PredictDeploymentAngleInitialize(DA_Vars);
    
    M.Sensors.Mag_Reference << -2.32f, 20.37f, -47.18f;
    M.Sensors.Mag_Reference.normalize();

    File inputFile = LittleFS.open("/novflight.csv", "r");
    File outputFile = LittleFS.open("/flight_results.txt", "w");

    if (!inputFile || !outputFile) {
        Serial.println("!!! File Error !!!");
        return;
    }

    Serial.println("Starting Benchmarked Replay...");
    outputFile.println("Time,Alt,PosN,PosE,V_Up,V_N,V_E,Zenith,BrakePct");

    float last_time_ms = 0;
    float base_alt = 0;
    int data_row_count = 0;
    bool header_skipped = false;
    
    unsigned long startTime = millis();
    unsigned long total_math_time_us = 0; // Accumulator for just the math

    while (inputFile.available()) {
        String line = inputFile.readStringUntil('\n');
        line.trim();
        if (line.length() < 10) continue;

        if (!header_skipped) {
            if (!isdigit(line[0]) && line[0] != '-') {
                header_skipped = true;
                continue; 
            }
            header_skipped = true; 
        }

        // --- I/O: PARSE CSV (NOT TIMED) ---
        float vals[21] = {0};
        int valIdx = 0, startPos = 0;
        for (int i = 0; i < (int)line.length() && valIdx < 21; i++) {
            if (line[i] == ',' || i == (int)line.length() - 1) {
                vals[valIdx++] = line.substring(startPos, i + (i == (int)line.length() - 1 ? 1 : 0)).toFloat();
                startPos = i + 1;
            }
        }
        
        if (valIdx < 15) continue; 
        float curr_time_ms = vals[0];
        
        if (data_row_count == 0) {
            base_alt = get_altitude(vals[1] * 100.0f);
        }

        data_row_count++;

        if (data_row_count <= 10) {
            last_time_ms = curr_time_ms; 
            continue; 
        }

        // ==========================================
        //         START CORE MATH TIMER
        // ==========================================
        unsigned long math_start = micros();

        float dt = (curr_time_ms - last_time_ms) / 1000.0f;
        if (dt <= 0) dt = 0.001f; 
        M.Sensors.dt = dt;

        float pressure_pa = vals[1] * 100.0f;
        M.Sensors.Barometer = get_altitude(pressure_pa) - base_alt;

        M.Sensors.Accelerometer << vals[3] * GRAVITY_CONST, vals[4] * GRAVITY_CONST, vals[5] * GRAVITY_CONST;
        M.Sensors.AccelerometerHG << -vals[9] * GRAVITY_CONST, -vals[10] * GRAVITY_CONST, vals[11] * GRAVITY_CONST; 
        M.Sensors.Gyroscope << vals[6] * DEG_TO_RAD, vals[7] * DEG_TO_RAD, vals[8] * DEG_TO_RAD;
        M.Sensors.GPS.setZero(); 

        Eigen::Vector3f raw_mag(vals[12], vals[13], vals[14]);
        if (M.apogee == true) { 
            M.Sensors.Magnetometer = SOFT_IRON_DESCENT * (raw_mag - HARD_IRON_DESCENT);
        } else {
            M.Sensors.Magnetometer = SOFT_IRON_ASCENT * (raw_mag - HARD_IRON_ASCENT);
        }

        AB_loop(M);

        float v_horiz = sqrt(M.HorizState.Velocity_North * M.HorizState.Velocity_North + 
                             M.HorizState.Velocity_East * M.HorizState.Velocity_East);
        float zenith_deg = atan2(v_horiz, M.VertState.Velocity_Up) * RAD_TO_DEG;
        
        float airbrake_pct = 0.0f;
        if ((M.burning || M.burnout) && !M.apogee) {
            float apogeeIC[] = {M.VertState.Altitude, M.VertState.Velocity_Up, (float)(zenith_deg * DEG_TO_RAD), 0.0f};
            float predicted_val = (float)PredictDeploymentAngle(apogeeIC, DA_Vars, AP_Vars);
            airbrake_pct = constrain(predicted_val * 100.0f, 0.0f, 100.0f);
        }

        unsigned long math_end = micros();
        total_math_time_us += (math_end - math_start);
        // ==========================================
        //          END CORE MATH TIMER
        // ==========================================

        // --- I/O: LOGGING (NOT TIMED) ---
        float time_sec = curr_time_ms / 1000.0f;
        if (time_sec > 14780.0f) {
            outputFile.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                time_sec, M.VertState.Altitude, 
                M.HorizState.Position_North, M.HorizState.Position_East,
                M.VertState.Velocity_Up, M.HorizState.Velocity_North, M.HorizState.Velocity_East,
                zenith_deg, airbrake_pct);
        }

        last_time_ms = curr_time_ms;
        if (data_row_count % 100 == 0) { Serial.print("."); yield(); }
    }

    inputFile.close();
    outputFile.close();
    unsigned long total_real_time = millis() - startTime;

    // --- BENCHMARK RESULTS ---
    Serial.println("\n\n--- BENCHMARK COMPLETE ---");
    Serial.printf("Rows Processed: %d\n", data_row_count);
    
    Serial.printf("\n[CORE MATH ONLY] (Filter + Predictor)\n");
    Serial.printf("Total Math Time:  %.2f ms\n", total_math_time_us / 1000.0f);
    Serial.printf("Avg Time/Loop:    %.4f ms\n", (total_math_time_us / 1000.0f) / data_row_count);
    Serial.printf("Max Filter Rate:  %.2f Hz\n", (data_row_count * 1000.0f) / (total_math_time_us / 1000.0f));

    Serial.printf("\n[FULL REPLAY] (Including File I/O)\n");
    Serial.printf("Total Real Time:  %lu ms\n", total_real_time);

    log_memory();
}

void loop() {
    if (Serial.read() == 'd') { 
        File results = LittleFS.open("/flight_results.txt", "r");
        if (results) {
            while (results.available()) Serial.write(results.read());
            results.close();
        }
    }
}