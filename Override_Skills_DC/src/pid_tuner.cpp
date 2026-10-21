#include "main.h" 
#include "lemlib/api.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Imu imu;
extern pros::Controller master; 
extern lemlib::Chassis chassis; 

extern char tuner_status[32];
extern int tuner_iteration;
extern int tuner_max_iterations;
extern double tuner_kP, tuner_kI, tuner_kD;
extern double tuner_best_cost;

// Global Targets and Flags
double target_ang = 90.0; 
double target_lin = 24.0;
bool is_aborted = false;

// ==========================================
// Phase 1: Angular Movement Logic
// ==========================================
double runAngularTest(double kP, double kI, double kD, int iteration) {
    double current = imu.get_heading();
    if (std::isinf(current) || std::isnan(current)) return 999999; 

    double error = std::remainder(target_ang - current, 360.0);
    double prev_error = error;
    double integral = 0, total_cost = 0;
    int time_elapsed = 0, settled_time = 0, timeout = 2500; 

    while (settled_time < 200 && time_elapsed < timeout) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            is_aborted = true;
            left_motors.brake(); right_motors.brake();
            return 999999; 
        }

        current = imu.get_heading();
        if (std::isinf(current) || std::isnan(current)) break; 
        
        error = std::remainder(target_ang - current, 360.0);

        if (std::abs(error) < 1.5) settled_time += 10;
        else settled_time = 0; 

        if (std::abs(error) < 15.0) integral += error;
        else integral = 0; 
        if (integral > 300) integral = 300; else if (integral < -300) integral = -300;

        double derivative = error - prev_error;
        double power = (error * kP) + (integral * kI) + (derivative * kD);
        if (power > 127) power = 127; else if (power < -127) power = -127;

        left_motors.move(power);
        right_motors.move(-power); 

        total_cost += (error * error);
        prev_error = error;
        pros::delay(10); time_elapsed += 10;
    }
    
    left_motors.brake(); right_motors.brake();
    if (time_elapsed >= timeout) total_cost += 500000; 
    
    target_ang = (target_ang == 90.0) ? 0.0 : 90.0;
    pros::delay(500); 
    return total_cost;
}

// ==========================================
// Phase 2: Linear Movement Logic
// ==========================================
double runLinearTest(double kP, double kI, double kD, int iteration) {
    double current = chassis.getPose().y;
    double error = target_lin - current;
    double prev_error = error;
    double integral = 0, total_cost = 0;
    int time_elapsed = 0, settled_time = 0, timeout = 3000; 

    while (settled_time < 200 && time_elapsed < timeout) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            is_aborted = true;
            left_motors.brake(); right_motors.brake();
            return 999999; 
        }

        current = chassis.getPose().y;
        error = target_lin - current;

        if (std::abs(error) < 1.0) settled_time += 10;
        else settled_time = 0; 

        if (std::abs(error) < 5.0) integral += error;
        else integral = 0; 
        if (integral > 300) integral = 300; else if (integral < -300) integral = -300;

        double derivative = error - prev_error;
        double power = (error * kP) + (integral * kI) + (derivative * kD);
        if (power > 127) power = 127; else if (power < -127) power = -127;

        left_motors.move(power);
        right_motors.move(power); 

        total_cost += (error * error);
        prev_error = error;
        pros::delay(10); time_elapsed += 10;
    }
    
    left_motors.brake(); right_motors.brake();
    if (time_elapsed >= timeout) total_cost += 500000; 
    
    target_lin = (target_lin == 24.0) ? 0.0 : 24.0;
    pros::delay(500); 
    return total_cost;
}

// ==========================================
// Twiddle Math Engine
// ==========================================
typedef double (*TestFunc)(double, double, double, int);

void runTwiddleLogic(TestFunc testFunc, const char* prefix, double start_p, double start_d, double start_dp, double start_dd, double &out_p, double &out_d) {
    double p[3] = {start_p, 0.0, start_d}; 
    double dp[3] = {start_dp, 0.0, start_dd}; 
    
    master.print(0, 0, "%s INIT...      ", prefix); pros::delay(60);
    master.print(2, 0, "Tst P:%.2f D:%.1f", p[0], p[2]); pros::delay(60);
    double best_err = testFunc(p[0], p[1], p[2], 0);
    
    const double TOLERANCE = 0.5; 
    tuner_iteration = 1;
    double best_p[3] = {p[0], p[1], p[2]};

    while ((dp[0] + dp[1] + dp[2] > TOLERANCE) && (tuner_iteration <= tuner_max_iterations)) {
        if (is_aborted) break;

        // TAG ADDED HERE: "ANG Itr: 1/15" or "LIN Itr: 1/15"
        master.print(0, 0, "%s Itr:%d/%d  ", prefix, tuner_iteration, tuner_max_iterations); pros::delay(60);
        master.print(1, 0, "Bst P:%.2f D:%.1f", best_p[0], best_p[2]); pros::delay(60);

        for(int i = 0; i < 3; i++) {
            if (is_aborted) break;

            p[i] += dp[i]; 
            master.print(2, 0, "Tst P:%.2f D:%.1f", p[0], p[2]); pros::delay(60);
            
            double err = testFunc(p[0], p[1], p[2], tuner_iteration);

            if(err < best_err) {
                best_err = err;
                dp[i] *= 1.1; 
                best_p[0] = p[0]; best_p[1] = p[1]; best_p[2] = p[2];
            } else {
                p[i] -= 2 * dp[i]; 
                master.print(2, 0, "Tst P:%.2f D:%.1f", p[0], p[2]); pros::delay(60);
                
                err = testFunc(p[0], p[1], p[2], tuner_iteration);

                if(err < best_err) {
                    best_err = err;
                    dp[i] *= 1.1;
                    best_p[0] = p[0]; best_p[1] = p[1]; best_p[2] = p[2];
                } else {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }
        tuner_iteration++;
    }
    out_p = best_p[0];
    out_d = best_p[2];
}

// ==========================================
// Main Tuner Sequence
// ==========================================
void runTwiddleTuner() {
    is_aborted = false;
    tuner_max_iterations = 15; 
    
    double final_ang_p = 0, final_ang_d = 0;
    double final_lin_p = 0, final_lin_d = 0;

    // --- PHASE 1: ANGULAR ---
    master.clear(); pros::delay(60);
    snprintf(tuner_status, sizeof(tuner_status), "Tuning Angular");
    
    imu.tare_heading();
    target_ang = 90.0;
    // Passes "ANG" to prefix the screen
    runTwiddleLogic(runAngularTest, "ANG", 3, 15.0, 0.2, 1.0, final_ang_p, final_ang_d);
    
    if (is_aborted) {
        master.clear(); pros::delay(60);
        master.print(0, 0, "ABORTED         "); pros::delay(60);
        return;
    }

    // --- PHASE 2: LINEAR ---
    master.clear(); pros::delay(60);
    snprintf(tuner_status, sizeof(tuner_status), "Tuning Linear");
    
    chassis.setPose(0, 0, 0); 
    target_lin = 24.0;
    // Passes "LIN" to prefix the screen
    runTwiddleLogic(runLinearTest, "LIN", 11.0, 2.5, 1.0, 0.5, final_lin_p, final_lin_d);

    if (is_aborted) {
        master.clear(); pros::delay(60);
        master.print(0, 0, "ABORTED         "); pros::delay(60);
        return;
    }

    // --- COMPLETE: DISPLAY FINAL RESULTS ---
    master.clear(); pros::delay(60);
    master.print(0, 0, "ANG P:%.2f D:%.2f", final_ang_p, final_ang_d); pros::delay(60);
    master.print(1, 0, "LIN P:%.2f D:%.2f", final_lin_p, final_lin_d); pros::delay(60);
    master.print(2, 0, "ALL COMPLETE!   "); pros::delay(60);
    snprintf(tuner_status, sizeof(tuner_status), "ALL COMPLETE!");
}