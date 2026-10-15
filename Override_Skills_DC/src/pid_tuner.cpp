#include "main.h"
#include "pid_tuner.hpp"
#include <cmath>

// Pull in the hardware defined in main.cpp
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Imu imu;
extern pros::Controller master;

double test_target = 90.0; 

double runTestMovement(double kP, double kI, double kD) {
    double current = imu.get_heading();
    double error = test_target - current;
    
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    double prev_error = error;
    double integral = 0;
    double total_cost = 0;
    int time_elapsed = 0;
    int timeout = 2500; 

    while (std::abs(error) > 1.5 && time_elapsed < timeout) {
        current = imu.get_heading();
        
        error = test_target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        integral += error;
        double derivative = error - prev_error;

        double power = (error * kP) + (integral * kI) + (derivative * kD);
        
        left_motors.move(power);
        right_motors.move(-power);

        total_cost += (error * error);
        prev_error = error;
        
        pros::delay(10);
        time_elapsed += 10;
    }
    
    left_motors.move(0);
    right_motors.move(0);
    
    // Alternate target for the next loop (drives back and forth)
    test_target = (test_target == 90.0) ? 0.0 : 90.0;
    
    pros::delay(500); 
    return total_cost;
}

void runTwiddleTuner() {
    pros::lcd::clear_line(4);
    pros::lcd::print(4, "TUNING IN PROGRESS...");
    master.print(0, 0, "Tuning...");

    double p[3] = {2.0, 0.0, 10.0}; 
    double dp[3] = {0.5, 0.01, 2.0}; 
    
    double best_err = runTestMovement(p[0], p[1], p[2]);
    double tolerance = 0.1; 

    while(dp[0] + dp[1] + dp[2] > tolerance) {
        for(int i = 0; i < 3; i++) {
            p[i] += dp[i]; 
            double err = runTestMovement(p[0], p[1], p[2]);

            if(err < best_err) {
                best_err = err;
                dp[i] *= 1.1; 
            } else {
                p[i] -= 2 * dp[i]; 
                err = runTestMovement(p[0], p[1], p[2]);

                if(err < best_err) {
                    best_err = err;
                    dp[i] *= 1.1;
                } else {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }
        printf("kP: %.2f, kI: %.2f, kD: %.2f | Cost: %.0f\n", p[0], p[1], p[2], best_err);
    }
    
    printf("\n=== TUNING COMPLETE ===\n");
    printf("kP: %.2f\nkI: %.2f\nkD: %.2f\n", p[0], p[1], p[2]);
    
    pros::lcd::clear_line(4);
    pros::lcd::print(4, "DONE! P:%.1f D:%.1f", p[0], p[2]);
    master.print(0, 0, "P:%.1f D:%.1f", p[0], p[2]);
}