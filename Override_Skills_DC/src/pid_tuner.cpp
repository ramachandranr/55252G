#include "main.h"
#include "lemlib/api.hpp"
#include "pid_tuner.hpp"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Imu imu;
extern pros::Controller master;
extern lemlib::Chassis chassis;

namespace {

// ==========================================
// Tuning Constants
// ==========================================
constexpr int LOOP_MS = 10;                  // PID loop period
constexpr int SETTLE_MS = 200;               // time inside the settle band that ends a test
constexpr int REST_MS = 500;                 // pause between tests so the robot stops moving
constexpr double ANGULAR_TEST_DEG = 90;      // LemLib docs tune the angular PID with turnToHeading(90, ...)
constexpr double LINEAR_TEST_IN = 48;        // LemLib docs use 48" (2 field tiles) for lateral motions
constexpr double TIMEOUT_PENALTY = 500000;   // added to the cost when a test never settles
// Returned for an aborted test or a gain that is never run. Infinity, not a big number: a real
// timed-out cost can exceed any fixed value, and a failed trial must never look like an improvement.
constexpr double FAILED_COST = std::numeric_limits<double>::infinity();
constexpr int MAX_ITERATIONS = 15;
constexpr double TOLERANCE = 0.5;            // twiddle stops once the step sizes sum to less than this

bool is_aborted = false;
const char* abort_message = "ABORTED";

// ==========================================
// Controller Output
// All tuner messages go to the controller; the brain LCD is never used.
// ==========================================

// The controller only accepts a print about every 50 ms, hence the delay.
void ctrlClear() {
    master.clear();
    pros::delay(60);
}

__attribute__((format(printf, 2, 3)))
void ctrlPrint(std::uint8_t line, const char* fmt, ...) {
    char text[24];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    // Pad so a shorter message fully overwrites the previous one on this line
    master.print(line, 0, "%-19s", text);
    pros::delay(60);
}

// ==========================================
// One Tunable Movement (angular or linear)
// ==========================================
struct Axis {
    const char* name;           // shown on the controller
    double far_target;          // the test alternates between far_target and home_target
    double home_target;
    double target;              // the target for the next test
    double settle_error;        // |error| inside this counts as settled
    int timeout_ms;
    double (*getError)(double target);   // NaN if the sensor failed
    void (*drive)(double power);
};

double angularError(double target) {
    double heading = imu.get_heading();
    if (!std::isfinite(heading)) return NAN;
    return std::remainder(target - heading, 360.0);
}

double linearError(double target) {
    return target - chassis.getPose().y;
}

void driveAngular(double power) {
    left_motors.move(power);
    right_motors.move(-power);
}

void driveLinear(double power) {
    left_motors.move(power);
    right_motors.move(power);
}

void stopDrive() {
    left_motors.brake();
    right_motors.brake();
}

// brake() uses the configured brake mode, and the drivetrain defaults to coast, which would let the
// robot roll on after every test. Hold it for the tuner's duration, then put coast back (the mode
// initSubsystems() gives the drivetrain).
void setDriveBrake(pros::motor_brake_mode_e_t mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
    stopDrive();
}

struct DriveBrakeHold {
    DriveBrakeHold() { setDriveBrake(pros::E_MOTOR_BRAKE_HOLD); }
    ~DriveBrakeHold() { setDriveBrake(pros::E_MOTOR_BRAKE_COAST); }
};

// Runs one move with the given gains and returns its cost (lower is better).
// Cost is the sum of squared error, so it rewards moves that are fast and don't overshoot.
// The control law is LemLib's own lemlib::PID, so the gains behave as they will in the chassis.
// Like the docs' procedure, only kP and kD are in play: no anti-windup, exit ranges or slew.
double runTest(Axis& axis, double kP, double kI, double kD) {
    lemlib::PID pid(kP, kI, kD);
    double cost = 0;
    int elapsed = 0, settled = 0;

    while (settled < SETTLE_MS && elapsed < axis.timeout_ms) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            is_aborted = true;
            abort_message = "ABORTED";
            stopDrive();
            return FAILED_COST;
        }

        double error = axis.getError(axis.target);
        if (!std::isfinite(error)) {
            is_aborted = true;
            abort_message = "SENSOR ERROR";
            stopDrive();
            return FAILED_COST;
        }

        if (std::abs(error) < axis.settle_error) settled += LOOP_MS;
        else settled = 0;

        axis.drive(std::clamp(static_cast<double>(pid.update(error)), -127.0, 127.0));

        cost += error * error;
        pros::delay(LOOP_MS);
        elapsed += LOOP_MS;
    }

    stopDrive();
    if (settled < SETTLE_MS) cost += TIMEOUT_PENALTY;   // ran out of time without settling

    axis.target = (axis.target == axis.far_target) ? axis.home_target : axis.far_target;
    pros::delay(REST_MS);
    return cost;
}

// ==========================================
// Twiddle Math Engine
// ==========================================
struct Gains { double p, d; };

// Nudges one gain at a time: keep a change if the cost drops (and grow its step),
// otherwise undo it (and shrink its step). kI is not tuned, so its step stays 0.
Gains twiddle(Axis& axis, Gains start, Gains step) {
    double p[3] = {start.p, 0.0, start.d};
    double dp[3] = {step.p, 0.0, step.d};
    double best_p[3] = {p[0], p[1], p[2]};

    // Show the gains being tested, run the move, and return its cost.
    // A negative gain would drive the robot away from the target, so it is never run.
    auto trial = [&]() {
        if (p[0] < 0 || p[2] < 0) return FAILED_COST;
        ctrlPrint(2, "Tst P:%.2f D:%.1f", p[0], p[2]);
        return runTest(axis, p[0], p[1], p[2]);
    };

    ctrlPrint(0, "%s INIT...", axis.name);
    double best_err = trial();
    if (is_aborted) return {best_p[0], best_p[2]};

    // Returns true if the trial improved on the best cost so far
    auto accept = [&](double err, int i) {
        if (err >= best_err) return false;
        best_err = err;
        dp[i] *= 1.1;
        std::copy(p, p + 3, best_p);
        return true;
    };

    for (int iteration = 1; iteration <= MAX_ITERATIONS && dp[0] + dp[1] + dp[2] > TOLERANCE; iteration++) {
        ctrlPrint(0, "%s Itr:%d/%d", axis.name, iteration, MAX_ITERATIONS);
        ctrlPrint(1, "Bst P:%.2f D:%.1f", best_p[0], best_p[2]);

        // Conditions drift as the tuner runs (battery sag, warming motors, wheel slip), so a cost
        // measured earlier is not comparable to a new one. Re-measure the best gains each round
        // (p equals best_p here) and judge this round's changes against that fresh number.
        if (iteration > 1) {
            best_err = trial();
            if (is_aborted) return {best_p[0], best_p[2]};
        }
        for (int i = 0; i < 3; i++) {
            if (dp[i] == 0.0) continue;   // nothing to nudge (kI): a trial would just repeat the last one

            p[i] += dp[i];
            double err = trial();
            if (is_aborted) return {best_p[0], best_p[2]};
            if (accept(err, i)) continue;

            p[i] -= 2 * dp[i];
            err = trial();
            if (is_aborted) return {best_p[0], best_p[2]};
            if (accept(err, i)) continue;

            p[i] += dp[i];
            dp[i] *= 0.9;
        }
    }
    return {best_p[0], best_p[2]};
}

void showAborted() {
    ctrlClear();
    ctrlPrint(0, "%s", abort_message);
}

// Blocks until A is pressed. Returns false (and flags an abort) if B is pressed first.
bool waitForContinue() {
    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);   // drop any press from before the prompt

    while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            is_aborted = true;
            abort_message = "ABORTED";
            return false;
        }
        pros::delay(20);
    }
    return true;
}

}  // namespace

// ==========================================
// Main Tuner Sequence
// ==========================================
void runTwiddleTuner() {
    is_aborted = false;
    DriveBrakeHold brakeHold;   // coast is restored on every return path

    Axis angular = {"ANG", ANGULAR_TEST_DEG, 0.0, ANGULAR_TEST_DEG, 1.5, 2500, angularError, driveAngular};
    Axis linear = {"LIN", LINEAR_TEST_IN, 0.0, LINEAR_TEST_IN, 1.0, 4000, linearError, driveLinear};

    // Starting gains are the LemLib docs' example values (tutorial "4 - PID Tuning"):
    //   angular kP 2, kD 10    lateral kP 10, kD 3    (kI stays 0: the docs call it a last resort)
    // Each step is about 10% of its gain.

    // --- PHASE 1: ANGULAR ---
    ctrlClear();
    imu.tare_heading();
    Gains ang = twiddle(angular, {2.0, 10.0}, {0.2, 1.0});
    if (is_aborted) return showAborted();

    // --- WAIT FOR THE DRIVER: lets them read the angular result and clear 48" in front of the robot ---
    ctrlClear();
    ctrlPrint(0, "ANG P:%.2f D:%.2f", ang.p, ang.d);
    ctrlPrint(1, "Clear 48in ahead");
    ctrlPrint(2, "A:LIN go  B:abort");
    setDriveBrake(pros::E_MOTOR_BRAKE_COAST);   // free the wheels so the robot can be moved by hand
    if (!waitForContinue()) return showAborted();
    setDriveBrake(pros::E_MOTOR_BRAKE_HOLD);

    // --- PHASE 2: LINEAR ---
    ctrlClear();
    chassis.setPose(0, 0, 0);   // zeroed after the wait, so the robot can be repositioned by hand
    Gains lin = twiddle(linear, {10.0, 3.0}, {1.0, 0.3});
    if (is_aborted) return showAborted();

    // --- COMPLETE: DISPLAY FINAL RESULTS ---
    ctrlClear();
    ctrlPrint(0, "ANG P:%.2f D:%.2f", ang.p, ang.d);
    ctrlPrint(1, "LIN P:%.2f D:%.2f", lin.p, lin.d);
    ctrlPrint(2, "ALL COMPLETE!");
}
