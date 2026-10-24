#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/exitcondition.hpp"
#include "lemlib/util.hpp"
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
extern lemlib::ControllerSettings linearController;    // exit rule and slew come from these, so a test
extern lemlib::ControllerSettings angularController;   // ends the way the real motion would

namespace {

// ==========================================
// Tuning Constants
// ==========================================
constexpr int LOOP_MS = 10;                  // PID loop period
constexpr int REST_MS = 500;                 // pause after each move so the robot stops moving
constexpr int PROGRESS_CHECK_MS = 500;       // by this time into a move the error must have started to shrink
constexpr double ANGULAR_TEST_DEG = 90;      // LemLib docs tune the angular PID with turnToHeading(90, ...)
constexpr double LINEAR_TEST_IN = 24;        // one field tile; the routine's drives are mostly 18-22". Keep it above 4x the linear largeError (12") or the NO PROGRESS check is skipped
constexpr double TIMEOUT_PENALTY = 500000;   // added to the cost of each move that never settles
constexpr double SLEW_FAR = 8;               // slew only limits the launch: it is off within this much (deg or in) of the target
// Power cap during a move, matching the maxSpeed the routine's moves use. Its drives are capped at 65-100;
// three of its four turns are uncapped (127), so the angular cap stays at full power.
constexpr double ANGULAR_MAX_POWER = 127;
constexpr double LINEAR_MAX_POWER = 100;
constexpr double HEADING_HOLD_KP = 2;        // linear test: steering power per degree of heading drift
constexpr double HEADING_HOLD_MAX = 20;      // ...and the most steering it may add or take away
constexpr double ACCEPT_MARGIN = 0.05;       // a candidate must beat the best cost by this fraction to count
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
// A per-trial log also goes to the terminal (`pros terminal`).
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
    const char* name;                          // shown on the controller
    double far_target;                         // each trial goes out to far_target and back to home_target
    double home_target;
    const lemlib::ControllerSettings& settings;   // exit rule (small/large error) and slew, as the chassis uses them
    int timeout_ms;                            // longest a single move may take
    double max_power;                          // the PID output is capped at this (0-127)
    double (*getError)(double target);         // NaN if the sensor failed
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

// Drives both sides at `power` and steers to keep the heading the pose was zeroed to (0). Without this the
// robot curves over many back-and-forth trials, its forward motion stops lining up with the Y axis the error
// is read from, and later trials are no longer like the first. The steering is small and capped, so the
// forward power still decides the move.
void driveLinear(double power) {
    double heading_error = std::remainder(0.0 - chassis.getPose().theta, 360.0);   // + means turn clockwise
    double turn = 0;
    if (std::isfinite(heading_error)) turn = std::clamp(HEADING_HOLD_KP * heading_error, -HEADING_HOLD_MAX, HEADING_HOLD_MAX);
    left_motors.move(std::clamp(power + turn, -127.0, 127.0));
    right_motors.move(std::clamp(power - turn, -127.0, 127.0));
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

struct Gains { double p, d; };

// The result of one move to one target
struct Leg {
    double cost;     // FAILED_COST if the move was aborted
    bool settled;    // false if it ran out of time
    int ms;          // how long it ran
};

Leg abortLeg(const char* message, int elapsed) {
    is_aborted = true;
    abort_message = message;
    stopDrive();
    return {FAILED_COST, false, elapsed};
}

// Runs one move to `target` with the given gains and returns its cost (lower is better).
// Cost is the sum of squared error, so it rewards moves that are fast and don't overshoot.
// The control law is LemLib's own lemlib::PID, a move ends on the chassis's own small/large error exits,
// and the launch is limited by the chassis's slew, so the gains behave as they will in a real motion.
// Only kP and kD are in play: no anti-windup.
Leg runLeg(const Axis& axis, double kP, double kI, double kD, double target) {
    lemlib::PID pid(kP, kI, kD);
    lemlib::ExitCondition smallExit(axis.settings.smallError, static_cast<int>(axis.settings.smallErrorTimeout));
    lemlib::ExitCondition largeExit(axis.settings.largeError, static_cast<int>(axis.settings.largeErrorTimeout));
    double cost = 0;
    double start_error = 0;   // |error| at the first reading of this move
    double last_power = 0;
    int elapsed = 0;
    bool settled = false;

    while (elapsed < axis.timeout_ms) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) return abortLeg("ABORTED", elapsed);

        // With the controller gone, B can't stop the test, so stop it here
        if (!master.is_connected()) return abortLeg("CTRL LOST", elapsed);

        double error = axis.getError(target);
        if (!std::isfinite(error)) return abortLeg("SENSOR ERROR", elapsed);

        // Runaway guard: the motors are driving at full tilt, so within half a second the error must have
        // moved toward zero. If it has not, the sensor is dead or reads the wrong way (the gains cannot
        // fix that, and the robot would drive until the timeout), or the robot is blocked.
        if (elapsed == 0) {
            start_error = std::abs(error);
        } else if (elapsed == PROGRESS_CHECK_MS && start_error > 4 * axis.settings.largeError &&
                   std::abs(error) >= start_error) {
            return abortLeg("NO PROGRESS", elapsed);
        }

        smallExit.update(error);
        largeExit.update(error);
        if (smallExit.getExit() || largeExit.getExit()) {
            settled = true;
            break;
        }

        double power = std::clamp(static_cast<double>(pid.update(error)), -axis.max_power, axis.max_power);
        // Limit the launch the way the chassis does, but not the braking near the target
        if (std::abs(error) > SLEW_FAR) power = lemlib::slew(power, last_power, axis.settings.slew);
        last_power = power;
        axis.drive(power);

        cost += error * error;
        pros::delay(LOOP_MS);
        elapsed += LOOP_MS;
    }

    stopDrive();
    if (!settled) cost += TIMEOUT_PENALTY;   // ran out of time without settling
    pros::delay(REST_MS);
    return {cost, settled, elapsed};
}

// Puts the robot back at the start before a trial, using the best gains found so far. This move is not
// scored. A trial that timed out leaves the robot short of home, and the next trial must not start there.
void returnHome(const Axis& axis, Gains reset) {
    double error = axis.getError(axis.home_target);
    if (!std::isfinite(error)) {
        abortLeg("SENSOR ERROR", 0);
        return;
    }
    if (std::abs(error) <= 0.5 * axis.settings.largeError) return;   // already home

    Leg leg = runLeg(axis, reset.p, 0.0, reset.d, axis.home_target);
    printf("[%s] reset to start from %.1f off: %s %dms\n", axis.name, error, leg.settled ? "ok" : "TIMEOUT", leg.ms);
}

// One trial: go out to far_target and back to home_target with the given gains, and return the total
// cost of both moves. Measuring both directions, from the same starting point, keeps trials comparable.
double runTrial(const Axis& axis, double kP, double kI, double kD, Gains reset) {
    returnHome(axis, reset);
    if (is_aborted) return FAILED_COST;

    Leg out = runLeg(axis, kP, kI, kD, axis.far_target);
    if (is_aborted) return FAILED_COST;
    Leg back = runLeg(axis, kP, kI, kD, axis.home_target);
    if (is_aborted) return FAILED_COST;

    double cost = out.cost + back.cost;
    printf("[%s] P=%.2f D=%.2f  cost=%.0f  out %s %dms  back %s %dms\n", axis.name, kP, kD, cost,
           out.settled ? "ok" : "TIMEOUT", out.ms, back.settled ? "ok" : "TIMEOUT", back.ms);
    return cost;
}

// ==========================================
// Twiddle Math Engine
// ==========================================

// Nudges one gain at a time: keep a change if the cost drops clearly (and grow its step),
// otherwise undo it (and shrink its step). kI is not tuned, so its step stays 0.
Gains twiddle(const Axis& axis, Gains start, Gains step) {
    double p[3] = {start.p, 0.0, start.d};
    double dp[3] = {step.p, 0.0, step.d};
    double best_p[3] = {p[0], p[1], p[2]};

    // Show the gains being tested, run the trial, and return its cost.
    // A negative gain would drive the robot away from the target, so it is never run.
    auto trial = [&]() {
        if (p[0] < 0 || p[2] < 0) return FAILED_COST;
        ctrlPrint(2, "Tst P:%.2f D:%.1f", p[0], p[2]);
        return runTrial(axis, p[0], p[1], p[2], Gains{best_p[0], best_p[2]});
    };

    ctrlPrint(0, "%s INIT...", axis.name);
    // A single run is noisy, so the best gains are judged by a running average: each new
    // measurement counts half, which keeps one lucky or unlucky run from setting the bar.
    double best_cost = trial();
    if (is_aborted) return {best_p[0], best_p[2]};

    // Returns true if the trial beat the best cost by the margin
    auto accept = [&](double err, int i) {
        if (err >= best_cost * (1.0 - ACCEPT_MARGIN)) return false;
        best_cost = err;
        dp[i] *= 1.1;
        std::copy(p, p + 3, best_p);
        printf("[%s] accepted: best is now P=%.2f D=%.2f (cost %.0f)\n", axis.name, best_p[0], best_p[2], best_cost);
        return true;
    };

    for (int iteration = 1; iteration <= MAX_ITERATIONS && dp[0] + dp[1] + dp[2] > TOLERANCE; iteration++) {
        ctrlPrint(0, "%s Itr:%d/%d", axis.name, iteration, MAX_ITERATIONS);
        ctrlPrint(1, "Bst P:%.2f D:%.1f", best_p[0], best_p[2]);
        printf("[%s] round %d/%d: best P=%.2f D=%.2f, average cost %.0f\n", axis.name, iteration, MAX_ITERATIONS,
               best_p[0], best_p[2], best_cost);

        // Conditions drift as the tuner runs (battery sag, warming motors, wheel slip), so a cost
        // measured earlier is not comparable to a new one. Re-measure the best gains each round
        // (p equals best_p here) and fold it into their average, which follows the drift.
        if (iteration > 1) {
            double again = trial();
            if (is_aborted) return {best_p[0], best_p[2]};
            best_cost = 0.5 * (best_cost + again);
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

    Axis angular = {"ANG", ANGULAR_TEST_DEG, 0.0, angularController, 2500, ANGULAR_MAX_POWER, angularError, driveAngular};
    Axis linear = {"LIN", LINEAR_TEST_IN, 0.0, linearController, 4000, LINEAR_MAX_POWER, linearError, driveLinear};

    // Starting gains are the LemLib docs' example values (tutorial "4 - PID Tuning"):
    //   angular kP 2, kD 10    lateral kP 10, kD 3    (kI stays 0: the docs call it a last resort)
    // Each step is about 10% of its gain.

    // --- PHASE 1: ANGULAR ---
    ctrlClear();
    imu.tare_heading();
    Gains ang = twiddle(angular, {2.0, 10.0}, {0.2, 1.0});
    if (is_aborted) return showAborted();

    // --- WAIT FOR THE DRIVER: lets them read the angular result and clear LINEAR_TEST_IN in front of the robot ---
    ctrlClear();
    ctrlPrint(0, "ANG P:%.2f D:%.2f", ang.p, ang.d);
    ctrlPrint(1, "Clear %.0fin ahead", LINEAR_TEST_IN);
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
