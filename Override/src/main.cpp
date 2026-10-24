#include "main.h"
#include "lemlib/api.hpp"
#include "pid_tuner.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <vector>

// ==========================================
// Hardware Configuration
// ==========================================

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-19, -20}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({11, 12}, pros::MotorGearset::blue);

pros::Motor intake(1, pros::MotorGearset::blue);
pros::MotorGroup cascade({3, -9}, pros::MotorGearset::blue);
pros::Motor clamp_rollers(2, pros::MotorGearset::green);
pros::Motor clamp_wrist(10, pros::MotorGearset::green);

pros::adi::DigitalIn bumper('A');
pros::Imu imu(13);
pros::Rotation horizontal(18);
lemlib::TrackingWheel horizontal_sensor(&horizontal, lemlib::Omniwheel::NEW_2, 1.5, 1);

// ==========================================
// LemLib Configuration
// ==========================================

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.6, lemlib::Omniwheel::NEW_275, 450, 8);
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2.94, 0, 17.95, 3, 1, 100, 3, 500, 0);
lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal_sensor, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

std::atomic<bool> isArcadeMode{true};

// ==========================================
// PID Tuner Hand-off
// ==========================================
//
// The tuner reports only on the controller. While it runs, nothing else may use the
// brain LCD or the mechanisms. Rules that keep this crash-free:
//   - LCD button callbacks only set a flag; they never create tasks or touch hardware.
//   - The screen task is the only code that draws on the LCD, and it also starts the tuner.
//   - The tuner task deletes itself instead of returning.

std::atomic<bool> isTuning{false};
std::atomic<bool> tunerRequested{false};

// How long the final tuner result stays on the controller before driving resumes
const uint32_t TUNER_RESULT_HOLD_MS = 10000;

void toggleDriveMode() {
    isArcadeMode = !isArcadeMode;
}

void requestTuner() {
    tunerRequested = true;
}

void tuningTaskWrapper(void* param) {
    runTwiddleTuner();
    pros::delay(TUNER_RESULT_HOLD_MS);
    isTuning = false;

    // A task must never fall off the end of its function: delete it explicitly
    pros::c::task_delete(pros::c::task_get_current());
}

void startTuner() {
    if (isTuning.exchange(true)) return;   // already running

    // SAFE TASK SPAWNING: Using the native C API prevents memory aborts
    auto task = pros::c::task_create(tuningTaskWrapper, nullptr, TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Tuner Task");
    if (task == nullptr) isTuning = false;   // could not start: hand the robot back
}

// ==========================================
// Motor Diagnostics
// ==========================================
//
// Built entirely in fixed buffers: no std::vector, std::string or std::stable_sort,
// so the repeating diagnostic task never allocates from the heap.

constexpr int DIAG_LINES = 6;
constexpr int DIAG_LINE_LEN = 64;
constexpr double MAX_TEMP_C = 55.0;
constexpr int MAX_CURRENT_MA = 1200;

struct DiagLine {
    bool has_issue;
    char text[DIAG_LINE_LEN];
};

// Latest diagnostic text. Written by the diagnostic task, read by the screen task.
char diag_text[DIAG_LINES][DIAG_LINE_LEN] = {};
pros::Mutex diag_mutex;

// Describes every motor in a motor or motor group on one line
void buildDiagLine(DiagLine& line, const char* name, const pros::AbstractMotor& motor) {
    char body[DIAG_LINE_LEN] = "";
    size_t used = 0;
    line.has_issue = false;

    for (int i = 0; i < motor.size(); i++) {
        double temp = motor.get_temperature(i);
        int current = motor.get_current_draw(i);

        // A missing or unplugged motor reports PROS_ERR_F / PROS_ERR instead of real values
        bool disconnected = !std::isfinite(temp) || current == PROS_ERR;
        if (disconnected || temp >= MAX_TEMP_C || current > MAX_CURRENT_MA) line.has_issue = true;

        int written = disconnected ? snprintf(body + used, sizeof(body) - used, "| --C  N/C   ")
                                   : snprintf(body + used, sizeof(body) - used, "| %2.0fC %4dmA ", temp, current);
        if (written > 0) used = std::min(used + static_cast<size_t>(written), sizeof(body) - 1);
    }

    snprintf(line.text, sizeof(line.text), "%s%-5s%s", line.has_issue ? "[!] " : "[OK] ", name, body);
}

void diagnosticTaskWrapper(void* param) {
    while (true) {
        if (isTuning) {
            pros::delay(500);
            continue;
        }

        DiagLine lines[DIAG_LINES];
        buildDiagLine(lines[0], "L_Drv", left_motors);
        buildDiagLine(lines[1], "R_Drv", right_motors);
        buildDiagLine(lines[2], "Cascd", cascade);
        buildDiagLine(lines[3], "Intak", intake);
        buildDiagLine(lines[4], "ClmpR", clamp_rollers);
        buildDiagLine(lines[5], "ClmpW", clamp_wrist);

        {
            std::lock_guard<pros::Mutex> lock(diag_mutex);
            int row = 0;
            // Lines with a problem go first, otherwise keep the original order
            for (bool issues : {true, false}) {
                for (const DiagLine& line : lines) {
                    if (line.has_issue == issues) snprintf(diag_text[row++], DIAG_LINE_LEN, "%-40s", line.text);
                }
            }
        }
        pros::delay(2000);
    }
}

// ==========================================
// Screen Task
// ==========================================

void screenTaskWrapper(void* param) {
    while (true) {
        // A press while connected to a competition switch or field control is dropped: the tuner would
        // take over the drivetrain in the middle of a match or an autonomous run.
        if (tunerRequested.exchange(false) && !pros::competition::is_connected()) startTuner();

        // While tuning, the LCD is left untouched (no clear, no print);
        // everything is reported on the controller instead.
        if (!isTuning) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "[%s] X:%.1f Y:%.1f T:%.1f",
                             isArcadeMode ? "ARC" : "TNK", pose.x, pose.y, pose.theta);

            // Copy under the lock, print after releasing it
            char lines[DIAG_LINES][DIAG_LINE_LEN];
            {
                std::lock_guard<pros::Mutex> lock(diag_mutex);
                std::copy(&diag_text[0][0], &diag_text[0][0] + DIAG_LINES * DIAG_LINE_LEN, &lines[0][0]);
            }
            for (int i = 0; i < DIAG_LINES; i++) {
                pros::lcd::print(i + 1, "%s", lines[i]);
            }
        }
        pros::delay(200);
    }
}

// ==========================================
// Setup Functions
// ==========================================

void calibrateChassis() {
    chassis.calibrate();
}   

// Brake modes, and the cascade zero. Runs once at boot (from initialize), so autonomous gets the same
// brake modes as driver control and both use the same zero. The cascade must be fully lowered when the
// program starts (the Robot starts within 18" <SG1>); driver control re-zeroes it on the bumper.
void initSubsystems() {
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    clamp_rollers.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp_wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascade.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    cascade.tare_position_all();
}

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(7, "[Drive Mode]               [Tune PID]");

    initSubsystems();
    calibrateChassis();

    // Buttons are only live once calibration is done, so the tuner can't start on an uncalibrated IMU
    pros::lcd::register_btn0_cb(toggleDriveMode);
    pros::lcd::register_btn2_cb(requestTuner);

    // SAFE TASK SPAWNING: C API avoids all object-destruction memory aborts
    pros::c::task_create(diagnosticTaskWrapper, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Diag Task");
    pros::c::task_create(screenTaskWrapper, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Screen Task");
}

void disabled() {}
void competition_initialize() {}

// ==========================================
// Autonomous
// ==========================================
//
// Autonomous Coding Skills routine: one Robot, one 60 s autonomous, no driver period. The Robot starts in the
// Quadrant next to the red Alliance Station holding the red/yellow Pin preload. <RSC2>
// Points: each Scored red/blue Pin Half 5, each Scored yellow Pin Half 10 (only if Owned), Robot ending in the
// Midfield 8. <RSC3> The Robot may hold at most one Pin and one Cup at a time. <SG6>
//
// Coordinates are inches relative to the robot's start (the pose is zeroed at the beginning).
// Heading 0 faces the start direction (+Y) and increases clockwise. `forwards = false` drives backwards.
// Wrist positions are degrees from the wrist's tare point (taken at the start of autonomous).
// Cascade positions are degrees from the bottom (tared at boot); up is positive.
// Motor velocity arguments (move_absolute, move_relative) are RPM, not 0-127.
//
// LemLib motions run in the background by default (async). A new motion waits for the previous
// one to finish, while plain motor commands written right after a motion run while it is moving.
// The order of the statements below is therefore part of the routine; do not reorder them.
//
// Routine:
//   setupAutonomous  ->  runToggler  ->  moveToDropPin  ->  dropPin
//   ->  moveToPinAndCup1  ->  grabPinAndCup1  ->  moveToDropPinAndCup1  ->  dropPinAndCup1
//   ->  getPinAndCup2  ->  dropPinAndCup2  ->  park

// Start time of the autonomous run, for the log lines.
uint32_t autonStart = 0;

// When logStep last ran, so the next one can say how long the step in between took.
uint32_t lastStepStart = 0;

// The step logStep printed last, so the other log lines can say which step they belong to.
const char* currentStep = "";

// Passed to logArrival for a target that does not apply (a point move has no heading, a turn has no point).
constexpr double NO_TARGET = NAN;

// Room for one formatState line.
constexpr size_t STATE_LEN = 112;

// Formats the start of every log line, so any two lines can be compared field by field:
// the time since autonomous started, the pose (x, y in inches, theta in degrees), and the wrist and
// cascade positions.
void formatState(char* out, size_t size) {
    lemlib::Pose pose = chassis.getPose();
    snprintf(out, size, "[auton %6lu ms] x:%6.1f y:%6.1f t:%6.1f  wrist:%7.1f cascade:%7.1f",
             static_cast<unsigned long>(pros::millis() - autonStart), pose.x, pose.y, pose.theta,
             clamp_wrist.get_position(), cascade.get_position());
}

// Prints the state, the battery voltage, how long the previous step took, and the step that is starting. Read it
// with `pros terminal` to see how long each step really takes. The state is where the previous step ended, so
// compare it with that step's targets to see how far short (or long) it stopped. A low battery voltage makes
// every drive move slower and shorter.
void logStep(const char* step) {
    uint32_t now = pros::millis();
    char state[STATE_LEN];
    formatState(state, sizeof(state));
    printf("%s  battery:%5.2f V  prev step took:%5lu ms  >> %s\n", state, pros::battery::get_voltage() / 1000.0,
           static_cast<unsigned long>(now - lastStepStart), step);
    lastStepStart = now;
    currentStep = step;
}

// Average velocity (rpm) of a motor group. A motor reports PROS_ERR_F (inf) when it is unplugged.
double groupVelocity(const pros::AbstractMotor& group) {
    std::vector<double> velocities = group.get_actual_velocity_all();
    double sum = 0;
    for (double v : velocities) sum += v;
    return velocities.empty() ? 0 : sum / static_cast<double>(velocities.size());
}

// Prints where a drive motion or turn ended against where it was told to go. Call it right after the motion
// has ended (a blocking move, or after chassis.waitUntilDone()). `off` is the pose minus the target: dx and dy
// in inches, dist the straight-line miss, dt in degrees (wrapped to -180..180). Pass NO_TARGET for x and y
// (a turn) or for theta (a point move) to leave that part out. `drive` is the left and right wheel speed in rpm:
// not 0 means the robot was still moving when the motion ended.
void logArrival(const char* what, double x, double y, double theta) {
    lemlib::Pose pose = chassis.getPose();
    char state[STATE_LEN];
    formatState(state, sizeof(state));
    printf("%s  arrived %s", state, what);
    if (!std::isnan(x)) {
        printf("  target x:%6.1f y:%6.1f  off dx:%6.1f dy:%6.1f dist:%5.1f", x, y, pose.x - x, pose.y - y,
               std::hypot(pose.x - x, pose.y - y));
    }
    if (!std::isnan(theta)) {
        printf("  target t:%6.1f  off dt:%6.1f", theta, std::remainder(pose.theta - theta, 360.0));
    }
    printf("  drive L:%6.1f R:%6.1f rpm  in %s\n", groupVelocity(left_motors), groupVelocity(right_motors), currentStep);
}

// Zeroes the pose and sets the brake modes before anything moves.
void setupAutonomous() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);   // coast while the quick moves in runToggler start
    clamp_wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp_wrist.tare_position();                       // wrist positions below count from here
}

// Prints one wait log line: the state, then the motor against the target (err = position - target) with its
// velocity (rpm, 0 means not moving) and current draw, then `extra`, then the step.
// A stalled motor shows vel near 0 with a high cur; a slow one shows vel above 0.
void logWait(const char* name, const char* tag, const pros::AbstractMotor& motor, double position, double target,
             const char* extra) {
    char state[STATE_LEN];
    formatState(state, sizeof(state));
    printf("%s  wait %-7s %-7s pos:%7.1f target:%7.1f err:%7.1f  vel:%6.1f cur:%5ld mA  %s  in %s\n", state, name, tag,
           position, target, position - target, motor.get_actual_velocity(),
           static_cast<long>(motor.get_current_draw()), extra, currentStep);
}

// Waits until the motor (the first motor of a group) is within `tolerance` of `target`, or `timeoutMs` passes.
// move_absolute and move_relative return immediately, and a move_relative issued before the last
// one finishes starts from wherever the mechanism currently is. Use this before the next move of the same mechanism.
// The timeout is what keeps autonomous alive: a stalled motor never arrives, and an unplugged one reports
// infinity, so an untimed wait would freeze the rest of the run.
// `name` says which mechanism it is. Prints two wait lines (see logWait), both with the state and the step:
//   start:        also the target the motor was last commanded to (cmd), the tolerance and the time limit.
//                 `CMD MISMATCH` means cmd and the target waited for differ: the wait can never end well.
//   ok / TIMEOUT: when it ends, with the time waited. TIMEOUT means the motor never got within the tolerance.
//                 Its vel and cur tell a stall (vel near 0, high cur) from a slow move (vel above 0).
void waitForPosition(const char* name, const pros::AbstractMotor& motor, double target, int timeoutMs, double tolerance = 10) {
    uint32_t start = pros::millis();
    char extra[80];

    double commanded = motor.get_target_position();
    snprintf(extra, sizeof(extra), "cmd:%7.1f tol:%3.0f limit:%5d ms%s", commanded, tolerance, timeoutMs,
             std::abs(commanded - target) > 1 ? "  CMD MISMATCH" : "");
    logWait(name, "start", motor, motor.get_position(), target, extra);

    while (std::abs(motor.get_position() - target) > tolerance && pros::millis() - start < static_cast<uint32_t>(timeoutMs)) {
        pros::delay(10);
    }

    double position = motor.get_position();
    snprintf(extra, sizeof(extra), "waited:%5lu ms", static_cast<unsigned long>(pros::millis() - start));
    logWait(name, std::abs(position - target) > tolerance ? "TIMEOUT" : "ok", motor, position, target, extra);
}

constexpr int WRIST_TIMEOUT_MS = 3000;
constexpr double WRIST_TOLERANCE = 10;   // the wrist settles 2 to 6 units short of 1500, so 5 timed out

// Waits for the wrist to reach `target` (within +-10 units unless `tolerance` says otherwise), for at most 3 s.
void waitForWrist(double target, double tolerance = WRIST_TOLERANCE) {
    waitForPosition("wrist", clamp_wrist, target, WRIST_TIMEOUT_MS, tolerance);
}

// Runs the clamp rollers and moves back and forth twice (0 to -3 inches and back) at high speed.
// The rollers are left running; moveToDropPin turns them off.
void runToggler() {

    chassis.moveToPoint(0, -3, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, 0, 1500, {.forwards = true, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, -3, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, 0, 1500, {.forwards = true, .maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();                           // the last move is async: switch the brake after it ends
    logArrival("runToggler", 0, 0, NO_TARGET);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);    // hold the wheels for the precise moves that follow
}

// Backs up to (0, -9), then drives backwards to (-21, -16) facing 70 degrees, and waits there.
// The rollers stop and the wrist moves to 1100 while it is still backing up.
void moveToDropPin() {
    //chassis.moveToPoint(0, -10, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 10});
    chassis.moveToPose(-19, -13, 80, 4000, {.forwards = false, .maxSpeed = 70, .minSpeed = 10});   // 22 in: about 2 s expected
    clamp_rollers.move(0);
    clamp_wrist.move_absolute(1050, 100);
    waitForWrist(1050);

    chassis.waitUntilDone();
    logArrival("moveToDropPin", -19,-13, 80);
}

// At the drop point: drives backwards at -60 (no drive curve) for 0.7 s in total. After 0.2 s it moves the wrist
// to 1100 and runs the rollers in reverse (-50) for 0.5 s to release the preload Pin, then stops both.
void dropPin() {
    chassis.tank(-60, -60, true);    
    pros::delay(800);
    chassis.tank(0, 0, true);

    clamp_rollers.move(-25);
    pros::delay(500);

    clamp_rollers.move(0);
    clamp_wrist.move_absolute(1500, 100);
    waitForWrist(1500);
    logArrival("dropPin push", NO_TARGET, NO_TARGET, NO_TARGET);   // open loop: this is how far it actually went  

}

// Drives forward to (-5, -7) facing 90 degrees, moves the wrist to 1500, then drives backwards
// to (-24, -30) facing 35 degrees. Blocks until the motion ends (normal exit conditions) or 3 s pass.
void moveToPinAndCup1() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    //chassis.moveToPose(chassis.getPose().x+7, chassis.getPose().y+7, chassis.getPose().theta, 4000, {.maxSpeed = 90, .minSpeed = 10});   // 18 in: about 2 s expected
    chassis.tank(60,60,true);
    pros::delay(500);
    chassis.tank(0, 0, true);
    
    clamp_wrist.move_absolute(1500, 100);
    waitForWrist(1500);

    chassis.moveToPose(-23, -30, 40, 3000, {.forwards = false, .maxSpeed = 60}, false);
    logArrival("moveToPinAndCup1", -25, -32, 45);
}

// Runs the rollers forward (100) for 0.5 s, then moves the wrist to 280 and waits 0.5 s.
void grabPinAndCup1() {
    clamp_rollers.move(110);
    pros::delay(500);
    clamp_wrist.move_absolute(470, 100);
    waitForWrist(470, 100);                           // it rests on the Pin and Cup at about 317, so 5 never arrives
    pros::delay(500);
}

// Turns to 15 degrees, moves the wrist to 900, and raises the cascade by 1300 (to an absolute target,
// so the later drop can rely on it). Then turns to 160 degrees, backs up to (-23.4, -20), and waits
// for both the drive and the cascade to finish before returning.
// The 160 degree turn is capped at 2 s so a turn that never settles can't stall autonomous.
void moveToDropPinAndCup1() {

    clamp_wrist.move_absolute(1050, 95);
    waitForWrist(1050);
    cascade.move_absolute(900, 100);
    waitForPosition("cascade", cascade, 900, 1500);
    //pros::delay(500);
   //chassis.turnToHeading(1, 2000, {.maxSpeed = 80, .minSpeed = 10});
    chassis.swingToHeading(130,lemlib::DriveSide::RIGHT,2000,{.direction = lemlib::AngularDirection::CW_CLOCKWISE,.maxSpeed = 80, .minSpeed = 10},false  );
    //chassis.moveToPose(-25, -10,150,  2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 10});
    chassis.waitUntilDone();                           // dropPinAndCup1 must not start while still driving
    
    chassis.tank(-60, -60, true);
    pros::delay(500);
    chassis.tank(0, 0, true);

    logArrival("moveToDropPinAndCup1", -25, -10, NO_TARGET);
   // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

// Lowers the cascade to 500 (the 1200 moveToDropPinAndCup1 left it at, minus 700) while running the rollers in
// reverse (-25), waits for it to get there, then raises it to 800 and moves the wrist to 1400 (the 1100 it was
// left at, plus 300). Stops the drivetrain.
// All targets are absolute: they do not depend on where the mechanism happens to be when the step starts, so a
// wait in moveToDropPinAndCup1 that timed out does not shift them.
void dropPinAndCup1() {
    cascade.move_absolute(500, 100);                 // 1200 - 700
    waitForPosition("cascade", cascade, 500, 1500);

    clamp_rollers.move(-30);
    pros::delay(500);
    clamp_wrist.move_absolute(1500, 100);
    waitForWrist(1500, 500);

    cascade.move_absolute(800, 100);                 // 500 + 300
    waitForPosition("cascade", cascade, 800, 1500);
}

// Drives forward to (-20.4, -39) facing 132.9 (blocks), sends the cascade to 0 and the wrist to 1500, then backs
// up to (-44, -20) facing 130 and waits. Runs the rollers forward and lowers the wrist to 100 to collect
// the second Pin and Cup, raises it to 250 (waits for it), and turns to 140 degrees.
void getPinAndCup2() {
    chassis.tank(70,70,true);
    pros::delay(500);
    chassis.tank(0,0,true);

    cascade.move_absolute(0, 100);
    waitForPosition("cascade", cascade, 0, 2500);



    clamp_wrist.move_absolute(1500, 100);
    waitForWrist(1500, 560);

    logArrival("getPinAndCup2 forward", -20.4, -39.0, 132.9);
    chassis.moveToPose(-45, -19, 130, 3000, {.forwards = false, .maxSpeed = 50},false); // move to Pin + cup

    //pickup
    logArrival("getPinAndCup2 back", -44, -20, 130);
    clamp_rollers.move(100);
    pros::delay(800);

 
    clamp_wrist.move_absolute(470, 100);
    waitForWrist(470, 560);

    pros::delay(800);


    clamp_wrist.move_absolute(1100, 100);
    waitForWrist(1100, 500);
 
}

// Turns to 242 degrees, raises the cascade to 2000 (waits for it), drives backwards at -60 (open loop, about 1.5 s
// in total) while lowering the cascade to 1700 and running the rollers in reverse (-25) to release the Pin and Cup,
// raises the cascade to 2000 again and the wrist to 1350, then drives to (-44, -20) facing 130 and lowers the cascade to 0.
// The open-loop drive is not seen by odometry accurately, so the pose is off for the move that follows.
// The remaining fixed delays are action durations (how long the drive and the rollers run), so they stay.
void dropPinAndCup2() {

    cascade.move_absolute(1700, 127);
    waitForPosition("cascade", cascade, 1700, 2500);

    chassis.turnToHeading(242, 3000,{.maxSpeed=50,.minSpeed=0});                // about 100 degrees: about 1 s expected

    chassis.waitUntilDone();

    logArrival("dropPinAndCup2 turn", NO_TARGET, NO_TARGET, 242);


    clamp_wrist.move_absolute(1050, 90);
    waitForWrist(1050);

    chassis.tank(-60, -60, true);
    pros::delay(500);
    chassis.tank(0, 0, true);

    cascade.move_absolute(1400, 127);
    waitForPosition("cascade", cascade, 1400, 1500);


    clamp_rollers.move(-30);
    pros::delay(500);


    cascade.move_absolute(1600, 127);
    waitForPosition("cascade", cascade, 1600, 1500);
    
}

// Drives forwards to (-3.6, -46.3), intended to end the run with the Robot in the Midfield (8 points, <RSC3b>).
// Whether this point is inside the Midfield has not been checked against the Field.
// 48 in at a max of 90: about 2 s expected, 4 s limit.
// This is a queued background motion: it waits for the move before it to end, then autonomous() returns
// while it is still driving.
void park(){
    
    clamp_wrist.move_absolute(1500, 90);
    waitForWrist(1500, 1500);
    pros::delay(900);

    chassis.moveToPoint(-3.6, -46.3, 4000, {.forwards = true, .maxSpeed = 90});

    cascade.move_absolute(0, 100);
    waitForPosition("cascade", cascade, 0, 2000);

}

void autonomous() {
    autonStart = pros::millis();
    lastStepStart = autonStart;
    logStep("setupAutonomous");
    setupAutonomous();
    logStep("runToggler");
    runToggler();
    logStep("moveToDropPin");
    moveToDropPin();
    logStep("dropPin");
    dropPin();
    logStep("moveToPinAndCup1");
    moveToPinAndCup1();
    logStep("grabPinAndCup1");
    grabPinAndCup1();
    logStep("moveToDropPinAndCup1");
    moveToDropPinAndCup1();
    logStep("dropPinAndCup1");
    dropPinAndCup1();

    logStep("getPinAndCup2");
    getPinAndCup2();

    logStep("dropPinAndCup2");
    dropPinAndCup2();

    // // park() waits for the move before it to end, so this line prints when the park drive actually starts
    logStep("park");
    park();
    // logStep("park started (still driving)");
}

// ==========================================
// Control Logic Functions
// ==========================================

void controlDrivetrain() {
    if (isArcadeMode) {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
    } else {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);
    }
}

void controlIntake() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) intake.move(127); 
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) intake.move(-127); 
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) intake.brake(); 
}

// Cascade travel limit, in degrees from the bottom. Up is positive, as in autonomous. It keeps the Robot under
// the 50" height limit <SG3>. 2800 is carried over from the old code and is unverified: measure the
// Robot's height at this count and lower it if needed.
constexpr double CASCADE_TOP_LIMIT = 2800.0;

void controlCascadeSpool() {
    double position_deg = cascade.get_position();
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
          if (bumper.get_value() == 1) {
            cascade.brake();
            cascade.tare_position_all();
          } else cascade.move(-127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (position_deg >= CASCADE_TOP_LIMIT) cascade.brake();
        else cascade.move(127); 
    } else {
        cascade.brake();
    } 
}

void controlClampWrist() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) clamp_wrist.move(100); 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) clamp_wrist.move(-100); 
    else clamp_wrist.brake();
}

void controlClampRollers() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) clamp_rollers.move(127); 
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) clamp_rollers.move(-127); 
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) clamp_rollers.brake(); 
}

void opcontrol() {
    //autonomous();
    while (true) {
        if (!isTuning) {
            controlDrivetrain();
            controlIntake();
            controlCascadeSpool();
            controlClampWrist();
            controlClampRollers();
        } else {
            intake.brake();
            cascade.brake();
            clamp_wrist.brake();
            clamp_rollers.brake();
        }
        pros::delay(20); 
    }
}
