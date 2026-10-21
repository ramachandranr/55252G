#include "main.h"
#include "lemlib/api.hpp"
#include "pid_tuner.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>

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
// TODO: CHECK the horizontal odometry setup on the robot before trusting any moveToPose.
//   - wheel diameter: NEW_2 is 2.125" (LemLib's 2" omni); confirm it matches the real wheel
//   - offset: 0 assumes the wheel is on the line through the robot's centre of rotation; measure it
//     (front of centre is positive, behind is negative)
//   - direction: if x reads backwards when the robot is pushed sideways, reverse the sensor above
//     (pros::Rotation horizontal(8, true))
// A quick check: push the robot 24" sideways by hand and compare the x reading on the brain screen.
pros::Rotation horizontal(8);
lemlib::TrackingWheel horizontal_sensor(&horizontal, lemlib::Omniwheel::NEW_2, 0);   // sensor, wheel diameter, offset from the tracking centre (in)

// ==========================================
// LemLib Configuration
// ==========================================

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.6, lemlib::Omniwheel::NEW_275, 450, 2);
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2.3, 0, 19, 3, 1, 100, 3, 500, 0);
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
        if (tunerRequested.exchange(false)) startTuner();

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
// Coordinates are inches relative to the robot's start (the pose is zeroed at the beginning).
// Heading 0 faces the start direction (+Y) and increases clockwise. `forwards = false` drives backwards.
// Wrist positions are degrees from the wrist's tare point (taken at the start of autonomous).
//
// LemLib motions run in the background by default (async). A new motion waits for the previous
// one to finish, while plain motor commands written right after a motion run while it is moving.
// The order of the statements below is therefore part of the routine; do not reorder them.
//
// Routine:
//   setupAutonomous  ->  runToggler  ->  moveToDropPin  ->  dropPin
//   ->  moveToPinAndCup1  ->  grabPinAndCup1  ->  moveToDropPinAndCup1  ->  dropPinAndCup1
//   ->  getPinAndCup2  ->  dropPinAndCup2   (the last two are not written yet)

// Waits until `motor` is within `tolerance` of `target`, or `timeoutMs` passes.
// move_absolute and move_relative return immediately, and a move_relative issued before the last
// one finishes starts from wherever the motor currently is. Wait here before the next move.
void waitForPosition(const pros::AbstractMotor& motor, double target, int timeoutMs, double tolerance = 10) {
    uint32_t start = pros::millis();
    while (std::abs(motor.get_position() - target) > tolerance && pros::millis() - start < static_cast<uint32_t>(timeoutMs)) {
        pros::delay(10);
    }
}

// Drives the cascade down onto the bumper and zeroes it there, so the absolute cascade targets below
// mean the same thing every run. If the bumper is never reached (1.5 s), the cascade is left un-zeroed.
void zeroCascade() {
    cascade.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    uint32_t start = pros::millis();
    while (bumper.get_value() != 1 && pros::millis() - start < 1500) {
        cascade.move(-127);
        pros::delay(10);
    }
    cascade.brake();
    if (bumper.get_value() == 1) cascade.tare_position_all();
}

// Zeroes the pose, the cascade and the wrist, and sets the brake modes before anything moves.
void setupAutonomous() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);   // coast while the quick moves in runToggler start
    clamp_wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp_wrist.tare_position();                       // wrist positions below count from here
    zeroCascade();                                     // cascade positions below count from the bumper
}

// Runs the clamp rollers and moves back and forth twice (0 to -3 inches and back) at high speed.
// The rollers are left running; moveToDropPin turns them off.
void runToggler() {
    clamp_rollers.move(100);

    chassis.moveToPoint(0, -3, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, 0, 1500, {.forwards = true, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, -3, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, 0, 1500, {.forwards = true, .maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();                           // the last move is async: switch the brake after it ends
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);    // hold the wheels for the precise moves that follow
}

// Backs up to (0, -9), then drives backwards to (-21, -16) facing 70 degrees, and waits there.
// The rollers stop and the wrist moves to 1100 (horizontal) while it is still backing up.
void moveToDropPin() {
    chassis.moveToPoint(0, -9, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 60});
    clamp_rollers.move(0);
    clamp_wrist.move_absolute(1100, 100);

    chassis.moveToPose(-21, -16, 70, 10000, {.forwards = false, .maxSpeed = 70, .minSpeed = 60});
    chassis.waitUntilDone();
}

// At the drop point: keeps driving backwards at -60 (power, no drive curve) for 0.2 s, then runs the rollers
// in reverse (-50) for another 0.2 s while still driving. Then stops the drive and the rollers.
void dropPin() {
    chassis.tank(-60, -60, true);
    pros::delay(200);
    clamp_wrist.move_absolute(1100, 100);
    clamp_rollers.move(-50);
    pros::delay(200);
    chassis.tank(0, 0, true);
    clamp_rollers.move(0);
}

// Drives forward to (-5, -7) facing 90 degrees, moves the wrist to 1500, then drives backwards
// to (-22.3, -29.5) facing 38 degrees. Blocks until the robot is within 5 inches of that point
// (earlyExitRange) or 3 s pass.
void moveToPinAndCup1() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPose(-5, -7, 90, 10000, {.maxSpeed = 90, .minSpeed = 80});

    clamp_wrist.move_absolute(1500, 100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPose(-22.3, -29.5, 38, 3000, {.forwards = false, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 5}, false);
}

// Collects the pin and cup. Runs the rollers forward (100) and lowers the wrist 1400 below where it is,
// waiting until it gets there (3 s cap), then raises it 150. Both moves use absolute targets: a second
// move_relative issued early would start from wherever the wrist had reached. Then turns to 15 degrees,
// waits 0.5 s, and moves the wrist to 1100 to hold the pin and cup.
void grabPinAndCup1() {
    clamp_rollers.move(100);
    double wristDown = clamp_wrist.get_position() - 1400;
    clamp_wrist.move_absolute(wristDown, 100);
    waitForPosition(clamp_wrist, wristDown, 3000);
    clamp_wrist.move_absolute(wristDown + 150, 100);
    pros::delay(125);
    chassis.turnToHeading(15, 1000);
    pros::delay(500);
    clamp_wrist.move_absolute(1100, 100);
}

// Turns toward the scoring spot: wrist to 1100, cascade to 900 (absolute, speed 300), then turns to 160
// degrees and backs up to (-23.4, -20). Waits for the drive to finish before returning.
// The 160 degree turn is capped at 2 s so a turn that never settles can't stall autonomous.
void moveToDropPinAndCup1() {
    clamp_wrist.move_absolute(1100, 95);
    cascade.move_absolute(900, 300);
    pros::delay(300);
    chassis.turnToHeading(160, 2000, {.maxSpeed = 80, .minSpeed = 25});
    chassis.moveToPoint(-23.4, -20, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 4});
    chassis.waitUntilDone();                           // dropPinAndCup1 must not start while still driving
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

// Scores the pin and cup. Drives backwards at -70 / -60 while the cascade goes to 600 (absolute) and the
// rollers run in reverse (-25); 0.2 s later the cascade returns to 900, and 0.5 s after that the wrist
// moves by +300. Then stops the drive.
void dropPinAndCup1() {
    chassis.tank(-70, -60, true);
    cascade.move_absolute(600, 300);
    clamp_rollers.move(-25);
    pros::delay(200);
    cascade.move_absolute(900, 300);
    pros::delay(500);
    clamp_wrist.move_relative(300, 100);
    chassis.tank(0, 0, true);
}

// TODO: not written yet. Drive to the second pin and cup and pick them up.
void getPinAndCup2() {}

// TODO: not written yet. Drop the second pin and cup.
void dropPinAndCup2() {}

void autonomous() {
    setupAutonomous();
    runToggler();
    moveToDropPin();
    dropPin();
    moveToPinAndCup1();
    grabPinAndCup1();
    moveToDropPinAndCup1();
    dropPinAndCup1();
    getPinAndCup2();
    dropPinAndCup2();
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

void controlCascadeSpool() {
    double position_deg = cascade.get_position();
    // Up is positive: Down drives into the bumper, which is zero. A negative limit could never be reached.
    double TOP_LIMIT = 2800.0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
          if (bumper.get_value() == 1) {
            cascade.brake();
            cascade.tare_position_all();
          } else cascade.move(-127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (position_deg >= TOP_LIMIT) cascade.brake();
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
    initSubsystems();

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
