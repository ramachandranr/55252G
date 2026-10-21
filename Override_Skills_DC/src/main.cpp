#include "main.h"
#include "lemlib/api.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>

extern void runTwiddleTuner();

// ==========================================
// Thread-Safe Global Variables
// ==========================================
char tuner_status[32] = "Waiting...";
int tuner_iteration = 0;
int tuner_max_iterations = 10;
double tuner_kP = 0, tuner_kI = 0, tuner_kD = 0;
double tuner_best_cost = 0;

// Array to safely pass diagnostic text to the screen task
char diag_text[6][64] = {"", "", "", "", "", ""};

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
pros::Rotation horizontal(8); 
lemlib::TrackingWheel horizontal_sensor(&horizontal, 1.0, false); 

// ==========================================
// LemLib Configuration
// ==========================================

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.6, lemlib::Omniwheel::NEW_275, 450, 2);
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2.3, 0, 19, 3, 1, 100, 3, 500, 0);
lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal_sensor, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

bool isArcadeMode = true;
bool isTuning = false; 

void toggleDriveMode() {
    isArcadeMode = !isArcadeMode;
}

// ==========================================
// Safe Task Wrappers 
// ==========================================

void tuningTaskWrapper(void* param) {
    runTwiddleTuner();
    pros::delay(10000); 
    isTuning = false; 
}

void startTunerBtn() {
    if (isTuning) return; 
    isTuning = true;
    
    // SAFE TASK SPAWNING: Using the native C API prevents memory aborts
    pros::c::task_create(tuningTaskWrapper, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tuner Task");
}

struct MotorData { double temp; int current; int voltage; bool has_issue; };
struct GroupStat { std::string name; bool has_issue; std::vector<MotorData> motors; };

void diagnosticTaskWrapper(void* param) {
    while (true) {
        if (isTuning) {
            pros::delay(500); 
            continue;
        }

        std::vector<GroupStat> stats;
        auto checkGroup = [&](pros::MotorGroup& group, const std::string& baseName) {
            auto temps = group.get_temperature_all();
            auto currents = group.get_current_draw_all();
            GroupStat groupStat;
            groupStat.name = baseName;
            groupStat.has_issue = false;

            for (size_t i = 0; i < temps.size(); i++) {
                bool issue = (temps[i] >= 55.0) || (currents[i] > 1200);
                if (issue) groupStat.has_issue = true;
                groupStat.motors.push_back({temps[i], currents[i], 0, issue}); 
            }
            stats.push_back(groupStat);
        };

        auto checkMotor = [&](pros::Motor& motor, const std::string& name) {
            double temp = motor.get_temperature();
            int current = motor.get_current_draw();
            bool issue = (temp >= 55.0) || (current > 1200);
            GroupStat groupStat;
            groupStat.name = name;
            groupStat.has_issue = issue;
            groupStat.motors.push_back({temp, current, 0, issue});
            stats.push_back(groupStat);
        };

        checkGroup(left_motors, "L_Drv"); 
        checkGroup(right_motors, "R_Drv");
        checkGroup(cascade, "Cascd");
        checkMotor(intake, "Intak");
        checkMotor(clamp_rollers, "ClmpR");
        checkMotor(clamp_wrist, "ClmpW");

        std::stable_sort(stats.begin(), stats.end(), [](const GroupStat& a, const GroupStat& b) {
            return a.has_issue > b.has_issue; 
        });

        for (int i = 0; i < 6; i++) {
            if (i < stats.size()) {
                const auto& g = stats[i];
                std::string line = g.has_issue ? "[!] " : "[OK] ";
                char nameBuf[10];
                snprintf(nameBuf, sizeof(nameBuf), "%-5s", g.name.c_str());
                line += nameBuf;

                for (size_t m_idx = 0; m_idx < g.motors.size(); m_idx++) {
                    const auto& m = g.motors[m_idx];
                    char m_buf[25];
                    snprintf(m_buf, sizeof(m_buf), "| %2.0fC %4dmA ", m.temp, m.current);
                    line += m_buf;
                }
                snprintf(diag_text[i], sizeof(diag_text[i]), "%-40s", line.c_str());
            } else {
                snprintf(diag_text[i], sizeof(diag_text[i]), " "); 
            }
        }
        pros::delay(2000);
    }
}

void screenTaskWrapper(void* param) {
    bool wasTuning = false;
    
    while (true) {
        if (isTuning != wasTuning) {
            pros::lcd::clear();
            wasTuning = isTuning;
        }
        
        if (!isTuning) {
            // FIX: Removed dangerous cascade motor object from variadic print
            pros::lcd::print(0, "[%s] X:%.1f Y:%.1f T:%.1f", 
                             isArcadeMode ? "ARC" : "TNK",
                             chassis.getPose().x, 
                             chassis.getPose().y, 
                             chassis.getPose().theta); 
            
            for (int i = 0; i < 6; i++) {
                pros::lcd::print(i + 1, "%s", diag_text[i]);
            }
        } else {
            pros::lcd::print(0, "--- TWIDDLE TUNER ---");
            pros::lcd::print(1, "Status: %s", tuner_status);
            pros::lcd::print(2, "Iter: %d / %d", tuner_iteration, tuner_max_iterations);
            pros::lcd::print(3, "P: %.4f", tuner_kP);
            pros::lcd::print(4, "I: %.4f", tuner_kI);
            pros::lcd::print(5, "D: %.4f", tuner_kD);
            pros::lcd::print(6, "Cost: %.0f", tuner_best_cost);
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
    
    pros::lcd::register_btn0_cb(toggleDriveMode); 
    pros::lcd::register_btn2_cb(startTunerBtn);   
    pros::lcd::set_text(7, "[Drive Mode]               [Tune PID]"); 
    
    calibrateChassis(); 

    // SAFE TASK SPAWNING: C API avoids all object-destruction memory aborts
    pros::c::task_create(diagnosticTaskWrapper, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Diag Task");
    pros::c::task_create(screenTaskWrapper, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Screen Task");
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    clamp_wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp_rollers.move(100);
    clamp_wrist.tare_position();

    chassis.moveToPoint(0, -3, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, 0, 1500, {.forwards = true, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, -3, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.moveToPoint(0, 0, 1500, {.forwards = true, .maxSpeed = 100, .minSpeed = 80});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    chassis.moveToPoint(0, -9, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 60});
    clamp_rollers.move(0);
    clamp_wrist.move_absolute(1050, 100); 

    chassis.moveToPose(-21, -16,70, 10000, {.forwards = false, .maxSpeed = 70, .minSpeed = 60});
    chassis.waitUntilDone();
    clamp_wrist.move_absolute(1150, 100);
    clamp_rollers.move(-50); 

    pros::delay(500);
    clamp_rollers.move(0); 

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPose(-5, -7,90, 10000, {.maxSpeed = 90, .minSpeed = 80}); 
   
    clamp_wrist.move_absolute(1500, 100); 
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPose(-22.3, -29.5,32, 3000, {.forwards = false, .maxSpeed = 90,.minSpeed = 0, .earlyExitRange = 5}, false); 

    clamp_rollers.move(100);
    pros::delay(500);
    clamp_wrist.move_absolute(280, 100); 
    pros::delay(500);
    chassis.turnToHeading(15,1000);
    
    clamp_wrist.move_absolute(900, 95);
    cascade.move_relative(1300, -127);
    pros::delay(300);
    chassis.turnToHeading(160, 1000000, {.maxSpeed = 80, .minSpeed = 25});
    chassis.moveToPoint(-23.4, -20, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 4});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    cascade.move_relative(-500, 100);
    clamp_rollers.move(-25);
    pros::delay(200);
    cascade.move_relative(300, 100);
    clamp_wrist.move_relative(300, 100);
    chassis.tank(0, 0, true);
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
    double TOP_LIMIT = -2800.0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
          if (bumper.get_value() == 1) {
            cascade.brake(); 
            cascade.tare_position_all(); 
          } else cascade.move(-127);       
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (position_deg <= TOP_LIMIT) cascade.brake(); 
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