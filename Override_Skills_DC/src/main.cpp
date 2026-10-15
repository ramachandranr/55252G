#include "main.h"
#include "lemlib/api.hpp"
#include <vector>
#include <string>
#include <algorithm>

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

// Initialize bumper switch on ADI port 'A'
pros::adi::DigitalIn bumper('A');
pros::Imu imu(13);
pros::Rotation horizontal(8); // true = reversed
lemlib::TrackingWheel horizontal_sensor(&horizontal, 1.0, false); // port, diameter in inches, reversed

// ==========================================
// LemLib Configuration
// ==========================================

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.6, lemlib::Omniwheel::NEW_275, 450, 2);
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2.3, 0, 19, 3, 1, 100, 3, 500, 0);
lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal_sensor, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

bool isArcadeMode = true;

void toggleDriveMode() {
    isArcadeMode = !isArcadeMode;
}

// ==========================================
// Diagnostics Task
// ==========================================

struct MotorData {
    double temp;
    int current;
    int voltage;
    bool has_issue;
};

struct GroupStat {
    std::string name;
    bool has_issue; // True if ANY motor in this group has an issue
    std::vector<MotorData> motors;
};

void startDiagnosticTask() {
    pros::Task diagTask([]() {
        while (true) {
            std::vector<GroupStat> stats;

            // Helper to gather stats for a MotorGroup
            auto checkGroup = [&](pros::MotorGroup& group, const std::string& baseName) {
                auto temps = group.get_temperature_all();
                auto currents = group.get_current_draw_all();
                
                GroupStat groupStat;
                groupStat.name = baseName;
                groupStat.has_issue = false;

                for (size_t i = 0; i < temps.size(); i++) {
                    // Trigger an issue if Temp is 55C+ or Current is high (1200mA+)
                    bool issue = (temps[i] >= 55.0) || (currents[i] > 1200);
                    if (issue) groupStat.has_issue = true;
                    
                    // Voltage omitted to save screen space on the V5 Brain
                    groupStat.motors.push_back({temps[i], currents[i], 0, issue}); 
                }
                stats.push_back(groupStat);
            };

            // Helper to gather stats for a single Motor (treats it as a group of 1)
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

            // 1. Collect Data
            checkGroup(left_motors, "L_Drv"); 
            checkGroup(right_motors, "R_Drv");
            checkGroup(cascade, "Cascd");
            checkMotor(intake, "Intak");
            checkMotor(clamp_rollers, "ClmpR");
            checkMotor(clamp_wrist, "ClmpW");

            // 2. Sort Data (Groups with 'has_issue == true' go to the top)
            std::stable_sort(stats.begin(), stats.end(), [](const GroupStat& a, const GroupStat& b) {
                return a.has_issue > b.has_issue; 
            });

            // 3. Print Data to LCD Lines 1 through 6
            for (size_t i = 0; i < stats.size(); i++) {
                const auto& g = stats[i];
                
                // Add the warning or OK prefix
                std::string line = g.has_issue ? "[!] " : "[OK] ";
                
                // Add the group name padded to 5 characters
                char nameBuf[10];
                snprintf(nameBuf, sizeof(nameBuf), "%-5s", g.name.c_str());
                line += nameBuf;

                // Loop through every motor in this specific group and append it to the same line
                for (size_t m_idx = 0; m_idx < g.motors.size(); m_idx++) {
                    const auto& m = g.motors[m_idx];
                    char m_buf[25];
                    // %2.0f rounds temp to a whole number to save space
                    snprintf(m_buf, sizeof(m_buf), "| %2.0fC %4dmA ", m.temp, m.current);
                    line += m_buf;
                }
                
                // Print to the physical screen, starting at line 1 (leaves line 0 for odometry)
                pros::lcd::print(i + 1, "%s", line.c_str());
            }
            
            // Run every 2 seconds
            pros::delay(2000);
        }
    });
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
    pros::lcd::set_text(7, "[Toggle Drive]");
    calibrateChassis(); 
    
    startDiagnosticTask(); // Launch the background diagnostics

    // Separate task for brain screen Line 0 so it updates faster than 2 seconds
    pros::Task screenTask([&]() {
        while (true) {
            // All odometry and cascade data condensed onto Line 0
            pros::lcd::print(0, "[%s] X:%.1f Y:%.1f T:%.1f C:%.0f", 
                             isArcadeMode ? "ARC" : "TNK",
                             chassis.getPose().x, 
                             chassis.getPose().y, 
                             chassis.getPose().theta, 
                             cascade.get_position()); 
            
            // delay to save resources
            pros::delay(200);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    clamp_wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //Start clamp rollers to suck the pin in
    clamp_rollers.move(100);
    clamp_wrist.tare_position();

    //Toggler routine
    chassis.moveToPoint(0, -3, 1000000, {.forwards = false, .maxSpeed = 90, .minSpeed = 80});
    pros::delay(300);
    chassis.moveToPoint(0, 0, 1000000, {.forwards = true, .maxSpeed = 127, .minSpeed = 80});
    chassis.moveToPoint(0, -3, 1000000, {.forwards = false, .maxSpeed = 90, .minSpeed = 80});
    pros::delay(300);
    chassis.moveToPoint(0, 0, 1000000, {.forwards = true, .maxSpeed = 127, .minSpeed = 80});
    pros::delay(300);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    //Move to score the preload
    chassis.moveToPoint(0, -9, 1000000, {.forwards = false, .maxSpeed = 80, .minSpeed = 70});
    pros::delay(500);
    clamp_rollers.move(0);
    clamp_wrist.move_absolute(1050, 100);
    pros::delay(500);
    chassis.moveToPose(-13.802, -16.080,70, 1000000, {.forwards = false, .maxSpeed = 70, .minSpeed = 60});
    pros::delay(500);
    chassis.cancelAllMotions();                  // make sure no async motion is fighting it
    chassis.tank(-60, -60, true);                  // push forward; true = skip the drive curve
    clamp_wrist.move_absolute(1100, 100);
    pros::delay(500);
    clamp_rollers.move(-50);
    pros::delay(500);
    chassis.tank(0, 0, true);  
    pros::delay(500);
    clamp_rollers.move(0);

   //Move to collect the first pin+cup
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(-7, -7, 1000000, {.maxSpeed = 90, .minSpeed = 80});
    pros::delay(500);
    clamp_wrist.move_absolute(1500, 100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPose(-23.248, -30.744,40, 3000, {.forwards = false, .maxSpeed = 90,.minSpeed = 40, .earlyExitRange = 5}, true);

   //Collect pin + cup
    pros::delay(1000);
    clamp_rollers.move(100);
    pros::delay(500);
    clamp_wrist.move_absolute(680, 100);
    pros::delay(500);
    chassis.turnToHeading(15,1000);
    
    //collected pin + cup, now turn towards scoring
    pros::delay(300);
    clamp_wrist.move_absolute(1100, 95);
    cascade.move_relative(1500, -127);
    clamp_rollers.move(0);
    pros::delay(300);
    chassis.turnToHeading(180, 1000000, {.maxSpeed = 100, .minSpeed = 85});
    chassis.moveToPose(-24.248, -34.744,165,  1000000, {.forwards = false, .maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 4});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(300);

    //Score Pin + Cup
    chassis.cancelAllMotions();                  // make sure no async motion is fighting it
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.tank(-80, -60, true);   
    cascade.move_relative(-600, 100);
    pros::delay(1200);
    clamp_rollers.move(-50);
    pros::delay(500);
    cascade.move_relative(300, 100);
    pros::delay(500);
    clamp_wrist.move_relative(300, 100);
    pros::delay(500);
    chassis.tank(0, 0, true);

    chassis.moveToPose(-24.248, -34.744,135,  1000000, {.forwards = false, .maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 4});
}

// ==========================================
// Control Logic Functions
// ==========================================

void controlDrivetrain() {

    if (isArcadeMode) {
        // ARCADE
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
    } else {
        // TANK
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);
    }
}

void controlIntake() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move(127); 
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.move(-127); 
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        intake.brake(); // Fix: Ensures the intake actually stops when released
    }
}

void controlCascadeSpool() {
    double position_deg = cascade.get_position();
    double TOP_LIMIT = -2800.0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
          if (bumper.get_value() == 1) {
            cascade.brake(); 
            cascade.tare_position_all(); // Prevents encoder drift
          } else {
            cascade.move(-127); 
        }       
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (position_deg <= TOP_LIMIT) {
            cascade.brake(); 
        }else{ 
            cascade.move(127); 
        }
        
    }else{
        cascade.brake();
    } 
}

void controlClampWrist() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        clamp_wrist.move(100); 
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        clamp_wrist.move(-100); 
    } else  {
        clamp_wrist.brake();
    }
}

void controlClampRollers() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        clamp_rollers.move(127); 
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        clamp_rollers.move(-127); 
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        clamp_rollers.brake(); 
    }
}

void opcontrol() {
    // Set brake modes and tare the spool
    initSubsystems();

    while (true) {
        controlDrivetrain();
        controlIntake();
        controlCascadeSpool();
        controlClampWrist();
        controlClampRollers();

        pros::delay(20); 
    }
}