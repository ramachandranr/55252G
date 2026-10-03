#include "main.h"

#include "lemlib/api.hpp"

// ==========================================
// Hardware Configuration
// ==========================================

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-19, -20}, pros::MotorGearset::blue); 
pros::MotorGroup right_motors({11, 12}, pros::MotorGearset::blue);

pros::Motor intake(1, pros::MotorGearset::blue);
pros::MotorGroup cascade({-3, 9}, pros::MotorGearset::blue); 
pros::Motor clamp_rollers(2, pros::MotorGearset::green); 
pros::Motor clamp_wrist(10, pros::MotorGearset::green); 

pros::Imu imu(13);
	
// ==========================================
// LemLib Configuration
// ==========================================

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.6, lemlib::Omniwheel::NEW_275, 428, 2);
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

// ==========================================
// Setup Functions
// ==========================================

void calibrateChassis() {
    chassis.calibrate();
}

void initSubsystems() {
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp_rollers.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp_wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
    cascade.tare_position();
}

// ==========================================
// Control Logic Functions
// ==========================================

void controlDrivetrain() {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.tank(leftY, rightY);
}

// ==========================================
// Skills Driver System
// ==========================================
// 22 selectable skills. Each skill has a Start task (its set of tasks, run
// once when the skill becomes selected) and an End task (its ending task,
// run once right before the skill is switched away from). The controller's
// Left/Right buttons step back and forth through the list; on every switch,
// in either direction, the outgoing skill's End task runs first and only
// then does the incoming skill's Start task run.

void skill1() {
    clamp_wrist.move_absolute(1100, 100);
    cascade.move_absolute(0, 100);
    intake.move(0);
    clamp_rollers.move(0);


}
void skill2() {
    clamp_wrist.move_absolute(1100, 100);
    cascade.move_absolute(0, 100);
    intake.move(0);
    clamp_rollers.move(0);
    clamp_rollers.move(-100);
    pros::delay(500);
    clamp_wrist.move_absolute(1460, 100);
}
void skill3() {
    clamp_wrist.move_absolute(1200, 100);
    clamp_rollers.move(100);
}

void skill4() {
    clamp_wrist.move_absolute(700,-100);
}
void skill5() {}
void skill6() {}
void skill7() {}
void skill8() {}
void skill9() {}
void skill10() {}
void skill11() {}
void skill12() {}
void skill13() {}
void skill14() {}
void skill15() {}
void skill16() {}
void skill17() {}
void skill18() {}
void skill19() {}
void skill20() {}
void skill21() {}
void skill22() {}

struct Skill {
    void (*start)();
    const char* description;
};

const int NUM_SKILLS = 22;

// Controller line fits ~19 chars, so keep descriptions to about 15.
Skill skills[NUM_SKILLS] = {
    {skill1,   "Description"},
    {skill2,  "Description"},
    {skill3,  "Description"},
    {skill4,  "Description"},
    {skill5,  "Description"},
    {skill6,  "Description"},
    {skill7,  "Description"},
    {skill8,  "Description"},
    {skill9,  "Description"},
    {skill10, "Description"},
    {skill11, "Description"},
    {skill12, "Description"},
    {skill13, "Description"},
    {skill14, "Description"},
    {skill15, "Description"},
    {skill16, "Description"},
    {skill17, "Description"},
    {skill18, "Description"},
    {skill19, "Description"},
    {skill20, "Description"},
    {skill21, "Description"},
    {skill22, "Description"},
};

int currentSkillIndex = 0;

void displaySelectedSkill() {
    master.print(0, 0, "%d: %-15s", currentSkillIndex + 1, skills[currentSkillIndex].description);
}

void switchToSkill(int newIndex) {
    currentSkillIndex = newIndex;
    displaySelectedSkill();
    skills[currentSkillIndex].start();
}

void updateSkillSelection() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        switchToSkill((currentSkillIndex + 1) % NUM_SKILLS);
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        switchToSkill((currentSkillIndex - 1 + NUM_SKILLS) % NUM_SKILLS);
    }
}

// ==========================================
// Competition Control
// ==========================================

void initialize() {
    initSubsystems();
    calibrateChassis();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    // Separate task so tank drive keeps running even if a skill function blocks.
    pros::Task driveTask([] {
        while (true) {
            controlDrivetrain();
            pros::delay(10);
        }
    });

    // Start task for the initially selected skill (index 0); no End task
    // runs here since no skill was active yet.
    displaySelectedSkill();
    skills[currentSkillIndex].start();

    while (true) {
        updateSkillSelection();
        pros::delay(20);
    }

    // Casade measurement test

}

