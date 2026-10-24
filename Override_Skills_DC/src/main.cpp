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
    clamp_wrist.tare_position();
    clamp_rollers.tare_position();
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
    //Move Wrist into scoring position for alliance goal
    clamp_wrist.move_absolute(1100, 200);

}
void skill2() {
    //Score Pre Load and then retract the wrist. This is the first scoring Pin
    clamp_wrist.move_absolute(1100, 200);
    clamp_rollers.move(-100);
    pros::delay(500);
    clamp_wrist.move_absolute(1460, 200);
}
void skill3() {
    //Lower wrist and run clamp rollers to intake. This is for the first pin + cup combo
    clamp_wrist.move_absolute(400, -200);
    clamp_rollers.move(100);
}

void skill4() {
    //Move wrist into scoring position and prepare cascade for scoring
    clamp_wrist.move_absolute(1150,200);
    clamp_rollers.move(100);
    cascade.move_absolute(-750, 400);
}
void skill5() {
    //Prepare cascade for scoring by bringing it down, score the first pin and cup and retract wrist
    cascade.move_absolute(-600, 400);
    pros::delay(500);
    clamp_rollers.move(-100);
    pros::delay(500);
    clamp_wrist.move_absolute(1400, 200);
}

void skill6() {
    //Move wrist into scoring position, run clamp rollers, and bring cascade down. This collects the second pin + cup
    clamp_wrist.move_absolute(1250,200);
    pros::delay(500);
    clamp_rollers.move(100);
    cascade.move_absolute(0, -400);
    pros::delay(500);
    clamp_wrist.move_absolute(400, 200);
}
void skill7() {
    //Move wrist into scoring position and bring cascade down
    clamp_wrist.move_absolute(1250, 200);
    pros::delay(500);
    cascade.move_absolute(-1650, 400);
}
void skill8() {
    //Score the second pin + cup
    cascade.move_absolute(-1450,400);
    clamp_rollers.move(-50);
    pros::delay(1000);
    clamp_rollers.move(0);
    cascade.move_absolute(-1650,400);
    clamp_wrist.move_absolute(1650, 400);
}
void skill9() {
    //Bring cascade to the starting position and run intake to take the pin
    cascade.move_absolute(0, -400);
    pros::delay(500);
    clamp_wrist.move_absolute(-1400, 200);
    clamp_rollers.move(127);
    intake.move(127);
}

void skill10() {
    // Bring Cascade up to score the pin
    intake.move(0);
    clamp_wrist.move_absolute(1250, 200);
    clamp_rollers.move(100);
    cascade.move_absolute(0,400);
}

void skill11() {
    //score the pin
    clamp_rollers.move(-50);

}
void skill12() {
    //Bring cascade to the starting position and run intake to take the next pin and a cup
    cascade.move_absolute(0, -400);
    pros::delay(500);
    clamp_wrist.move_absolute(-1300, 200);
    clamp_rollers.move(100);
    intake.move(200);
}

void skill13() {
    //Move wrist into scoring position and prepare cascade for scoring
    clamp_wrist.move_absolute(1250,200);
    clamp_rollers.move(100);
    cascade.move_absolute(-750, 400);
}

void skill14() {
    //Prepare cascade for scoring by bringing it down, score  and retract wrist
    cascade.move_absolute(-600, 400);
    pros::delay(500);
    clamp_rollers.move(-100);
    pros::delay(500);
    clamp_wrist.move_absolute(1400, 200);
}

void skill15() {
    //Bring cascade to the starting position and run intake to take the pin
    cascade.move_absolute(0, -400);
    pros::delay(500);
    clamp_wrist.move_absolute(-600, 200);
    clamp_rollers.move(100);
    intake.move(200);
}

void skill16() {
//Move wrist into scoring position and bring cascade down
    clamp_wrist.move_absolute(1250, 200);
    pros::delay(500);
    cascade.move_absolute(-1650, 400);
}
void skill17() {
    //score the pin and cup
    cascade.move_absolute(-1450,400);
    clamp_rollers.move(-50);
    pros::delay(1000);
    clamp_rollers.move(0);
    cascade.move_absolute(-1650,400);
    clamp_wrist.move_absolute(1650, 400);
}
void skill18() {
    //bring everything down. while the toggler happens
    cascade.move_absolute(0, -400);
    pros::delay(500);
    clamp_wrist.move_absolute(-1400, 200);
    clamp_rollers.move(0);
    intake.move(0);
}
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
    {skill1,   "Skill 1"},
    {skill2,  "Skill 2"},
    {skill3,  "Skill 3"},
    {skill4,  "Skill 4"},
    {skill5,  "Skill 5"},
    {skill6,  "Skill 6"},
    {skill7,  "Skill 7"},
    {skill8,  "Skill 8"},
    {skill9,  "Skill 9"},
    {skill10, "Skill 10"},
    {skill11, "Skill 11"},
    {skill12, "Skill 12"},
    {skill13, "Skill 13"},
    {skill14, "Skill 14"},
    {skill15, "Skill 15"},
    {skill16, "Skill 16"},
    {skill17, "Skill 17"},
    {skill18, "Skill 18"},
    {skill19, "Skill 19"},
    {skill20, "Skill 20"},
    {skill21, "Skill 21"},
    {skill22, "Skill 22"},
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
        switchToSkill((currentSkillIndex) % NUM_SKILLS);
        currentSkillIndex = (currentSkillIndex) +1;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        switchToSkill((currentSkillIndex - 1 + NUM_SKILLS) % NUM_SKILLS);
        currentSkillIndex = currentSkillIndex - 1;
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
    //skills[currentSkillIndex].start();

    while (true) {
        updateSkillSelection();
        pros::delay(20);
    }

    // Casade measurement test

}

