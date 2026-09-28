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
// Initialize bumper switch on ADI port 'A'
pros::adi::DigitalIn bumper('A');
pros::Imu imu(13);

// ==========================================
// LemLib Configuration
// ==========================================

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.6, lemlib::Omniwheel::NEW_4, 428, 2);
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
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    clamp_rollers.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp_wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
    cascade.tare_position_all();
}

// ==========================================
// Control Logic Functions
// ==========================================

void controlDrivetrain() {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(leftY, rightX);
}

void controlIntake() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move(127); 
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.move(-127); 
    } else if  (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        intake.brake();
    }
}

void controlCascadeSpool() {
    double net_rotations = cascade.get_position() / 360.0;


    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
          if (bumper.get_value() == 1) {
            cascade.brake(); 
          } else {
            cascade.move(127); 
        }
        

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        // if (net_rotations <= 0.0) {
        //     cascade.brake(); 
        // } else {
        //     cascade.move(-127); 
        // }
        cascade.move(-127); 

    } else {
        cascade.brake(); 
    }
}

void controlClampWrist() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        clamp_wrist.move_velocity(100); 
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        clamp_wrist.move_velocity(-100); 
    } else {
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

void initialize() {
    pros::lcd::initialize();
    calibrateChassis(); 
	    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "cascade: %f", cascade.get_position()); // heading

			// log position telemetry
            // delay to save resources
            pros::delay(100);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Insert autonomous routine here
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