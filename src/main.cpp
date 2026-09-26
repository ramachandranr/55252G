#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include <ios>


pros::MotorGroup left_motors({19,20}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({11,12},pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// create an imu on port 10
pros::Imu imu(13);

// create a v5 rotation sensor on port 1
pros::Rotation rotation_sensor(8);

pros::Rotation horizontal_sensor(8);


// imu
pros::Imu imu(13);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);
