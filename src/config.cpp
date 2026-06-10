#include "main.h"

// Motor groups 
pros::MotorGroup leftMotors({4, -5, -6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-1, 2, 3}, pros::MotorGearset::blue);

// IMU: Inertial Measurement Unit
pros::Imu imu(8);

// Set the encoders 
pros::Rotation verticalEncoder(-18);
// pros::Rotation horizontalEncoder(12);

// TrackingWheel(encoder, wheelDiameter, offset, gearRatio = 1)
odyssey::TrackingWheel verticalWheel(&verticalEncoder, odyssey::Omniwheel::NEW_2, 0.0);
// odyssey::TrackingWheel horizontalWheel(&horizontalEncoder, odyssey::Omniwheel::NEW_2, -2.5);

// Define the drivetrain
odyssey::Drivetrain drivetrain{
    &leftMotors,
    &rightMotors,
    11.5,                        // track width, inches
    odyssey::Omniwheel::NEW_325, // drive wheel diameter, inches
    450,                         // drive rpm after external gearing
    2                            // horizontal drift
};

// PID constants for the drivetrain

odyssey::ControllerSettings lateralSettings{
    10,  // kP
    0,   // kI
    3,   // kD
    3,   // anti-windup range, inches
    1,   // small error range, inches
    100, // small error timeout, ms
    3,   // large error range, inches
    500, // large error timeout, ms
    20   // slew: max output change per 10 ms
};

odyssey::ControllerSettings angularSettings{
    2,   // kP
    0,   // kI
    10,  // kD
    3,   // anti-windup range, degrees
    1,   // small error range, degrees
    100, // small error timeout, ms
    3,   // large error range, degrees
    500, // large error timeout, ms
    0    // slew (usually 0 for turns)
};


// Odometry Sensors: tracking wheels and IMU
odyssey::OdomSensors sensors{
    &verticalWheel,   // vertical tracking wheel 1
    nullptr,          // vertical tracking wheel 2
    nullptr,          // horizontal tracking wheel 1
    nullptr,          // horizontal tracking wheel 2
    &imu              // inertial sensor
};

// Chassis controller

// optional driver-control input shaping
odyssey::ExpoDriveCurve throttleCurve(3, 10, 1.019);
odyssey::ExpoDriveCurve steerCurve(3, 10, 1.019);

odyssey::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors,
                         &throttleCurve, &steerCurve);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);


// old configs
pros::adi::Pneumatics scraper('E', false);
pros::adi::Pneumatics wings('H', false);
pros::adi::Pneumatics flap('F', true);
pros::adi::Pneumatics intake_piston('D', true);

pros::Motor intake_lower(9, pros::MotorGearset::blue);
pros::Motor intake_upper(-10, pros::MotorGearset::blue);

