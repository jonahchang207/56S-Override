#pragma once

#include "main.h"

// Motor groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// Sensors
extern pros::Imu imu;
extern pros::Rotation verticalEncoder;
extern pros::Rotation horizontalEncoder;

// Tracking wheels
extern odyssey::TrackingWheel verticalWheel;
// extern odyssey::TrackingWheel horizontalWheel;

// Drivetrain
extern odyssey::Drivetrain drivetrain;

// PID settings
extern odyssey::ControllerSettings lateralSettings;
extern odyssey::ControllerSettings angularSettings;

// Odometry sensors
extern odyssey::OdomSensors sensors;

// Drive curves
extern odyssey::ExpoDriveCurve throttleCurve;
extern odyssey::ExpoDriveCurve steerCurve;

// Chassis
extern odyssey::Chassis chassis;

// Controller
extern pros::Controller master;

// old congigs 
extern pros::adi::Pneumatics scraper;        // match loader
extern pros::adi::Pneumatics wings;
extern pros::adi::Pneumatics flap;
extern pros::adi::Pneumatics intake_piston;

extern pros::Motor intake_lower;
extern pros::Motor intake_upper;