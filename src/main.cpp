#include "main.h"

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); // ~3 s: calibrates the IMU, starts odometry

    // optional: live pose readout on the brain screen
    pros::Task screenTask([&]() {
        while (true) {
            odyssey::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f in", pose.x);
            pros::lcd::print(1, "Y: %.2f in", pose.y);
            pros::lcd::print(2, "Heading: %.2f deg", pose.theta);
            pros::delay(50);
        }
    });
}

void autonomous() {
	chassis.setPose(48, 16, 0);

	odyssey::MoveToPointParams moveParams;
	moveParams.maxSpeed = 70;
	chassis.moveToPoint(48, 48, 7000, moveParams);
	chassis.waitUntilDone();

	odyssey::TurnToPointParams turnParams;
	turnParams.maxSpeed = 70;
	chassis.turnToPoint(90, 48, 7000, turnParams);
	chassis.waitUntilDone();

	moveParams = {};
	moveParams.maxSpeed = 70;
	chassis.moveToPoint(60, 48, 7000, moveParams);
	chassis.waitUntilDone();

	moveParams.maxSpeed = 50;
	moveParams.forwards = false;
	chassis.moveToPoint(26, 48, 7000, moveParams);
	chassis.waitUntilDone();


	moveParams.maxSpeed = 70;
	moveParams.forwards = true;
	chassis.moveToPoint(40, 48, 7000, moveParams);
	chassis.waitUntilDone();

	moveParams.maxSpeed = 70;
	chassis.moveToPoint(24, 24, 7000, moveParams);
	chassis.waitUntilDone();

	moveParams.maxSpeed = 70;
	chassis.moveToPoint(24, -24, 7000, moveParams);
	chassis.waitUntilDone();

	moveParams.maxSpeed = 70;
	moveParams.forwards = false;
	chassis.moveToPoint(12, -12, 7000, moveParams);
	chassis.waitUntilDone();

	pros::delay(1000);

	moveParams.maxSpeed = 70;
	moveParams.forwards = true;
	chassis.moveToPoint(40, -48, 3000, moveParams);
	chassis.waitUntilDone();
}

void opcontrol() {
    while (true) {
        const int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);
        pros::delay(20);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			autonomous();
			break;
		}
		intake.updateOpControl();
    }

	
}