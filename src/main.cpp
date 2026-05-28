#include "main.h"

void drive_test_auton() {
  odom.reset();
  chassis.drive_distance(24.0, 3000);
  chassis.turn_to_heading(90.0, 2000);
  chassis.drive_to_point(24.0, 24.0, 4000);
}

void path_test_auton() {
  odom.reset();

  Vortex::FollowParams follow;
  follow.lookahead_in = 10.0;
  follow.smooth = true;

  chassis.follow_path(
      Vortex::make_path({
          {0.0, 0.0, 100},
          {0.0, 24.0, 110},
          {24.0, 24.0, 90},
      }),
      5000, follow);

  chassis.drive_to_pose(36.0, 24.0, 90.0, 3500);
}

void initialize() {
  printf("Starting robot...\n");
  odom.init();
  chassis.init();

  auton_selector
      .add("Drive test", drive_test_auton)
      .add("Path test", path_test_auton);
  auton_selector.bind_dashboard(&odom, &chassis, &master);
  auton_selector.load_selected();
  auton_selector.print();
  auton_selector.start_dashboard();

  pid_tuner.bind_chassis(&chassis);
  pid_tuner.bind_defaults();

  printf("Robot ready.\n");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  auton_selector.run_selected();
}

void opcontrol() {
  std::uint32_t last_print = 0;

  while (true) {
    const int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(throttle, turn);

    if (pros::millis() - last_print > 250) {
      const Vortex::Pose pose = odom.get_pose();
      printf("x:%6.2f y:%6.2f h:%6.2f\n", pose.x, pose.y,
             pose.heading_deg());
      last_print = pros::millis();
    }

    pros::delay(10);
  }
}
