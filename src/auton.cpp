#include "main.h"

bool is_turning = false;
double integral = 0;
double previous_error = 0;
double target_angle = 0;
double max_power = 50;

void turn_to_angle(double angle) {
  target_angle = angle;
  integral = 0;
  previous_error = 0;
  is_turning = true;
}

void update_turning(double kP, double kI, double kD) {
  if (is_turning) {
    double current_angle = imu.get_heading();

    double error = target_angle - current_angle;
    // if (error > 180) error -= 360;
    // if (error < -180) error += 360;
    // if (error < 0) error += 360;
    integral += error;
    double derivative = error - previous_error;
    previous_error = error;

    double power = kP * error + kI * integral + kD * derivative;

    // LEFT_MOTOR_FRONT.move(power);
    // LEFT_MOTOR_REAR.move(power);
    // RIGHT_MOTOR_FRONT.move(-power);
    // RIGHT_MOTOR_REAR.move(-power);
    if (power > max_power)
      power = max_power;
    else if (power < -max_power)
      power = -max_power;
    pros::lcd::print(2, "Current Angle: %f", current_angle);
    pros::lcd::print(3, "Error: %f", error);
    pros::lcd::print(4, "Power: %f", power);

    lf.move(-power);
    lm.move(-power);
    lb.move(-power);
    rf.move(power);
    rm.move(power);
    rb.move(power);

    if (fabs(error) < 2) {
      is_turning = false;
      // LEFT_MOTOR_FRONT.move(0);
      // LEFT_MOTOR_REAR.move(0);
      // RIGHT_MOTOR_FRONT.move(0);
      // RIGHT_MOTOR_REAR.move(0);
      lf.move(0);
      lm.move(0);
      lb.move(0);
      rf.move(0);
      rm.move(0);
      rb.move(0);
    }
  }
}

void initialize() {
  pros::lcd::initialize();
  pros::lcd::print(0, "LCD initialized");

  imu.reset(true);
  imu.set_data_rate(5);

  master.clear();
}

void opcontrol() {
  lf.move_relative(574, 200);
  lm.move_relative(574, 200);
  lb.move_relative(574, 200);
  rf.move_relative(574, -200);
  rm.move_relative(574, -200);
  rb.move_relative(574, -200);
  intakeLower.move(110);
  intakeUpper.move(110);

  pros::delay(1000);
  lf.move_relative(-574, 100);
  lm.move_relative(-574, 100);
  lb.move_relative(-574, 100);
  rf.move_relative(-574, -100);
  rm.move_relative(-574, -100);
  rb.move_relative(-574, -100);
  conveyor.move(55);
  pros::delay(1000);
  conveyor.move(0);
  pros::delay(1000);
  lf.move(20);
  lm.move(20);
  lb.move(20);
  rf.move(-20);
  rm.move(-20);
  rb.move(-20);
  pros::delay(1000);

  turn_to_angle(270);
  while (is_turning) {
    update_turning(1.0, 0, 0.5);
    pros::delay(20); // Control update rate
  }
    pros::delay(200);
  lf.move_relative(656, 100);
  lm.move_relative(656, 100);
  lb.move_relative(656, 100);
  rf.move_relative(656, -100);
  rm.move_relative(656, -100);
  rb.move_relative(656, -100);
  pros::delay(500);
  turn_to_angle(90);
  
  while (is_turning) {
    update_turning(0.8, 0, 0.4);
    pros::delay(20); // Control update rate
  }
  pros::delay(200);
  lf.move_relative(-1300, -200*SCALING_FACTOR);
  lm.move_relative(-1300, -200*SCALING_FACTOR);
  lb.move_relative(-1300, -200*SCALING_FACTOR);
  rf.move_relative(-1300, 200*SCALING_FACTOR);
  rm.move_relative(-1300, 200*SCALING_FACTOR);
  rb.move_relative(-1300, 200*SCALING_FACTOR);
  pros::delay(2000);
  solenoid.set_value(1);
  pros::delay(500);
  conveyor.move(110);
  turn_to_angle(270);
  
  while (is_turning) {
    update_turning(1.7, 0, 0.7);
    pros::delay(20); // Control update rate
  }
  pros::delay(200);
  lf.move_relative(2296, 200);
  lm.move_relative(2296, 200);
  lb.move_relative(2296, 200);
  rf.move_relative(2296, -200);
  rm.move_relative(2296, -200);
  rb.move_relative(2296, -200);

}