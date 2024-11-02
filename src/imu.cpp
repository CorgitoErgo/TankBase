// #include "main.h"

// bool is_turning = false;
// double integral = 0;
// double previous_error = 0;
// double target_angle = 0;
// double max_power = 50;


// double kp = 0.0;
// double ki = 0.0;
// double kd = 0.0;

// void turn_to_angle(double angle) {
//   target_angle = angle;
//   integral = 0;
//   previous_error = 0;
//   is_turning = true;
// }

// void update_turning(double kP,double kI, double kD) {
//   if (is_turning) {
//     double current_angle = imu.get_heading();
//     double error = target_angle - current_angle;

//     integral += error;
//     double derivative = error - previous_error;
//     previous_error = error;

//     double power = kP * error + kI * integral + kD * derivative;

//     // LEFT_MOTOR_FRONT.move(power);
//     // LEFT_MOTOR_REAR.move(power);
//     // RIGHT_MOTOR_FRONT.move(-power);
//     // RIGHT_MOTOR_REAR.move(-power);
//     if (power > max_power) power = max_power;
//     else if (power < -max_power) power = -max_power;
//     pros::lcd::print(2, "Current Angle: %f", current_angle);
//     pros::lcd::print(3, "Error: %f", error);
//     pros::lcd::print(4, "Power: %f", power);

//     lf.move(-power);
//     lm.move(-power);
//     lb.move(-power);
//     rf.move(power);
//     rm.move(power);
//     rb.move(power);

    

//     if (fabs(error) < 2) {
//       is_turning = false;
//       // LEFT_MOTOR_FRONT.move(0);
//       // LEFT_MOTOR_REAR.move(0);
//       // RIGHT_MOTOR_FRONT.move(0);
//       // RIGHT_MOTOR_REAR.move(0);
//       lf.move(0);
//       lm.move(0);
//       lb.move(0);
//       rf.move(0);
//       rm.move(0);
//       rb.move(0);
//     }
//   }
// }

// void initialize() {
// 	pros::lcd::initialize();
//   pros::lcd::print(0, "LCD initialized");

// 	imu.reset(true);
//   imu.set_data_rate(5);

// 	master.clear();
// }

// void opcontrol() {
//   while (true) {
    
//     // You can call the turn function here as well if needed
//     if (master.get_digital_new_press(DIGITAL_X)) {
//       pros::lcd::print(1, "mehmeh", imu.get_heading());
//       turn_to_angle(90);
//        kp= 0.75;
//        ki = 0.0;
//         kd = 0.4;
//     }

//     if (master.get_digital_new_press(DIGITAL_A)) {
//       pros::lcd::print(1, "mehmeh", imu.get_heading());
//       turn_to_angle(60);
//        kp= 0.3;
//        ki = 0.0;
//         kd = 0.3;
//     }

//     if (master.get_digital_new_press(DIGITAL_B)) {
//       pros::lcd::print(1, "mehmeh", imu.get_heading());
//       turn_to_angle(270);
//        kp= 0.8;
//        ki = 0.0;
//         kd = 0.4;

      
//     }
//     update_turning(kp,ki,kd);
    
    
//     pros::delay(50);
//   }
// }