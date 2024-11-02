// #include "main.h"

// double bound_value(double value)
// {
//     if (value > MAX_RPM)
//         return MAX_RPM;
//     if (value < -MAX_RPM)
//         return -MAX_RPM;
//     return value;
// }

// void initialize()
// {

//     intakeLower.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//     intakeUpper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
// }

// void opcontrol()
// {
//     while (true)
//     {
//         leftY = bound_value(master.get_analog(ANALOG_LEFT_Y));
//         conveyor.move(leftY);

//         if (master.get_digital(DIGITAL_R1))
//         {
//             intakeLower.move(110);
//             intakeUpper.move(110);
//         }
//         else
//         {
//             intakeLower.move(0);
//             intakeUpper.move(0);
//         }

//         if (master.get_digital_new_press(DIGITAL_A))
//             actuated = !actuated;

//         if (actuated)
//             solenoid.set_value(0);
//         else
//             solenoid.set_value(1);

//         pros::delay(5);
//     }
// }
