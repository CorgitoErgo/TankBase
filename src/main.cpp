// #include "main.h"
// #include "pros/motors.h"

// void disabled() {}
// void competition_initialize() {}
// void autonomous() {}

// void serialRead(void* params){
//     vexGenericSerialEnable(SERIALPORT - 1, 0);
//     vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
//     pros::delay(10);
//     pros::screen::set_pen(COLOR_BLUE);
//     double distX, distY = 0;
//     while(true){
//         uint8_t buffer[256];
//         int bufLength = 256;
//         int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
//         if(nRead >= 0){
//             std::stringstream dataStream("");
//             bool recordOpticalX, recordOpticalY = false;
//             for(int i=0;i<nRead;i++){
//                 char thisDigit = (char)buffer[i];
//                 if(thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X'||  thisDigit == 'C'||  thisDigit == 'Y'){
//                     recordOpticalX = false;
//                     recordOpticalY = false;
//                 }
//                 if(thisDigit == 'C'){
//                     recordOpticalX = false;
//                     dataStream >> distX;
//                     pros::lcd::print(1, "Optical Flow:");
//                     pros::lcd::print(2, "distX: %.2lf", distX/100);
//                     dataStream.str(std::string());
//                 }
//                 if(thisDigit == 'D'){
//                     recordOpticalY = false;
//                     dataStream >> distY;
//                     global_distY = distY/100;
//                     pros::lcd::print(3, "distY: %.2lf", distY/100);
//                     dataStream.str(std::string());
//                 }
//                 if (recordOpticalX) dataStream << (char)buffer[i];
//                 if (recordOpticalY) dataStream << (char)buffer[i];
//                 if (thisDigit == 'X') recordOpticalX = true;
//                 if (thisDigit == 'Y') recordOpticalY = true;
//             }
//         }
//         pros::Task::delay(25);
//     }
// }

// void brake(){
//     lf.brake();
//     lm.brake();
//     lb.brake();

//     rf.brake();
//     rm.brake();
//     rb.brake();
//     pros::delay(2);
// }

// double bound_value(double value){
//     if (value > MAX_RPM) return MAX_RPM;
//     if (value < -MAX_RPM) return -MAX_RPM;
//     return value;
// }

// void initialize() {
// 	pros::lcd::initialize();
// 	lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
// 	lm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
// 	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

// 	rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
// 	rm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
// 	rb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

// 	intakeLower.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
// 	intakeUpper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
// 	conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

// 	imu.reset(true);
//     imu.set_data_rate(5);

// 	// pros::Task serial_read(serialRead);

// 	master.clear();
// }

// void opcontrol(){
// 	while (true){
// 		leftY = bound_value(master.get_analog(ANALOG_LEFT_Y)*SCALING_FACTOR);
// 		rightX = bound_value(master.get_analog(ANALOG_RIGHT_X)*SCALING_FACTOR);
//         // 		leftY = master.get_analog(ANALOG_LEFT_Y);
// 		// rightX = master.get_analog(ANALOG_RIGHT_X);

//         lf.move(leftY - rightX);
//         lm.move(leftY - rightX);
//         lb.move(leftY - rightX);

//         rf.move(leftY + rightX);
//         rm.move(leftY + rightX);
//         rb.move(leftY + rightX);

// 		if(master.get_digital(DIGITAL_R1)){
// 			intakeLower.move(110);
// 			intakeUpper.move(110);
// 		}
//         else if(master.get_digital(DIGITAL_L1)){
// 			intakeLower.move(-110);
// 			intakeUpper.move(-110);
// 		}
// 		else{
// 			intakeLower.move(0);
// 			intakeUpper.move(0);
// 		}

//         if(master.get_digital(DIGITAL_R2)){
//             conveyor.move(110);
//         }
//         else if(master.get_digital(DIGITAL_L2)){
//             conveyor.move(-110);
//         }
//         else{
//             conveyor.move(0);
//         }

//         if(master.get_digital_new_press(DIGITAL_A)) actuated = !actuated;

//         if(actuated) solenoid.set_value(0);
//         else solenoid.set_value(1);

// 		pros::delay(5);
// 	}
// }
