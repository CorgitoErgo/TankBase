#include "main.h"

void disabled() {}
void competition_initialize() {}

void brake(){
    lf.brake();
    lm.brake();
    lb.brake();

    rf.brake();
    rm.brake();
    rb.brake();
    pros::delay(2);
}

void base_PID(double targetDistance, double targetTurning) {
    // Movement variables
    double powerL = 0;
    double powerR = 0;
    double encdleft = 0;
    double encdright = 0;
    double errorLeft = 0;
    double errorRight = 0;
    double prevErrorLeft = 0;
    double prevErrorRight = 0;
    double totalErrorLeft = 0;
    double totalErrorRight = 0;

    // Turning variables
    double initialHeading = imu.get_rotation(); // Store the initial heading
    double currentHeading = initialHeading;
    double turnError = 0.0;

    // 1. Perform turning first
    if (targetTurning != 0) {
        imu.tare_rotation();
        double prevError = 0.0;
        while (true) {
            currentHeading = fabs(imu.get_rotation());
            turnError = fabs(fabs(targetTurning) - (currentHeading));
            pros::lcd::print(0, "Error: %.lf", turnError);

            // Check if we are within the error threshold
            if (fabs(turnError) <= 1.0) {
                // Stop turning if we are close enough
                brake();
                lf.move(0);
                lm.move(0);
                lb.move(0);
                rf.move(0);
                rm.move(0);
                rb.move(0);
                break; // Exit turning loop
            }
            if (fabs(turnError) > fabs(targetTurning)) break;
            double turnDerivative = prevError - turnError;

            // Calculate turn power
            double turnPower = turnError * turn_Kp + turnDerivative * turn_Kd; // Tune this gain
            prevError = turnError;

            // Adjust motor powers for turning
            if(targetTurning > 0){
                lf.move(-turnPower);  // Adjust left side for turning
                lm.move(-turnPower);
                lb.move(-turnPower);
                rf.move(turnPower);    // Adjust right side for turning
                rm.move(turnPower);
                rb.move(turnPower);
            }
            else if(targetTurning < 0){
                lf.move(turnPower);  // Adjust left side for turning
                lm.move(turnPower);
                lb.move(turnPower);
                rf.move(-turnPower);    // Adjust right side for turning
                rm.move(-turnPower);
                rb.move(-turnPower);
            }
            pros::delay(5); // Delay to reduce CPU load
        }
    }

    bool l_move = false;
    bool r_move = false;

    // 2. Now perform distance movement
    if(targetDistance != 0){
        l_move = true;
        r_move = true;
    }

    lf.tare_position();
    rf.tare_position();

    while (l_move || r_move) {
        // Get encoder values
        encdleft = lf.get_position() * PosConvert;
        encdright = rf.get_position() * PosConvert;

        // Calculate distance error
        errorLeft = fabs(targetDistance) - fabs(encdleft);
        errorRight = fabs(targetDistance) - fabs(encdright);
        totalErrorLeft += errorLeft;
        totalErrorRight += errorRight;

        // PID for left motors
        if (fabs(errorLeft) <= base_error) {
            powerL = 0;
            l_move = false;
        } else {
            // Gradually reduce power as you approach the target
            if (fabs(errorLeft) < decelerationThreshold) {
                powerL *= 0.5; // Reduce power to half when close to target
            } else {
                powerL = base_kp * errorLeft + base_ki * totalErrorLeft + base_kd * (prevErrorLeft - errorLeft);
            }
        }

        // PID for right motors
        if (fabs(errorRight) <= base_error) {
            powerR = 0;
            r_move = false;
        } else {
            // Gradually reduce power as you approach the target
            if (fabs(errorRight) < decelerationThreshold) {
                powerR *= 0.5; // Reduce power to half when close to target
            } else {
                powerR = base_kp * errorRight + base_ki * totalErrorRight + base_kd * (prevErrorLeft - errorRight);
            }
        }

        // Move the motors
        if(targetDistance > 0){
            lf.move(powerL);
            lm.move(powerL);
            lb.move(powerL);
            rf.move(powerR);
            rm.move(powerR);
            rb.move(powerR);
        }
        else if(targetDistance < 0){
            lf.move(-powerL);
            lm.move(-powerL);
            lb.move(-powerL);
            rf.move(-powerR);
            rm.move(-powerR);
            rb.move(-powerR);
        }

        pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
        pros::lcd::print(1, "ErrorR: %.lf", errorRight);

        // Update previous errors for the next iteration
        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;

        pros::delay(2); // Delay to reduce CPU load
    }
    brake();
}

void serialRead(void* params){
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    pros::delay(10);
    pros::screen::set_pen(COLOR_BLUE);
    double distX, distY = 0;
    while(true){
        uint8_t buffer[256];
        int bufLength = 256;
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
        if(nRead >= 0){
            std::stringstream dataStream("");
            bool recordOpticalX, recordOpticalY = false;
            for(int i=0;i<nRead;i++){
                char thisDigit = (char)buffer[i];
                if(thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X'||  thisDigit == 'C'||  thisDigit == 'Y'){
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                if(thisDigit == 'C'){
                    recordOpticalX = false;
                    dataStream >> distX;
                    pros::lcd::print(1, "Optical Flow:");
                    pros::lcd::print(2, "distX: %.2lf", distX/100);
                    dataStream.str(std::string());
                }
                if(thisDigit == 'D'){
                    recordOpticalY = false;
                    dataStream >> distY;
                    global_distY = distY/100;
                    pros::lcd::print(3, "distY: %.2lf", distY/100);
                    dataStream.str(std::string());
                }
                if (recordOpticalX) dataStream << (char)buffer[i];
                if (recordOpticalY) dataStream << (char)buffer[i];
                if (thisDigit == 'X') recordOpticalX = true;
                if (thisDigit == 'Y') recordOpticalY = true;
            }
        }
        pros::Task::delay(25);
    }
}

double bound_value(double value){
    if (value > MAX_RPM) return MAX_RPM;
    if (value < -MAX_RPM) return -MAX_RPM;
    return value;
}

double dead_band(int reading){
    return abs(reading) > 2 ? reading : 0;
}

void groupMove(double distance, int velocity){
    distance = distance / PosConvert;
    lf.tare_position();
    lm.tare_position();
    lb.tare_position();
    rf.tare_position();
    rm.tare_position();
    rb.tare_position();
    while(fabs(lf.get_position()) < fabs(distance)){
        lf.move_absolute(distance, velocity);
        lm.move_absolute(distance, velocity);
        lb.move_absolute(distance, velocity);
        rf.move_absolute(distance, velocity);
        rm.move_absolute(distance, velocity);
        rb.move_absolute(distance, velocity);
        pros::delay(1);
        if(fabs(lf.get_position()) + fabs(rf.get_position())/2.0 >= fabs(distance)) break;
    }
}

void autonomous(){
    intakeLower.move(127);
	intakeUpper.move(127);
    //conveyor.move(110);
    groupMove(60.9, 110);
    pros::delay(300);
    //groupMove(0, 0);
    turn_Kp = 4.5;
    base_PID(0, -15);
    pros::delay(300);
    base_kp = 0.4;
    lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    base_PID(-595, 0);
    groupMove(-180, 90);
    pros::delay(200);
    solenoid.set_value(1);
    //brake();
    //groupMove(0, 0);
    pros::delay(300);
    conveyor.move(100);
    base_PID(0, 45);
    groupMove(200, 110);
    pros::delay(200);
    turn_Kp = 1.1;
    base_PID(0, 147);
    pros::delay(50);
    groupMove(905, 140);
    pros::delay(50);
    turn_Kp = 1.15;
    base_PID(0, 84);
    groupMove(160, 150);
    pros::delay(1000);
    groupMove(160, 120);
    pros::delay(1000);
    //base_PID(200, 0);
    //base_PID()
    //base_PID(0, 89.5);
}

double target = 0;
double slam_Kp = 0.4;
double slam_Kd = 0.1;
double slam_Ki = 0.0;

void slamDunk(){
    double Derivative = 0.0;
    double prevError = 0.0;
    double Error = 0.0;
    double Integral = 0.0;
    while(true){
        if(slammingState == 0){
            target = 680;
        }
        else if(slammingState == 1){
            target = 940;
        }
        else if(slammingState == 2){
            target = 1900;
        }
        else if(slammingState == 3){
            target = 2150;
        }
        Derivative = prevError - Error;
        Error = fabs(target - slam_dunk.get_value());
        Integral += Error;
        double motorPower = slam_Kp * Error + slam_Kd * Derivative + slam_Ki * Integral;

        if(fabs(Error) <= 5){
            slam_dunk_l.move(0);
            slam_dunk_r.move(0);
            slam_dunk_l.brake();
            slam_dunk_r.brake();
        }
        else{
            if(target > slam_dunk.get_value()){
                slam_dunk_l.move(motorPower);
                slam_dunk_r.move(motorPower);
            }
            else if(target < slam_dunk.get_value()){
                slam_dunk_l.move(-motorPower);
                slam_dunk_r.move(-motorPower);
            }
        }
        prevError = Error;
        pros::Task::delay(10);
    }
}

void initialize() {
	pros::lcd::initialize();
	lf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	rf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    slam_dunk_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slam_dunk_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	intakeLower.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intakeUpper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	imu.reset(false);
    imu.set_data_rate(5);

    slam_dunk.calibrate();

	//pros::Task serial_read(serialRead);
    pros::Task slam_dunk(slamDunk);

	master.clear();
}

void opcontrol(){
	while (true){
        if(master.get_digital_new_press(DIGITAL_Y)) tankDrive = !tankDrive;
        if(tankDrive){
            leftY = dead_band(master.get_analog(ANALOG_LEFT_Y));
            rightY = dead_band(master.get_analog(ANALOG_RIGHT_Y));
            lf.move(rightY);
            lm.move(rightY);
            lb.move(rightY);
            rf.move(leftY);
            rm.move(leftY);
            rb.move(leftY);
        }
        else{
            leftY = dead_band(master.get_analog(ANALOG_LEFT_Y));
            rightX = dead_band(master.get_analog(ANALOG_RIGHT_X));
            lf.move(leftY - rightX);
            lm.move(leftY - rightX);
            lb.move(leftY - rightX);
            rf.move(leftY + rightX);
            rm.move(leftY + rightX);
            rb.move(leftY + rightX);
        }

		if(master.get_digital(DIGITAL_R1)){
			intakeLower.move(110);
			intakeUpper.move(110);
		}
        else if(master.get_digital(DIGITAL_R2)){
            intakeLower.move(-110);
			intakeUpper.move(-110);
        }
        else{
            intakeLower.move(0);
			intakeUpper.move(0);
        }

        if(master.get_digital(DIGITAL_L1)){
            conveyor.move(110);
		}
		else if(master.get_digital(DIGITAL_L2)){
            conveyor.move(-110);
		}
        else{
            conveyor.move(0);
        }

        if(master.get_digital_new_press(DIGITAL_A)) actuated = !actuated;

        if(master.get_digital_new_press(DIGITAL_B)) autonomous();

        if(master.get_digital_new_press(DIGITAL_LEFT)) slammingState = 0;
        if(master.get_digital_new_press(DIGITAL_RIGHT)) slammingState = 1;
        if(master.get_digital_new_press(DIGITAL_UP)) slammingState = 2;
        if(master.get_digital_new_press(DIGITAL_DOWN)) slammingState = 3;

        if(actuated) solenoid.set_value(1);
        else solenoid.set_value(0);

		pros::delay(5);
	}
}
