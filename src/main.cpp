#include "main.h"

void disabled() {}
void competition_initialize() {}

void base_PID(double targetDistance, double targetTurning = 0) {
    double wheel_diameter = 69.85; // Diameter in mm
    double PosConvert = M_PI * wheel_diameter / 360; // Conversion factor

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
    double initialHeading = imu.get_heading(); // Store the initial heading
    double currentHeading = initialHeading;
    double turnError = 0;

    // 1. Perform turning first
    if (targetTurning != 0) {
        imu.tare_heading();
        while (true) {
            currentHeading = imu.get_heading();
            turnError = targetTurning - (currentHeading - initialHeading);

            // Check if we are within the error threshold
            if (fabs(turnError) <= base_error) {
                // Stop turning if we are close enough
                lf.move(0);
                lm.move(0);
                lb.move(0);
                rf.move(0);
                rm.move(0);
                rb.move(0);
                break; // Exit turning loop
            }

            // Calculate turn power
            double turnPower = turnError * 0.5; // Tune this gain

            // Adjust motor powers for turning
            lf.move(-turnPower);  // Adjust left side for turning
            lm.move(-turnPower);
            lb.move(-turnPower);
            rf.move(turnPower);    // Adjust right side for turning
            rm.move(turnPower);
            rb.move(turnPower);

            pros::delay(2); // Delay to reduce CPU load
        }
    }

    // 2. Now perform distance movement
    bool l_move = true;
    bool r_move = true;

    lf.tare_position();
    rf.tare_position();

    while (l_move || r_move) {
        // Get encoder values
        encdleft = lf.get_position() * PosConvert;
        encdright = rf.get_position() * PosConvert;

        // Calculate distance error
        errorLeft = targetDistance - encdleft;
        errorRight = targetDistance - encdright;
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
                powerL = base_kp * errorLeft + base_ki * totalErrorLeft + base_kd * (errorLeft - prevErrorLeft);
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
                powerR = base_kp * errorRight + base_ki * totalErrorRight + base_kd * (errorRight - prevErrorRight);
            }
        }

        // Move the motors
        lf.move(powerL);
        lm.move(powerL);
        lb.move(powerL);
        rf.move(powerR);
        rm.move(powerR);
        rb.move(powerR);

        // Update previous errors for the next iteration
        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;

        pros::delay(2); // Delay to reduce CPU load
    }
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

void brake(){
    lf.brake();
    lm.brake();
    lb.brake();

    rf.brake();
    rm.brake();
    rb.brake();
    pros::delay(2);
}

double bound_value(double value){
    if (value > MAX_RPM) return MAX_RPM;
    if (value < -MAX_RPM) return -MAX_RPM;
    return value;
}

void autonomous(){
    intakeLower.move(110);
	intakeUpper.move(110);
    conveyor.move(110);
    base_PID(0, 90); // Turning first with no distance target
    base_PID(1000);   //move forward
    base_PID(0, 90);
    base_PID(1000);
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
        if(target > slam_dunk.get_value()){
            slam_dunk_l.move(motorPower);
            slam_dunk_r.move(motorPower);
        }
        else if(target < slam_dunk.get_value()){
            slam_dunk_l.move(-motorPower);
            slam_dunk_r.move(-motorPower);
        }

        if(fabs(Error) <= 5){
            slam_dunk_l.move(0);
            slam_dunk_r.move(0);
            slam_dunk_l.brake();
            slam_dunk_r.brake();
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
	conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	imu.reset(true);
    imu.set_data_rate(5);

    slam_dunk.calibrate();

	//pros::Task serial_read(serialRead);
    pros::Task slam_dunk(slamDunk);

	master.clear();
}

void opcontrol(){
	while (true){
		leftY = bound_value(master.get_analog(ANALOG_LEFT_Y)*SCALING_FACTOR);
		rightX = bound_value(master.get_analog(ANALOG_RIGHT_X)*SCALING_FACTOR);

        lf.move(leftY - rightX);
        lm.move(leftY - rightX);
        lb.move(leftY - rightX);

        rf.move(leftY + rightX);
        rm.move(leftY + rightX);
        rb.move(leftY + rightX);

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

        if(actuated) solenoid.set_value(0);
        else solenoid.set_value(1);

		pros::delay(5);
	}
}
