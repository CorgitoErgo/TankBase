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
    double turnIntegral = 0.0;

    // 1. Perform turning first
    if (targetTurning != 0) {
        while(!imu.tare_rotation());
        double prevError = 0.0;
        while (true) {
            currentHeading = fabs(imu.get_rotation());
            turnError = fabs(fabs(targetTurning) - (currentHeading));
            pros::lcd::print(0, "Error: %.lf", turnError);

            // Check if we are within the error threshold
            if (fabs(turnError) <= turn_margin) {
                break; // Exit turning loop
            }
            double turnDerivative = turnError - prevError;

            // Calculate turn power
            double turnPower = turnError * turn_Kp + turnDerivative * turn_Kd + turnIntegral * turn_Ki; // Tune this gain
            turnPower = std::clamp(turnPower, 80.0, 210.0);
            prevError = turnError;
            turnIntegral += turnError;

            // Adjust motor powers for turning
            if(targetTurning > 0){
                lf.move_velocity(-turnPower);  // Adjust left side for turning
                lm.move_velocity(-turnPower);
                lb.move_velocity(-turnPower);
                rf.move_velocity(turnPower);    // Adjust right side for turning
                rm.move_velocity(turnPower);
                rb.move_velocity(turnPower);
            }
            else if(targetTurning < 0){
                lf.move_velocity(turnPower);  // Adjust left side for turning
                lm.move_velocity(turnPower);
                lb.move_velocity(turnPower);
                rf.move_velocity(-turnPower);    // Adjust right side for turning
                rm.move_velocity(-turnPower);
                rb.move_velocity(-turnPower);
            }
            if (fabs(turnError) > fabs(targetTurning)) break;
            pros::delay(2); // Delay to reduce CPU load
        }
    }

    bool l_move = false;
    bool r_move = false;
    int timeout = 0;

    // 2. Now perform distance movement
    if(targetDistance != 0){
        l_move = true;
        r_move = true;
    }

    lf.tare_position();
    rf.tare_position();
    double encoder_degrees = targetDistance * degrees_per_mm;
    int encoder_ticks = static_cast<int>(round(encoder_degrees));
    while (l_move || r_move) {
        // Get encoder values
        encdleft = lf.get_position() / degrees_per_mm;
        encdright = rf.get_position() / degrees_per_mm;

        // Calculate distance error
        errorLeft = fabs(targetDistance) - fabs(encdleft);
        errorRight = fabs(targetDistance) - fabs(encdright);
        totalErrorLeft += errorLeft;
        totalErrorRight += errorRight;

        // PID for left motors
        if (fabs(errorLeft) <= base_error) {
            powerL = 0.0;
            l_move = false;
        } else {
            // Gradually reduce power as you approach the target
            if (fabs(errorLeft) < decelerationThreshold) {
                powerL *= 0.5; // Reduce power to half when close to target
            } else {
                powerL = base_kp * errorLeft + base_ki * totalErrorLeft + base_kd * (errorLeft - prevErrorLeft);
            }
            powerL = std::clamp(powerL, 80.0, 175.0);
        }

        // PID for right motors
        if (fabs(errorRight) <= base_error) {
            powerR = 0.0;
            r_move = false;
        } else {
            // Gradually reduce power as you approach the target
            if (fabs(errorRight) < decelerationThreshold) {
                powerR *= 0.5; // Reduce power to half when close to target
            } else {
                powerR = base_kp * errorRight + base_ki * totalErrorRight + base_kd * (errorRight - prevErrorRight);
            }
            powerR = std::clamp(powerR, 80.0, 175.0);
        }

        if(errorRight < 0 && errorLeft < 0) break;

        // Move the motors
        if(targetDistance > 0.0){
            lm.move_velocity(powerL);
            lf.move_velocity(powerL);
            lb.move_velocity(powerL);
            rf.move_velocity(powerR);
            rm.move_velocity(powerR);
            rb.move_velocity(powerR);
        }
        else if(targetDistance < 0.0){
            lf.move_velocity(-powerL);
            lm.move_velocity(-powerL);
            lb.move_velocity(-powerL);
            rf.move_velocity(-powerR);
            rm.move_velocity(-powerR);
            rb.move_velocity(-powerR);
        }

        if(timeout >= 2600) break;

        pros::lcd::print(0, "ErrorL: %.lf", errorLeft);
        pros::lcd::print(1, "ErrorR: %.lf", errorRight);

        // Update previous errors for the next iteration
        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;

        pros::delay(2); // Delay to reduce CPU load
        timeout += 2;
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

void groupMove(double distance_mm, int velocity){
    double encoder_degrees = distance_mm * degrees_per_mm;
    int encoder_ticks = static_cast<int>(round(encoder_degrees));
    lf.tare_position();
    lm.tare_position();
    lb.tare_position();
    rf.tare_position();
    rm.tare_position();
    rb.tare_position();
    while((fabs(rf.get_position()) + fabs(lf.get_position()))/2.0 < fabs(encoder_ticks)){
        lm.move_absolute(encoder_ticks, velocity);
        lb.move_absolute(encoder_ticks, velocity);
        lf.move_absolute(encoder_ticks, velocity);

        rf.move_absolute(encoder_ticks, velocity);
        rm.move_absolute(encoder_ticks, velocity);
        rb.move_absolute(encoder_ticks, velocity);
        if((fabs(rf.get_position()) + fabs(lf.get_position()))/2.0 >= fabs(encoder_ticks)) break;
    }
    brake();
}

void moveTime(int delay_ms, int velocity){
    lm.move_velocity(velocity);
    lb.move_velocity(velocity);
    lf.move_velocity(velocity);
    rf.move_velocity(velocity);
    rm.move_velocity(velocity);
    rb.move_velocity(velocity);
    pros::delay(delay_ms);
}

void autonomous(){
    intakeLower.move(127);
	intakeUpper.move(127);
    //groupMove(105, 70);
    base_PID(16, 0);
    //moveTime(800, 100);
    //pros::delay(100);
    pros::delay(300);
    turn_Kp = 0.5;
    turn_Kd = 0.0;
    turn_Ki = 0.001;
    turn_margin = 0.4;
    //base_PID(0, -1.0);
    turn_Ki = 0.0;
    //pros::delay(100);
    lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // base_error = 4.0;
    // base_kp = 0.14;
    // base_ki = 0.001;
    // base_kd = 0.01;
    // base_PID(-630, 0);

    base_PID(-840, 0);
    //groupMove(-630, 120);
    //groupMove(-200, 60);
    pros::delay(100);
    solenoid.set_value(1);
    pros::delay(500);
    conveyor.move(100);

    turn_Kp = 0.45;
    base_PID(0, 6);
    //pros::delay(100);
    base_kp = 1.7;
    base_PID(418, 0);
    base_kp = 1.58;
    //groupMove(415, 120);
    pros::delay(100);
    turn_Kp = 1.1;
    turn_margin = 1.0;
    base_PID(0, 121);
    turn_margin = 1.0;
    pros::delay(100);
    base_PID(540, 0);
    //groupMove(520, 140);
    pros::delay(100);
    turn_Kp = 0.61;
    turn_margin = 0.7;
    base_PID(0, 76);
    pros::delay(100);
    base_PID(818, 0);
    //groupMove(815, 140);
    pros::delay(100);
    turn_Kp = 0.4;
    turn_Ki = 0.005;
    turn_margin = 0.5;
    base_PID(0, 32);
    pros::delay(100);
    base_PID(180, 0);
    //groupMove(180, 70);
    pros::delay(200);
    base_PID(-180, 0);
    //groupMove(-180, 100);
    pros::delay(400);
    //base_PID(350, 0);
    moveTime(1200, 90);
    pros::delay(200);
    while(!imu.tare_heading());
    base_PID(-180, 0);
    //groupMove(-190, 140);
    turn_Kp = 0.71;
    turn_Ki = 0.001;
    turn_Kd = 0.0;
    turn_margin = 0.6;
    base_PID(0, -104);
    pros::delay(500);
    moveTime(2750, -180);
    pros::delay(800);
    solenoid.set_value(0);
    conveyor.move(-60);
    pros::delay(280);
    conveyor.move(0);
    //base_PID(200, 0);
    //groupMove(200, 90);
    pros::delay(500);
    turn_Kp = 0.46;
    turn_Kd = 0.0;
    turn_Ki = 0.001;
    turn_margin = 1.0;
    base_PID(100, 0);
    pros::delay(800);
    if(imu.get_heading() > 285.0)
        base_PID(0, -(imu.get_heading() - 285.0));
    else if(imu.get_heading() < 285.0)
        base_PID(0, 285.0 - imu.get_heading());
    base_PID(1100, 0);
    pros::delay(800);

    // //groupMove(1100, 90);
    base_PID(0, 157);
    pros::delay(800);
    base_PID(-750, 0);
    // //groupMove(-1000, 140);
    pros::delay(100);
    solenoid.set_value(1);
    pros::delay(320);
    conveyor.move(120);
    intakeLower.move(127);
	intakeUpper.move(127);
    // base_PID(0, -35);
    // groupMove(600, 300);

    base_PID(0, 88);
    pros::delay(100);
    base_PID(415, 0);
    pros::delay(100);
    base_PID(0, -170);
    pros::delay(100);
    base_PID(500, 0);

    // turn_Kp = 0.51;
    // turn_Kd = 0.001;
    // turn_Ki = 0.0001;
    // base_PID(0, -85);
    // pros::delay(100);



    // pros::delay(200);
    // groupMove(-100, 150);
    // pros::delay(600);
    // groupMove(300, 150);
    // pros::delay(50);
    // groupMove(-200, 140);
    // pros::delay(200);
    // turn_Kp = 2.2;
    // turn_margin = 2.0;
    // base_PID(0, -102);
    // pros::delay(500);
    // groupMove(-2000, 140);
    // pros::delay(2000);


    // groupMove(-160, 140);
    // pros::delay(800);
    // groupMove(350, 140);
    // pros::delay(1000);
    // groupMove(-200, 140);
    // pros::delay(1000);
    // turn_Kp = 2.2;
    // turn_margin = 3.0;
    // base_PID(0, -103);
    // pros::delay(500);
    // groupMove(-2000, 140);
    // pros::delay(2000);
}

void slamDunk(){
    double Derivative = 0.0;
    double prevError = 0.0;
    double Error = 0.0;
    double Integral = 0.0;
    while(true){
        if(slammingState == 0){
            slam_target = 660;
        }
        else if(slammingState == 1){
            slam_target = 870;
        }
        else if(slammingState == 2){
            slam_target = 1900;
        }
        else if(slammingState == 3){
            slam_target = 2150;
        }
        Derivative = prevError - Error;
        Error = fabs(slam_target - slam_dunk.get_value());
        Integral += Error;
        double motorPower = slam_Kp * Error + slam_Kd * Derivative + slam_Ki * Integral;

        if(fabs(Error) <= 10){
            slam_dunk_l.move(0);
            slam_dunk_r.move(0);
            slam_dunk_l.brake();
            slam_dunk_r.brake();
        }
        else{
            if(slam_target > slam_dunk.get_value() + 10){
                slam_dunk_l.move(motorPower);
                slam_dunk_r.move(motorPower);
            }
            else if(slam_target < slam_dunk.get_value() - 10){
                slam_dunk_l.move(-motorPower);
                slam_dunk_r.move(-motorPower);
            }
        }
        prevError = Error;
        pros::Task::delay(15);
    }
}

void initialize() {
	pros::lcd::initialize();
	lf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	rf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    slam_dunk_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slam_dunk_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	intakeLower.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intakeUpper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	imu.reset(true);
    imu.set_data_rate(5);

    slam_dunk.calibrate();

	//pros::Task serial_read(serialRead);
    pros::Task slam_dunk(slamDunk);

	//master.clear();
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
        if(master.get_digital_new_press(DIGITAL_Y)) imu.reset(true);
        if(actuated) solenoid.set_value(1);
        else solenoid.set_value(0);

		pros::delay(5);
	}
}
