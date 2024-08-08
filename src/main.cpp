#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-1, 2, -3}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({5, -4, -11}, pros::MotorGearset::blue); 
pros::Motor intake(-21, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Motor elevator(-20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
pros::adi::Pneumatics mogomech(1, true);

pros::Imu imu(10);
pros::Rotation verticalSensor(8);
pros::Rotation horizontalSensor(9);

//drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 
                              11.22, 
                              lemlib::Omniwheel::NEW_325, 
                              360, 
                              2
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "TYPE SHI");
	
	intake.set_brake_mode(pros::MotorBrake::coast);
	elevator.set_brake_mode(pros::MotorBrake::brake);
}

void disabled() {} // disregard don't delete

void competition_initialize() {
	//initialize sensors + autonomous selector here
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	while (true) {
		
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25);


		//intake R1
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(100);
			elevator.move(100);
		} else {
			intake.move(0);
			elevator.move(0);
		}
		//mogomech pneumatics L1, L2
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			mogomech.extend();
		} 
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			mogomech.retract();
		}
		pros::delay(20);                 
	}
}