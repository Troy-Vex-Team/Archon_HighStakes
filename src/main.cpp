#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <string>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-2, 1, -3}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({11, 12, -13}, pros::MotorGearset::blue); 

pros::Motor intake(14, pros::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Motor chain(-19, pros::MotorGears::blue, pros::MotorUnits::rotations);
//pros::Motor lift(-21, pros::MotorGears::green, pros::v5::MotorEncoderUnits::rotations);
pros::adi::Pneumatics mogomech(1, true);
pros::adi::Pneumatics release(2, false);

pros::Imu imu(4);
pros::Rotation verticalEnc(-17);
pros::Rotation horizontalEnc(15);

// tracking wheels
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 2);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, .625);

//drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 
                              11.22, 
                              lemlib::Omniwheel::NEW_325, 
                              360, 
                              8);

// lateral PID controller
lemlib::ControllerSettings linearController(6, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            14.5, // derivative gain (kD) 
                                            0.6, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(6.01, // proportional gain (kP)
                                              1.015, // integral gain (kI)
                                              48.75, // derivative gain (kD)
                                              1.5, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate sensors
    
	intake.set_brake_mode(pros::MotorBrake::coast);
	chain.set_brake_mode(pros::MotorBrake::brake);
//  lift.set_brake_mode(pros::MotorBrake::brake);
    /*
    while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f x", chassis.getPose().x);
        pros::lcd::print(3, "%f y", chassis.getPose().y);

        pros::delay(500);
    } */

    
}

void disabled() {} // disregard don't delete

void competition_initialize() {
	//initialize sensors + autonomous selector here
}

void autonomous() {
    // set chassis pose
    pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
    chassis.setPose(0, 0, 0);

    intake.move(127);
    chain.move(127);
    pros::delay(600);
    mogomech.extend();

    //start code here 
    // 1st quadrant 
    chassis.moveToPoint(0, 14, 4000);
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(31, 18, 4000, {.forwards = false}, false);
    //pick up mogo one
    chassis.turnToHeading(0, 1000);
    chassis.moveToPoint(29, 41, 4000);
    chassis.turnToHeading(56.5, 1000);
    chassis.moveToPoint(55.75, 66.5, 4000);
    chassis.moveToPose(54, 84.5, 180, 4000,{.forwards = false});
    chassis.moveToPoint(58.25, 12.75, 4000);
    chassis.moveToPose(34, 20.5, 90, 4000,{.forwards = false});    
    chassis.moveToPoint(54.75, 20.25, 4000);
    chassis.turnToHeading(-27, 1000);
    chassis.moveToPoint(69.0, 4, 4000, {.forwards = false});
    //drop mogo 1

    chassis.moveToPoint(29, 52.5, 4000, {.forwards = false}, false);
    chassis.turnToHeading(184, 1000);
    chassis.moveToPoint(28, 6, 4000);
    chassis.moveToPoint(28.25, 8.25, 4000, {.forwards = false}, false);
    chassis.turnToHeading(146.5, 1000);
    chassis.moveToPoint(31.75, -5, 4000);
    chassis.turnToHeading(339.25, 1000);
    chassis.moveToPoint(33.5, -13.25, 4000);
    //score mogoone
    chassis.moveToPose( -26, -7, -263.75, 4000);
    //pickup mogo
    chassis.moveToPoint(-33, -7, 4000, {.forwards = false}, false);
    chassis.turnToHeading(-371.75, 1000);
    chassis.moveToPose( -30, 10, -368.75, 4000);
    chassis.moveToPose( -56.75, 40.75, -406.75, 4000);
    chassis.turnToHeading(-553, 1000);
    chassis.moveToPoint(-54.25, 28.25, 4000);
    chassis.moveToPose( -58.25, -6.5, -528, 4000);
    chassis.moveToPose( -64.75, -3, -393.5, 4000);
    chassis.moveToPose( -76.5, -10.5, -315.5, 4000);
    //stop intake
    chassis.moveToPose( -50, 43, -346, 4000);
    chassis.turnToHeading(-313, 1000);
    chassis.moveToPoint(-31.25, 64.75, 4000);
    chassis.moveToPose(-23.4, 73.75, -494.5, 4000);
    chassis.moveToPoint(-23.25, 73.75, 4000, {.forwards = false}, false);
    chassis.moveToPose(-68.5, 69.5, -432.25, 4000);
    chassis.moveToPose(-32.0, 73.5, -250.5, 4000);
    chassis.turnToHeading(-212.75, 1000);
    chassis.moveToPoint(-8, 44, 4000);
    chassis.turnToHeading(-250, 1000);
    chassis.moveToPoint(12.75, 30, 4000);
    chassis.turnToHeading(-429.5, 1000);
    //drop mogo #3
    chassis.moveToPose(-16, 74.5, -338.25, 4000);
    chassis.turnToHeading(-427, 1000);
    chassis.moveToPoint(-2, 75, 4000);
    //pickup mogo #4
    chassis.turnToHeading(-438, 1000);
    //drop mogo
    chassis.moveToPoint(1.75, 76.5, 4000);
    chassis.moveToPose(-42.75, 93.5, -422.5, 4000);
    chassis.moveToPose(-49.75, 95.75, -242.5, 4000);
    chassis.moveToPoint(-51.75, 97, 4000);
    //pickup mogo #5
    chassis.moveToPose(-55.25, 100.25, -230, 4000);
    //drop mogo #5
    chassis.moveToPose(-36.25, 77, -152.25, 4000);



    //end code here 
    pros::lcd::print(1, "Autonomous Routine Finished");
    
    

}

void opcontrol() {
    bool mogo = true;
    bool last_L1_state = false; 
	while (true) {
		
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);


		//intake R1
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(100);
			chain.move(100);
		} else {
			intake.move(0);
			chain.move(0);
		}
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            chain.move(-75);
        }
        /*lift
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move(100);

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            lift.move(-100);
        } else {
            lift.move(0);
        }
        bool L2State = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);*/
		//mogomech pneumatics L1
        bool current_L1_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        if (current_L1_state && !last_L1_state) {
            mogomech.toggle(); 
        }
        last_L1_state = current_L1_state; 

        //release pneumatics x buttom
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            release.extend();
        }
		pros::delay(20);                 
	}
	
}
