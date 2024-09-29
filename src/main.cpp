#include "main.h"
#include "lemlib/api.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
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
                              2);

// lateral PID controller
lemlib::ControllerSettings linearController(32.8, // proportional gain (kP)
                                            10, // integral gain (kI)
                                            97, // derivative gain (kD)
                                            0, // anti windup
                                            5, // small error range, in inches
                                            1000, // small error range timeout, in milliseconds
                                            10, // large error range, in inches
                                            2000, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(6.2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              50.5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

//barely oscillates and undershoots: 5.9,0,50

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

//Runs initialization code. This occurs as soon as the program is started.

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate sensors
    
	intake.set_brake_mode(pros::MotorBrake::coast);
	chain.set_brake_mode(pros::MotorBrake::brake);
//  lift.set_brake_mode(pros::MotorBrake::brake);

    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
   	// turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90,2000);
    chassis.turnToHeading(0, 5000); 
    //chassis.turnToHeading(180, 100000); 

    //chassis.moveToPose(0, 0, 90, 10000);
    while (1) {
            pros::lcd::print(3, "%f Heading", chassis.getPose().theta); 
            pros::lcd::print(1, "%f X", chassis.getPose().x); 
            pros::lcd::print(2, "%f Y", chassis.getPose().y);

            pros::delay(500);
        }
}

void disabled() {} // disregard don't delete

void competition_initialize() {
	//initialize sensors + autonomous selector here
}

void autonomous() {

}

void opcontrol() {
    bool mogo = true;
    bool last_L1_state = false; 
	while (true) {
		
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY*.95, rightX*.95);

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
