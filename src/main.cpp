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

//CONTROLLER & SENSORS
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(4); //inertial sensor
pros::Rotation verticalEnc(-17); //vertical rotational sensor
pros::Rotation horizontalEnc(15); //horitontal rotational sensor
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 2); //vertical tracking wheel
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, .625); //horizontal tracking wheel

//MOTORS
pros::MotorGroup leftMotors({-2, 1, -3}, pros::MotorGearset::blue); //front, top, bottom (left)
pros::MotorGroup rightMotors({11, 12, -13}, pros::MotorGearset::blue); //front, top, bottom (right)
pros::Motor intake(14, pros::MotorGears::blue, pros::v5::MotorUnits::rotations); //front intake
pros::Motor chain(-19, pros::MotorGears::blue, pros::MotorUnits::rotations); //chain intake
pros::Motor stakemech(-21, pros::MotorGears::green, pros::v5::MotorEncoderUnits::rotations); //lady brown

//PNEUMATICS
pros::adi::Pneumatics mogomech(1, true); //mobile goal mech (peumatics)

//drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 
                              11.22, 
                              lemlib::Omniwheel::NEW_325, 
                              360, 
                              8
);

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

//DRIVETRAIN
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void checkIntakeChain(){
    if (chain.get_voltage() > 0 && chain.get_actual_velocity() < 25) {
        chain.move_relative(-1, 600);
    }
    chain.move(110);
}

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate sensors
    
    /*
    chassis.setPose(0,0,0); // coordinates + heading to 0
    chassis.turnToHeading(90, 5000); //turn 90 degrees
    chassis.moveToPoint(0, 24, 5000); //forward 24 inches
    
    while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }*/
    
}

void disabled() {} // disregard don't delete

void competition_initialize() {
	//initialize sensors + autonomous selector here
}

void autonomous() {
    // set chassis pose
    pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
    chassis.setPose(0, 0, 0);

    //start code here 
    //score pre load
    intake.move(127);
    chain.move(110);
    pros::delay(600);
    // 1st quadrant 
    chassis.moveToPoint(0, 14, 4000,{.maxSpeed = 80});
    chassis.turnToHeading(-90, 1000,{.maxSpeed = 90});
    mogomech.retract();
    chassis.moveToPoint(31, 18, 4000,{.forwards = false, .maxSpeed = 50}, false);
    //pick up mogo one
    mogomech.extend();
    pros::delay(300);
    chassis.turnToHeading(0, 1000, {.maxSpeed = 90});
    chassis.moveToPoint(23.5, 42, 4000, {.maxSpeed = 55}, false);
    pros::delay(500);
    chassis.turnToHeading(56.5, 1000, {.maxSpeed = 70});
    chassis.moveToPoint(52, 64, 4000, {.maxSpeed = 55});
    chassis.moveToPoint(38.5, 60, 4000, {.forwards = false, .maxSpeed = 55});
    chassis.turnToHeading(180, 1000, {.maxSpeed = 50});
    chassis.moveToPoint(46, 12.75, 4000, {.maxSpeed = 40});
    checkIntakeChain();
    chassis.moveToPose(30, 25, 90, 2500,{.forwards = false, .maxSpeed = 70});    
    chassis.moveToPoint(46, 16, 4000,{.maxSpeed = 50});
    checkIntakeChain();
    chassis.turnToHeading(-22, 1000,{.maxSpeed = 70});
    chassis.moveToPoint(63, 1, 4000, {.forwards = false, .maxSpeed = 70}, false);
    //drop mogo 1
    mogomech.retract();

    //4th quadrant
    chassis.moveToPose(20, 13, 90, 3000);
    chassis.moveToPoint(-27, 16.5, 4000, {.forwards = false, .maxSpeed = 55}, false);
    //pick up mogo 2
    mogomech.extend();
    pros::delay(300);
    chassis.turnToHeading(0, 1000, {.maxSpeed = 90});
    chassis.moveToPoint(-23.5, 42, 4000, {.maxSpeed = 55}, false);
    pros::delay(500);
    chassis.turnToHeading(-56.5, 1000, {.maxSpeed = 70});
    chassis.moveToPoint(-52, 64, 4000, {.maxSpeed = 55});
    chassis.moveToPoint(-40, 60, 4000, {.forwards = false, .maxSpeed = 55});
    chassis.turnToHeading(-180, 1000, {.maxSpeed = 50});
    chassis.moveToPoint(-45, 12.75, 4000, {.maxSpeed = 40});
    checkIntakeChain();
    chassis.moveToPose(-30, 22, -90, 2500,{.forwards = false, .maxSpeed = 70});    
    chassis.moveToPoint(-48, 12, 4000,{.maxSpeed = 50});
    checkIntakeChain();
    chassis.turnToHeading(22, 1000,{.maxSpeed = 70});
    chassis.moveToPoint(-63, -4, 4000, {.forwards = false, .maxSpeed = 70}, false);
    //drop mogo 2
    mogomech.retract();
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
			intake.move(127);
			chain.move(110);
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

		pros::delay(20);                 
	}
	
}
