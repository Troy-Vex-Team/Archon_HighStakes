#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-1, 2, -3}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({5, -4, -11}, pros::MotorGearset::blue); 
pros::Motor intake(-21, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Motor elevator(-20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
pros::adi::Pneumatics mogomech(1, true);

pros::Imu imu(10);
pros::Rotation verticalEnc(8);
pros::Rotation horizontalEnc(9);

// tracking wheels
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.5);

//drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 
                              11.22, 
                              lemlib::Omniwheel::NEW_325, 
                              360, 
                              2
);

// lateral PID controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate sensors
	pros::lcd::set_text(1, "TYPE SHI");

	while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
	
	intake.set_brake_mode(pros::MotorBrake::coast);
	elevator.set_brake_mode(pros::MotorBrake::brake);
        pros::delay(50); //delay
	}
}

void disabled() {} // disregard don't delete

void competition_initialize() {
	//initialize sensors + autonomous selector here
}

void autonomous() {
	chassis.setPose(0,0,0);
	chassis.turnToHeading(90,100000);
}

void opcontrol() {

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
