#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <atomic>
#include <string>
#include <sys/types.h>

// CONTROLLER & SENSORS
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(5);                                                                 // inertial sensor
pros::Rotation verticalEnc(20);                                                    // vertical rotational sensor
pros::Rotation horizontalEnc(-18);                                                  // horitontal rotational sensor
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.5); // vertical tracking wheel
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -5.5);       // horizontal tracking wheel
pros::adi::Potentiometer programSelector(3);

// MOTORS
pros::MotorGroup leftMotors({-19, -3, 2}, pros::MotorGearset::blue);                      // front, top, bottom (left)
pros::MotorGroup rightMotors({11, 9, -8}, pros::MotorGearset::blue);                      // front, top, bottom (right)
pros::Motor intake(1, pros::MotorGears::blue, pros::v5::MotorUnits::rotations);           // intake
pros::Motor stakemech(-12, pros::MotorGears::green, pros::v5::MotorEncoderUnits::counts); // lady brown

// PNEUMATICS
pros::adi::Pneumatics mogomech(1, false); // mobile goal mech
pros::adi::Pneumatics doinker(2, false); // doinker mech
pros::adi::Pneumatics intakeLift(3, false); // intake lift 

// DRIVETRAIN SETTINGS
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.25, lemlib::Omniwheel::NEW_325, 360, 8);

// LATERAL PID CONTROLLER
lemlib::ControllerSettings linearController(8.5,  // proportional gain (kP)
                                            0.6,  // integral gain (kI)
                                            40, // derivative gain (kD)
                                            0.5, // anti windup
                                            0, // small error range, in inches 1
                                            0, // small error range timeout, in milliseconds 100
                                            0, // large error range, in inches 3
                                            0, // large error range timeout, in milliseconds 500
                                            0 // maximum acceleration (slew) 20
);

// ANGULAR PID CONTROLLER
lemlib::ControllerSettings angularController(7,// proportional gain (kP)
                                             2,   // integral gain (kI) 2
                                             57, // derivative gain (kD) old: 6x
                                             0.3,   // anti windup
                                             0,    // small error range, in degrees
                                             0,    // small error range timeout, in milliseconds
                                             0,    // large error range, in degrees
                                             0,    // large error range timeout, in milliseconds
                                             0     // maximum acceleration (slew)
);

// TROTTLE INPUT CURVE
lemlib::ExpoDriveCurve throttleCurve(3,    // joystick deadband out of 127
                                     10,   // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// STEER INPUT CURVE
lemlib::ExpoDriveCurve steerCurve(3,    // joystick deadband out of 127
                                  10,   // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// ODOMETRY
lemlib::OdomSensors sensors(&vertical,   // vertical tracking wheel
                            nullptr,     // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr,     // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu         // inertial sensor
);

// DRIVETRAIN
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void disabled() {} // disregard don't delete

void initialize() {

    pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors
    
    /*
    //PID Tuning Setup
    chassis.setPose(0,0,0); // coordinates + heading to 0
    chassis.turnToHeading(90,3000);
    //chassis.moveToPoint(0, 24, 3000);

    while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }

    // Intialize brake mode & postitions
    intake.set_brake_mode(pros::MotorBrake::brake);
    stakemech.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode_all(pros::MotorBrake::coast);
    rightMotors.set_brake_mode_all(pros::MotorBrake::coast);
    stakemech.set_zero_position(0); 
    */
}

void autonomous() {

    
}

// DRIVER CODE UPDATED & FINISHED FOR SUPERNOVA 11/8/24
void opcontrol() {
    /*
    L1 - mogo
    L2 - set lady brown (hold)
    R1 - intake up
    R2 - intake down
    Left Arrow - doinker
    Right Arrow - score lady brown
    Y - down lady brown
    */

    bool last_L1_state = false;
    while (true)
    {
        // drivetrain
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        // intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(-127);
            intake.move(127);
        } else {
            intake.move(0);
            intake.move(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(100);
            intake.move(-75);
        }
        // lady brown
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // set
            stakemech.move_absolute(480, 90); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){ // move up
            stakemech.move(100);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { // move down
            stakemech.move(-127);
            stakemech.set_zero_position(0);
        } else {
            stakemech.move(0);
        }
        // mogomech
        bool current_L1_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        if (current_L1_state && !last_L1_state) {
            mogomech.toggle();
        }
        last_L1_state = current_L1_state;
        // doinker
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            doinker.toggle();
            pros::delay(300);
        }

        pros::delay(25); //controller debounce delay
    }
}