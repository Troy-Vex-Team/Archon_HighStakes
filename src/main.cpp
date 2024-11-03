#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <string>
#include <sys/types.h>

// CONTROLLER & SENSORS
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(21);                                                                 // inertial sensor
pros::Rotation verticalEnc(20);                                                    // vertical rotational sensor
pros::Rotation horizontalEnc(18);                                                  // horitontal rotational sensor
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 1.5); // vertical tracking wheel
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);       // horizontal tracking wheel
pros::adi::Potentiometer programSelector(3);

// MOTORS
pros::MotorGroup leftMotors({-11, -3, 2}, pros::MotorGearset::blue);                      // front, top, bottom (left)
pros::MotorGroup rightMotors({19, 9, -8}, pros::MotorGearset::blue);                      // front, top, bottom (right)
pros::Motor intake(1, pros::MotorGears::blue, pros::v5::MotorUnits::rotations);           // front intake
pros::Motor chain(-13, pros::MotorGears::blue, pros::MotorUnits::rotations);              // chain intake
pros::Motor stakemech(-12, pros::MotorGears::green, pros::v5::MotorEncoderUnits::counts); // lady brown

// PNEUMATICS
pros::adi::Pneumatics mogomech(1, true); // mobile goal mech
pros::adi::Pneumatics doinker(2, false); // doinker mech

// DRIVETRAIN SETTINGS
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors,
                              11.22,
                              lemlib::Omniwheel::NEW_325,
                              360,
                              8);

// LATERAL PID CONTROLLER
lemlib::ControllerSettings linearController(7.2,  // proportional gain (kP)
                                            .95,  // integral gain (kI)
                                            33.7, // derivative gain (kD)
                                            0.07, // anti windup
                                            0,    // small error range, in inches
                                            0,    // small error range timeout, in milliseconds
                                            0,    // large error range, in inches
                                            0,    // large error range timeout, in milliseconds
                                            0     // maximum acceleration (slew)
);

// ANGULAR PID CONTROLLER
lemlib::ControllerSettings angularController(5.25, // proportional gain (kP)
                                             1,    // integral gain (kI)
                                             44.8, // derivative gain (kD)
                                             2,    // anti windup
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

void selector() {
    while (true) {
        // Get the angle reading from the potentiometer

        // Print the reading directly
        pros::lcd::print(1, "Heading: %f", programSelector.get_value()/10.0); // Convert tenths to degrees

        // Delay for readability
        pros::delay(500);
    }
}

void initialize() {
    /*PID Tuning Setup
    chassis.setPose(0,0,0); // coordinates + heading to 0
    chassis.turnToHeading(120,3000);
    chassis.turnToHeading(0,3000);
    chassis.moveToPoint(0, 24, 5000); //forward 24 inches
    */

    pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors

    /*while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }

    // Intialize brake mode & postitions
    intake.set_brake_mode(pros::MotorBrake::coast);
    chain.set_brake_mode(pros::MotorBrake::coast);
    stakemech.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode_all(pros::MotorBrake::coast);
    rightMotors.set_brake_mode_all(pros::MotorBrake::coast);
    stakemech.set_zero_position(0); */

}

void autonomous() {
    /*
    0-100 = Red ring side
    100-200 = Red goal side
    200-300 = Blue goal side
    300-410 = Blue ring side
    */
    
    if (0 < programSelector.get_value()/10.0 <= 100.0) {
        //red ring side
    } else if (100.0 < programSelector.get_value()/10.0 <= 200.0) {
        //red goal side
    } else if (200.0 < programSelector.get_value()/10.0 <= 300.0) {
        //blue goal side
    } else if (300.0 < programSelector.get_value()/10.0 <= 410.0) {
        //blue ring side
    }

    intake.move(-127);
    chain.move(120);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -27.25, 1500, {.forwards=false, .maxSpeed=70});
    mogomech.retract(); //clamp mogo #1
    chassis.turnToHeading(56.5, 1500, {.maxSpeed=80});
    chassis.moveToPoint(16, -21, 1500); // ring 2
    chassis.swingToHeading(144, lemlib::DriveSide::RIGHT, 1500);
    chassis.moveToPoint(31.5, -28.5, 1500);
    chassis.moveToPose(29.25, -21.5,181.75, 1500, {.forwards=false});
    chassis.moveToPoint(26.75, -35, 1500);
    chassis.swingToHeading(44, lemlib::DriveSide::RIGHT, 1500, {.maxSpeed=80});
    chassis.moveToPoint(-36.75, -35, 1500, {.forwards=false, .maxSpeed=70});




    /*
    chassis.moveToPoint(-17.5, -30.25, 2000, {.forwards = false, .maxSpeed = 30}, false);
    mogomech.retract();
    //going to first stack
    chassis.moveToPoint(6.5, -26.5, 1000);
    chassis.moveToPoint(3, -25.25, 1000,{.forwards=false});
    //going to left two stack
    chassis.moveToPose(3, -51, 180, 1000);
    pros::delay(500);
    //prepare for right two stack
    chassis.moveToPose(6.5,-25,180,1000,{.forwards = false, .maxSpeed = 100});
    //go to right two stack
    chassis.moveToPose(14, -43.25,180, 1000);
    //go to middle red
    chassis.moveToPose(-38,0,-90,2500,{.forwards = false, .maxSpeed = 100});
    mogomech.extend();
    doinker.extend();
    */
    
}

// DRIVER CODE UPDATED & FINISHED FOR SUPERNOVA 11/1/24
void opcontrol() {
    /*
    L1 - mogo
    L2 - set lady brown
    left - doinker
    R1 - intake up
    R2 - chain down
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
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.move(-127);
            chain.move(127);
        }
        else
        {
            intake.move(0);
            chain.move(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake.move(100);
            chain.move(-75);
        }
        // lady brown
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            stakemech.move_absolute(440, 90);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        { // move up
            if (stakemech.get_position() > 1750)
            {
                stakemech.move(-25);
            }
            else
            {
                stakemech.move(100);
            }
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
        { // move down
            stakemech.move(-100);
        }
        else if (stakemech.get_position() > 1750)
        {
            stakemech.move(-25);
        }
        else
        {
            stakemech.move(0);
        }
        // mogomech
        bool current_L1_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        if (current_L1_state && !last_L1_state)
        {
            mogomech.toggle();
        }
        last_L1_state = current_L1_state;

        // doinker
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            doinker.toggle();
            pros::delay(300);
        }

        pros::delay(25);
    }
}
