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
#include "pros/rtos.hpp"
#include <string>

// CONTROLLER & SENSORS
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(21);                                                            // inertial sensor
pros::Rotation verticalEnc(20);                                               // vertical rotational sensor
pros::Rotation horizontalEnc(18);                                              // horitontal rotational sensor
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 1.5); // vertical tracking wheel
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);  // horizontal tracking wheel

// MOTORS
pros::MotorGroup leftMotors({-11, -3, 2}, pros::MotorGearset::blue);                         // front, top, bottom (left)
pros::MotorGroup rightMotors({19, 9, -8}, pros::MotorGearset::blue);                         // front, top, bottom (right)
pros::Motor intake(1, pros::MotorGears::blue, pros::v5::MotorUnits::rotations);              // front intake
pros::Motor chain(-13, pros::MotorGears::blue, pros::MotorUnits::rotations);                 // chain intake
pros::Motor stakemech(-12, pros::MotorGears::green, pros::v5::MotorEncoderUnits::counts); // lady brown

// PNEUMATICS
pros::adi::Pneumatics mogomech(1, true); //mobile goal mech
pros::adi::Pneumatics doinker(2, false); //doinker mech 

// DRIVETRAIN SETTINGS
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors,
                              11.22,
                              lemlib::Omniwheel::NEW_325,
                              360,
                              8
);

// LATERAL PID CONTROLLER
lemlib::ControllerSettings linearController(7.2,    // proportional gain (kP)
                                            .95,    // integral gain (kI)
                                            33.7, // derivative gain (kD)
                                            0.07,  // anti windup
                                            0,    // small error range, in inches
                                            0,  // small error range timeout, in milliseconds
                                            0,    // large error range, in inches
                                            0,  // large error range timeout, in milliseconds
                                            0    // maximum acceleration (slew)
);

// ANGULAR PID CONTROLLER
lemlib::ControllerSettings angularController(5.25,  // proportional gain (kP)
                                             1, // integral gain (kI)
                                             44.8, // derivative gain (kD)
                                             2,   // anti windup
                                             0,     // small error range, in degrees
                                             0,     // small error range timeout, in milliseconds
                                             0,     // large error range, in degrees
                                             0,     // large error range timeout, in milliseconds
                                             0      // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3,    // joystick deadband out of 127
                                     10,   // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
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

void checkIntakeChain() {
    if (chain.get_voltage() > 0 && chain.get_actual_velocity() < 25)
    {
        chain.move_relative(-1, 600);
    }
    chain.move(110);
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors

    intake.set_brake_mode(pros::MotorBrake::coast);
    chain.set_brake_mode(pros::MotorBrake::coast);
    stakemech.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode_all(pros::MotorBrake::coast);
    rightMotors.set_brake_mode_all(pros::MotorBrake::coast);
    stakemech.set_zero_position(0);

    /*PID Tuning Setup
    chassis.setPose(0,0,0); // coordinates + heading to 0
    chassis.turnToHeading(120,3000);
    chassis.turnToHeading(0,3000);
    chassis.moveToPoint(0, 24, 5000); //forward 24 inches
    */
   pros::delay(500);
    while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }
    
}

void disabled() {} // disregard don't delete

void competition_initialize()
{
    // initialize sensors + autonomous selector here
}

void autonomous()
{
    // set chassis pose
    chassis.setPose(-147.73, 126.329, 330);
    chassis.moveToPoint(0, -27.5, 1000)


    while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }


    /*
    // start code here
    // score pre load
    intake.move(127);
    chain.move(110);
    pros::delay(600);
    // 1st quadrant
    chassis.moveToPoint(0, 14, 4000, {.maxSpeed = 80});
    chassis.turnToHeading(-90, 1000, {.maxSpeed = 90});
    mogomech.retract();
    chassis.moveToPoint(31, 18, 4000, {.forwards = false, .maxSpeed = 50}, false);
    // pick up mogo one
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
    chassis.moveToPose(30, 25, 90, 2500, {.forwards = false, .maxSpeed = 70});
    chassis.moveToPoint(46, 16, 4000, {.maxSpeed = 50});
    checkIntakeChain();
    chassis.turnToHeading(-22, 1000, {.maxSpeed = 70});
    chassis.moveToPoint(63, 1, 4000, {.forwards = false, .maxSpeed = 70}, false);
    // drop mogo 1
    mogomech.retract();

    // 4th quadrant
    chassis.moveToPose(20, 13, 90, 3000);
    chassis.moveToPoint(-27, 16.5, 4000, {.forwards = false, .maxSpeed = 55}, false);
    // pick up mogo 2
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
    chassis.moveToPose(-30, 22, -90, 2500, {.forwards = false, .maxSpeed = 70});
    chassis.moveToPoint(-48, 12, 4000, {.maxSpeed = 50});
    checkIntakeChain();
    chassis.turnToHeading(22, 1000, {.maxSpeed = 70});
    chassis.moveToPoint(-63, -4, 4000, {.forwards = false, .maxSpeed = 70}, false);
    // drop mogo 2
    mogomech.retract();
    // end code here
    pros::lcd::print(1, "Autonomous Routine Finished");
    */
}

void opcontrol()
{
    /*
    L1 - mogo
    L2 - set lady brown
    left - doinker
    R1 - intake up
    R2 - chain down
    Right Arrow - score lady brown
    Y - down lady brown
    */
   pros::delay(500);
    while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }
    
    bool last_L1_state = false;
    while (true)
    {
        //drivetrain
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        //intake 
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(-127);
            chain.move(127);
        } else {
            intake.move(0);
            chain.move(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(100);
            chain.move(-75);
        }
        //lady brown
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            stakemech.move_absolute(440, 90);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) { //move up
            if (stakemech.get_position() > 1750) {
                stakemech.move(-25);
            } else{
                stakemech.move(100);
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { //move down
            stakemech.move(-100);
        } else if (stakemech.get_position() > 1750) {
                stakemech.move(-25);
        } else {
            stakemech.move(0);
        }
        //mogomech
        bool current_L1_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        if (current_L1_state && !last_L1_state)
        {
            mogomech.toggle();
        }
        last_L1_state = current_L1_state;

        //doinker
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            doinker.toggle();
            pros::delay(300);
        }

        pros::delay(25);
    }
}
