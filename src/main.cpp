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
#include <atomic>
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
                              8
);

// LATERAL PID CONTROLLER
lemlib::ControllerSettings linearController(7.2,  // proportional gain (kP)
                                            .95,  // integral gain (kI)
                                            33.7, // derivative gain (kD)
                                            0.07, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
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

int menu_state = 0;
bool ringside = false;
bool goalside = false;
bool redside = false;
bool blueside = false;

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

void checkIntakeChain() {
    if (chain.get_voltage() != 0 && chain.get_actual_velocity() < 50)
    {
        chain.move_relative(-1, 600);
    }
    chain.move(110);
}

void print_menu() {
    if(menu_state == 0) {
        pros::lcd::set_text(1, "1. Ringside");
        pros::lcd::set_text(2, "2. Goalside");
    }
    if(menu_state == 1) {
        pros::lcd::set_text(1, "1. Red Side");
        pros::lcd::set_text(2, "2. Blue Side");
    }
    if(menu_state == 2) {
        pros::lcd::set_text(1, "Current Configuration:");
        if(goalside) {
            if(redside) {
                pros::lcd::set_text(2, "Red Goalside");
            } else {
            pros::lcd::set_text(2, "Blue Goalside");
            }
        } else {
        if(redside) {
                pros::lcd::set_text(2, "Red Ringside");
            } else {
            pros::lcd::set_text(2, "Blue Ringside");
            }
        }
    }
    pros::lcd::set_text(4, "3. Reset");
}

void on_b0() {
    if(menu_state == 1) {
        redside = true;
        menu_state = 2;
    }
    if(menu_state == 0) {
        ringside = true;
        menu_state = 1;
    }
    print_menu();
}

void on_b1() {
    if(menu_state == 1) {
        blueside = true;
        menu_state = 2;
    }
    if(menu_state == 0) {
        goalside = true;
        menu_state = 1;
    }
    print_menu();
}

void on_b2() {
    redside = false;
    blueside = false;
    goalside = false;
    ringside = false;
    menu_state = false;
    print_menu();
}

void initialize() {

    pros::lcd::initialize();
    print_menu();
    pros::lcd::register_btn0_cb(on_b0);
    pros::lcd::register_btn1_cb(on_b1);
    pros::lcd::register_btn2_cb(on_b2);


    /*PID Tuning Setup
    chassis.setPose(0,0,0); // coordinates + heading to 0
    chassis.turnToHeading(120,3000);
    chassis.turnToHeading(0,3000);
    chassis.moveToPoint(0, 24, 5000); //forward 24 inches
    */
    chassis.calibrate(); // calibrate sensors

    /*while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }*/

    /*PID Tuning Setup
    chassis.setPose(0,0,0); // coordinates + heading to 0
    chassis.turnToHeading(120,3000);
    chassis.turnToHeading(0,3000);
    chassis.moveToPoint(0, 24, 5000); //forward 24 inches
    */

    // Intialize brake mode & postitions
    intake.set_brake_mode(pros::MotorBrake::coast);
    chain.set_brake_mode(pros::MotorBrake::coast);
    stakemech.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode_all(pros::MotorBrake::coast);
    rightMotors.set_brake_mode_all(pros::MotorBrake::coast);
    stakemech.set_zero_position(0);
}

void autonomous() {
    if(ringside) {
        if(redside) {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -16.25, 1500, {.forwards=false, .minSpeed=90});
            chassis.moveToPoint(0, -26.25, 5000, {.forwards=false, .maxSpeed=30}, false);
            mogomech.retract(); //clamp mogo #1
            pros::delay(400);
            intake.move(-127);
            chain.move(120);
            chassis.turnToHeading(56.5, 1000, {.maxSpeed=50});
            chassis.moveToPoint(16, -21, 1500, {.minSpeed=90}); // ring 2
            chassis.swingToHeading(144, lemlib::DriveSide::RIGHT, 1000);
            checkIntakeChain();
            chassis.moveToPoint(29, -28.5, 2000);
            checkIntakeChain();
            chassis.moveToPoint(29.25, -18.5, 1000, {.forwards=false});
            chassis.turnToHeading(181.75, 1000);
            checkIntakeChain();
            chassis.moveToPoint(26.75, -35, 2000);
            checkIntakeChain();
            pros::delay(3000);
            chain.brake();
            mogomech.extend();
            chassis.swingToHeading(44, lemlib::DriveSide::RIGHT, 1500, {.maxSpeed=80});
            stakemech.move_relative(800, 127);
            chassis.moveToPose(0, -57, 0, 1500, {.forwards=false, .minSpeed=80});
        } else {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -16.25, 1500, {.forwards=false, .minSpeed=90});
            chassis.moveToPoint(0, -26.25, 5000, {.forwards=false, .maxSpeed=30}, false);
            mogomech.retract(); //clamp mogo #1
            pros::delay(400);
            intake.move(-127);
            chain.move(120);
            chassis.turnToHeading(-56.5, 1000, {.maxSpeed=50});
            chassis.moveToPoint(-16, -21, 1500, {.minSpeed=90}); // ring 2
            chassis.swingToHeading(-144, lemlib::DriveSide::LEFT, 1000);
            checkIntakeChain();
            chassis.moveToPoint(-29, -28.5, 2000);
            checkIntakeChain();
            chassis.moveToPoint(-29.25, -18.5, 1000, {.forwards=false});
            chassis.turnToHeading(-181.75, 1000);
            checkIntakeChain();
            chassis.moveToPoint(-26.75, -35, 2000);
            checkIntakeChain();
            pros::delay(3000);
            chain.brake();
            mogomech.extend();
            chassis.swingToHeading(-44, lemlib::DriveSide::LEFT, 1500, {.maxSpeed=80});
            stakemech.move_relative(800, 127);
            chassis.moveToPose(0, -57, 0, 1500, {.forwards=false, .minSpeed=80});
        }
    } else {
        if(redside) {

        } else {

        }
    }
    /*
    0-100 = Red ring side
    100-200 = Red goal side
    200-300 = Blue goal side
    300-410 = Blue ring side
    */
    
    if (0 < programSelector.get_value()/10.0 <= 100.0) { // red ring side (FINISHED)
        /*chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -16.25, 1500, {.forwards=false, .minSpeed=90});
        chassis.moveToPoint(0, -26.25, 5000, {.forwards=false, .maxSpeed=30}, false);
        mogomech.retract(); //clamp mogo #1
        pros::delay(400);
        intake.move(-127);
        chain.move(120);
        chassis.turnToHeading(56.5, 1000, {.maxSpeed=50});
        chassis.moveToPoint(16, -21, 1500, {.minSpeed=90}); // ring 2
        chassis.swingToHeading(144, lemlib::DriveSide::RIGHT, 1000);
        checkIntakeChain();
        chassis.moveToPoint(29, -28.5, 2000);
        checkIntakeChain();
        chassis.moveToPoint(29.25, -18.5, 1000, {.forwards=false});
        chassis.turnToHeading(181.75, 1000);
        checkIntakeChain();
        chassis.moveToPoint(26.75, -35, 2000);
        checkIntakeChain();
        pros::delay(3000);
        chain.brake();
        mogomech.extend();
        chassis.swingToHeading(44, lemlib::DriveSide::RIGHT, 1500, {.maxSpeed=80});
        stakemech.move_relative(800, 127);
        chassis.moveToPose(0, -57, 0, 1500, {.forwards=false, .minSpeed=80});
        */
    } else if (100.0 < programSelector.get_value()/10.0 <= 200.0) { //red goal side
        
    } else if (200.0 < programSelector.get_value()/10.0 <= 300.0) { //blue goal side

    } else if (300.0 < programSelector.get_value()/10.0 <= 410.0) { //blue ring side (FINISHED)
        /*chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, -16.25, 1500, {.forwards=false, .minSpeed=90});
        chassis.moveToPoint(0, -26.25, 5000, {.forwards=false, .maxSpeed=30}, false);
        mogomech.retract(); //clamp mogo #1
        pros::delay(400);
        intake.move(-127);
        chain.move(120);
        chassis.turnToHeading(-56.5, 1000, {.maxSpeed=50});
        chassis.moveToPoint(-16, -21, 1500, {.minSpeed=90}); // ring 2
        chassis.swingToHeading(-144, lemlib::DriveSide::LEFT, 1000);
        checkIntakeChain();
        chassis.moveToPoint(-29, -28.5, 2000);
        checkIntakeChain();
        chassis.moveToPoint(-29.25, -18.5, 1000, {.forwards=false});
        chassis.turnToHeading(-181.75, 1000);
        checkIntakeChain();
        chassis.moveToPoint(-26.75, -35, 2000);
        checkIntakeChain();
        pros::delay(3000);
        chain.brake();
        mogomech.extend();
        chassis.swingToHeading(-44, lemlib::DriveSide::LEFT, 1500, {.maxSpeed=80});
        stakemech.move_relative(800, 127);
        chassis.moveToPose(0, -57, 0, 1500, {.forwards=false, .minSpeed=80});
        */
    }    



}

// DRIVER CODE UPDATED & FINISHED FOR SUPERNOVA 11/8/24
void opcontrol() {
    /*
    L1 - mogo
    L2 - set lady brown (hold)
    R1 - intake up
    R2 - chain down
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
            chain.move(127);
        } else {
            intake.move(0);
            chain.move(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(100);
            chain.move(-75);
        }
        // lady brown
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            stakemech.move_absolute(440, 90);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        { // move up
            if (stakemech.get_position() > 1750) {
                stakemech.move(-25);
            } else {
                stakemech.move(100);
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { // move down
            stakemech.move(-100);
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
