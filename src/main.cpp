#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
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
pros::Rotation horizontalEnc(18);                                                  // horitontal rotational sensor
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -0.5); // vertical tracking wheel
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

// VARIABLES 
int menuState = 0;
bool ringSide = false;
bool goalSide = false;
bool redSide = false;
bool blueSide = false;
bool skills = false;

// DRIVETRAIN SETTINGS
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.25, lemlib::Omniwheel::NEW_325, 360, 8);

// LATERAL PID CONTROLLER
lemlib::ControllerSettings linearController(2,  // proportional gain (kP)
                                            0,  // integral gain (kI)
                                            10, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches 1
                                            0, // small error range timeout, in milliseconds 100
                                            0, // large error range, in inches 3
                                            0, // large error range timeout, in milliseconds 500
                                            0 // maximum acceleration (slew) 20
);

// ANGULAR PID CONTROLLER
lemlib::ControllerSettings angularController(7,// proportional gain (kP)
                                             2,   // integral gain (kI) 2
                                             55.3, // derivative gain (kD) old: 6x
                                             .3,   // anti windup
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
// AUTON SELECTOR GUI
void print_menu() {
    if(menuState == 0) {
        pros::lcd::set_text(1, "1. Ring Side");
        pros::lcd::set_text(2, "2. Goal Side");
        pros::lcd::set_text(3, "3. Skills");
        pros::lcd::clear_line(4);
    }
    if(menuState == 1) {
        pros::lcd::set_text(1, "1. Red");
        pros::lcd::set_text(2, "2. Blue");
        pros::lcd::clear_line(3);
        pros::lcd::set_text(4, "3. Restart Configuration");

    }
    if(menuState == 2) {
        pros::lcd::set_text(1, "Current Configuration:");
        if(skills){
            pros::lcd::set_text(2, "Skills");

        } else {
            if(goalSide) {
                if(redSide) {
                    pros::lcd::set_text(2, "Red Goal Side");
                } else {
                pros::lcd::set_text(2, "Blue Goal Side");
                }
            } else {
                if(redSide) {
                    pros::lcd::set_text(2, "Red Ring Side");
                } else {
                    pros::lcd::set_text(2, "Blue Ring Side");
                }
            }
        }
        pros::lcd::clear_line(3);
        pros::lcd::set_text(4, "3. Restart Configuration");
    }
}
void on_b0() { // LEFT BUTTON
    if(menuState == 1) {
        redSide = true;
        menuState = 2;
    }
    if(menuState == 0) {
        ringSide = true;
        menuState = 1;
    }
    print_menu();
}
void on_b1() { // MIDDLE BUTTON
    if(menuState == 1) {
        blueSide = true;
        menuState = 2;
    }
    if(menuState == 0) {
        goalSide = true;
        menuState = 1;
    }
    print_menu();
}
void on_b2() { // RIGHT BUTTON
    if(menuState == 0) {
        skills = true;
        menuState = 2;
        print_menu();
    } else {
        redSide = false;
        blueSide = false;
        goalSide = false;
        ringSide = false;
        menuState = false;
        skills = false;
        print_menu();
    }
    
}

void initialize() {

    pros::lcd::initialize();
    /*
    print_menu();
    pros::lcd::register_btn0_cb(on_b0);
    pros::lcd::register_btn1_cb(on_b1);
    pros::lcd::register_btn2_cb(on_b2);
    */
    chassis.calibrate(); // calibrate sensors
    
    //PID Tuning Setup
    chassis.setPose(0,0,0); // coordinates + heading to 0
    //chassis.turnToHeading(45,3000);
    chassis.moveToPoint(0, 24, 3000);

    while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }

    // Intialize brake mode & postitions
    intake.set_brake_mode(pros::MotorBrake::coast);
    stakemech.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode_all(pros::MotorBrake::coast);
    rightMotors.set_brake_mode_all(pros::MotorBrake::coast);
    stakemech.set_zero_position(0); 
}

void autonomous() {
    
    if(skills) { // skills autonomous routine 
        chassis.setPose(0, 0, 0);
        //start code here 
        //score pre load
        intake.move(-127);
        intake.move(127);
        pros::delay(600);
        // 1st quadrant 
        chassis.moveToPoint(0, 6.75, 4000,{.minSpeed=60});
        chassis.turnToHeading(-90, 1000,{.maxSpeed = 90});
        mogomech.extend();
        chassis.moveToPoint(27, 13.25, 4000,{.forwards = false, .maxSpeed = 50}, false);
        //pick up mogo one 
        mogomech.retract(); 
        pros::delay(500);
        chassis.turnToHeading(0, 1000, {.maxSpeed = 60});
        chassis.moveToPoint(23.5, 42, 4000, {.maxSpeed = 70}, false);
        pros::delay(500);
        chassis.turnToHeading(56.5, 1000, {.maxSpeed = 60});

        chassis.moveToPoint(55.5, 61, 4000, {.maxSpeed = 70}); // very right top ring
        chassis.moveToPoint(45, 60, 4000, {.forwards = false, .maxSpeed = 55});
        chassis.turnToHeading(180, 1000, {.maxSpeed = 50});
        chassis.moveToPoint(48, 2, 4000, {.maxSpeed = 50}, false); // finish eating 3 rings
        pros::delay(1500);
        chassis.turnToHeading(50, 1500);
        chassis.moveToPoint(55.5,14,1500, {.maxSpeed = 80}); // pick up fourth ring
        pros::delay(1000);

        chassis.turnToHeading(-22, 1000,{.maxSpeed = 70}); // turn to face corner
        chassis.moveToPoint(60, 3, 1000, {.forwards = false}, false);
        //drop mogo 1
        intake.move(-127);
        mogomech.extend();
        pros::delay(500);

        //4th quadrant
        chassis.turnToHeading(-90, 1000, {}, false);
        chassis.moveToPose(20, 15, 90, 2000, {.minSpeed=50});
        intake.move(127);
        chassis.moveToPoint(-29, 15, 2000, {.forwards = false, .maxSpeed = 50}, false);
        //pick up mogo 2
        mogomech.retract();
        pros::delay(300);
        chassis.turnToHeading(0, 1000);
        chassis.moveToPoint(-23.5, 40, 4000, {.maxSpeed = 50}, false);
        pros::delay(500);
        chassis.turnToHeading(-56.5, 1000, {.maxSpeed = 50});

        chassis.moveToPoint(-54.5, 58, 4000, {.maxSpeed = 50}); // very left top ring
        chassis.moveToPoint(-44, 60, 4000, {.forwards = false, .maxSpeed = 55});
        chassis.turnToHeading(-180, 1000, {.maxSpeed = 50});
        chassis.moveToPoint(-46, 2, 4000, {.maxSpeed = 50}, false); // finish eating 3 rings
        pros::delay(1100);
        chassis.turnToHeading(-50, 900);
        chassis.moveToPoint(-55.5, 14, 1500, {.maxSpeed=70}); //pick up fourth ring
        pros::delay(1000);

        chassis.turnToHeading(22, 1000,{.maxSpeed = 70}); // turn to face corner
        chassis.moveToPoint(-60, 2, 4000, {.forwards = false, .maxSpeed = 70}, false);
        //drop mogo 2
        intake.move(-127);
        mogomech.extend();
        pros::delay(500);
        chassis.moveToPose(-47.5, 81,0,4000, {}, false);
        intake.brake();
        intake.move(-127);
        
        //other side
        mogomech.extend();
        chassis.setPose(0,0,0); //begin second half of auton (other side)
        chassis.turnToHeading(-150, 1000);
        chassis.moveToPoint(22.5, 34, 1500, {.forwards=false, .maxSpeed=60}, false);
        mogomech.retract();
        pros::delay(1000);
        chassis.turnToHeading(-225,2000);
        chassis.moveToPose(-9,45,-270,2000, {.forwards = false, .minSpeed=65}, false);
        pros::delay(1000);
        mogomech.extend();

        chassis.moveToPoint(41.5,39,2000, {.minSpeed=45});
        chassis.turnToHeading(180,1000);
        mogomech.retract();
        chassis.moveToPoint(50.5,45,2000, {.forwards = false}, false);
        pros::delay(500);
        intake.move(-127);
        intake.move(127);

    } else { // match autons
        if(ringSide) {
            if(redSide) { //red ring side (FINISHED)
                chassis.setPose(0, 0, 0);
                chassis.moveToPoint(0, -16.25, 1500, {.forwards=false, .minSpeed=90});
                mogomech.extend();
                chassis.moveToPoint(0, -26.25, 5000, {.forwards=false, .maxSpeed=30}, false);
                mogomech.retract(); //clamp mogo #1
                pros::delay(400);
                intake.move(-127);
                intake.move(120);
                chassis.turnToHeading(56.5, 1000, {.maxSpeed=50});
                chassis.moveToPoint(16, -21, 1500, {.minSpeed=90}); // ring 2
                chassis.swingToHeading(144, lemlib::DriveSide::RIGHT, 1000);
                chassis.moveToPoint(29, -28.5, 2000);
                chassis.moveToPoint(29.25, -18.5, 1000, {.forwards=false});
                chassis.turnToHeading(181.75, 1000);
                chassis.moveToPoint(26.75, -35, 2000);
                pros::delay(3000);
                intake.brake();
                mogomech.extend();
                chassis.swingToHeading(44, lemlib::DriveSide::RIGHT, 1500, {.maxSpeed=80});
                stakemech.move_relative(800, 127);
                chassis.moveToPose(0, -57, 0, 1500, {.forwards=false, .minSpeed=80});
            } else { //blue ring side (FINISHED)
                chassis.setPose(0, 0, 0);
                chassis.moveToPoint(0, -16.25, 1500, {.forwards=false, .minSpeed=90});
                mogomech.extend();
                chassis.moveToPoint(0, -26.25, 5000, {.forwards=false, .maxSpeed=30}, false);
                mogomech.retract(); //clamp mogo #1
                pros::delay(400);
                intake.move(-127); 
                intake.move(120);
                chassis.turnToHeading(-56.5, 1000, {.maxSpeed=50});
                chassis.moveToPoint(-16, -21, 1500, {.minSpeed=90}); // ring 2
                chassis.swingToHeading(-144, lemlib::DriveSide::LEFT, 1000);
                chassis.moveToPoint(-29, -28.5, 2000);
                chassis.moveToPoint(-29.25, -18.5, 1000, {.forwards=false});
                chassis.turnToHeading(-181.75, 1000);
                chassis.moveToPoint(-26.75, -35, 2000);
                pros::delay(3000);
                intake.brake();
                mogomech.extend();
                chassis.swingToHeading(-44, lemlib::DriveSide::LEFT, 1500, {.maxSpeed=80});
                stakemech.move_relative(800, 127);
                chassis.moveToPose(0, -57, 0, 1500, {.forwards=false, .minSpeed=80});
            }
        } else {
            if(redSide) { //red goal side (FINISHED)
                chassis.setPose(0,0,0);
                chassis.moveToPoint(0, -25, 2000, {.forwards=false, .minSpeed=85}, false);
                mogomech.extend();
                chassis.swingToHeading(-37, lemlib::DriveSide::RIGHT, 1000);
                chassis.moveToPoint(7, -43, 1000, {.forwards=false, .maxSpeed=50}, false);
                mogomech.retract();
                pros::delay(300);
                intake.move(-127);
                intake.move(120);
                pros::delay(500);
                chassis.moveToPose(10, -26.75, 30, 1500, {.minSpeed=90});
                chassis.turnToHeading(90, 1000);
                chassis.moveToPoint(12, -26.75, 500);
                doinker.extend();
                pros::delay(250);
                intake.brake();
                mogomech.extend();
                chassis.turnToHeading(-88.5, 1000, {}, false);
                doinker.retract();
                chassis.moveToPoint(31, -27.8, 1000, {.forwards=false, .maxSpeed=60}, false);
                mogomech.retract();
                pros::delay(750);
                chassis.moveToPose(52, -16, 22.5, 3000, {.maxSpeed=80}, false);
                intake.move(127);
                doinker.extend();
                pros::delay(500);
                chassis.turnToHeading(90, 1000, {.maxSpeed=70});
                doinker.retract();
                chassis.moveToPoint(66, -14, 1000, {.minSpeed=80});
                chassis.turnToHeading(200, 1000, {.maxSpeed=60}, false);
                chassis.moveToPoint(66, -22, 1000);
            } else { //blue goal side (FINISHED)
                chassis.setPose(0,0,0);
                chassis.moveToPoint(0, -25, 1000, {.forwards=false, .minSpeed=85}, false);
                mogomech.extend();
                chassis.swingToHeading(37, lemlib::DriveSide::RIGHT, 1000);
                chassis.moveToPoint(-7, -43, 1000, {.forwards=false, .maxSpeed=50}, false);
                mogomech.retract();
                pros::delay(300);
                intake.move(-127);
                intake.move(120);
                pros::delay(500);
                chassis.moveToPose(-10, -26.75, -30, 1500, {.minSpeed=90});
                chassis.turnToHeading(90, 1000);
                chassis.moveToPoint(-12, -26.75, 500);
                doinker.extend();
                pros::delay(250);
                intake.brake();
                mogomech.extend();
                chassis.turnToHeading(-88.5, 1000, {}, false);
                doinker.retract();
                chassis.moveToPoint(-31, -27.8, 1000, {.forwards=false, .maxSpeed=60}, false);
                mogomech.retract();
                pros::delay(750);
                intake.move(120);
                stakemech.move_relative(800, 127);
                chassis.moveToPoint(-40, -27.8, 2000);
            }
        }
    } 
    
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
