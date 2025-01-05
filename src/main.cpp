#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/core/lv_disp.h"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/lv_btnmatrix.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/apix.h"
#include <cstring>
#include <string>
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/device.hpp"
#include "pros/optical.h"
#include "pros/vision.hpp"
#include <atomic>
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
pros::adi::Pneumatics mogomech(1, true); // mobile goal mech
pros::adi::Pneumatics doinker(2, false); // doinker mech
pros::adi::Pneumatics intakeLift(3, false); // intake lift 

// DRIVETRAIN SETTINGS
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.25, lemlib::Omniwheel::NEW_325, 360, 8);

// LATERAL PID CONTROLLER
lemlib::ControllerSettings linearController(8.5,  // proportional gain (kP)
                                            0.6,  // integral gain (kI)
                                            40, // derivative gain (kD)
                                            0.5, // anti windup
                                            1, // small error range, in inches 1
                                            100, // small error range timeout, in milliseconds 100
                                            3, // large error range, in inches 3
                                            500, // large error range timeout, in milliseconds 500
                                            0 // maximum acceleration (slew) 20
);

// ANGULAR PID CONTROLLER
lemlib::ControllerSettings angularController(8,// proportional gain (kP)
                                             1,   // integral gain (kI) 2
                                             67, // derivative gain (kD) old: 6x
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

static lv_obj_t *auton_sel_screen = nullptr;
static lv_obj_t *match_sel_screen = nullptr;
static lv_obj_t *confirmation_screen = nullptr;
static lv_obj_t *current_screen = nullptr;

static lv_obj_t *match_btnm = nullptr;
static lv_obj_t *auton_btnm = nullptr;

static const char * auton_sel = nullptr;
static const char * team = nullptr;
static const char * side = nullptr;
static const char * wp = nullptr;

static lv_obj_t *auton_coordinates_label = nullptr;
static lv_obj_t *match_coordinates_label = nullptr;
static lv_obj_t *confirmation_coordinates_label = nullptr;
static std::atomic<bool> should_update_coordinates(true);

static void create_confirmation_screen(void);

static const char * auton_sel_map[] = {"Match Auton", "Skills Auton", ""};
static const char * match_sel_map[] = {"Red Side", "Blue Side", "\n",
                                      "Ring Side", "Goal Side", "\n",
                                      "Win Point", ""};

static void event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
    if (obj == NULL) {
        LV_LOG_ERROR("Object is NULL\n");
        return;
    }

    if(code == LV_EVENT_CLICKED) {
        lv_obj_t * child = lv_obj_get_child(obj, 0);

        if (child == NULL) {
            LV_LOG_ERROR("Button has no child\n");
            return;
        }

        LV_LOG_USER("Child class: %p\n", lv_obj_get_class(child));

        // Check if the child is a label
        if (lv_obj_get_class(child) == &lv_label_class) {
            const char * label = lv_label_get_text(child);

            if (label == NULL) {
                LV_LOG_ERROR("Label text is NULL\n");
                return;
            }

            LV_LOG_USER("Label text: %s\n", label);

            if(strcmp(label, "Confirm") == 0) {
                auton_sel = lv_btnmatrix_get_btn_text(auton_btnm, lv_btnmatrix_get_selected_btn(auton_btnm));

                if(lv_btnmatrix_has_btn_ctrl(match_btnm, 0, LV_BTNMATRIX_CTRL_CHECKED)) {
                  team = "Red";
                } else if (lv_btnmatrix_has_btn_ctrl(match_btnm, 1, LV_BTNMATRIX_CTRL_CHECKED)) {
                  team = "Blue";
                }

                if(lv_btnmatrix_has_btn_ctrl(match_btnm, 2, LV_BTNMATRIX_CTRL_CHECKED)) {
                  side = "Ring Side";
                } else if (lv_btnmatrix_has_btn_ctrl(match_btnm, 3, LV_BTNMATRIX_CTRL_CHECKED)) {
                  side = "Goal Side";
                }

                if(lv_btnmatrix_has_btn_ctrl(match_btnm, 4, LV_BTNMATRIX_CTRL_CHECKED)) {
                  wp = "Yes";
                } else {
                  wp = "No";
                }
                
                if (team == nullptr || side == nullptr || wp == nullptr) {
                    return;
                }
                
                create_confirmation_screen();
                lv_scr_load_anim(confirmation_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
                current_screen = confirmation_screen;
            }
        }
    }
    if(code == LV_EVENT_VALUE_CHANGED) {
        uint32_t id = lv_btnmatrix_get_selected_btn(obj);
        const char * txt = lv_btnmatrix_get_btn_text(obj, id);
        LV_LOG_USER("%s was pressed\n", txt);

        if(strcmp(txt, "Red Side") == 0) {
          lv_btnmatrix_clear_btn_ctrl(obj, 1, LV_BTNMATRIX_CTRL_CHECKED);
        }

        if(strcmp(txt, "Blue Side") == 0) {
          lv_btnmatrix_clear_btn_ctrl(obj, 0, LV_BTNMATRIX_CTRL_CHECKED);
        }

        if(strcmp(txt, "Ring Side") == 0) {
          lv_btnmatrix_clear_btn_ctrl(obj, 3, LV_BTNMATRIX_CTRL_CHECKED);
        }

        if(strcmp(txt, "Goal Side") == 0) {
          lv_btnmatrix_clear_btn_ctrl(obj, 2, LV_BTNMATRIX_CTRL_CHECKED);
        }

        if(strcmp(txt, "Match Auton") == 0) {
            lv_scr_load_anim(match_sel_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
            current_screen = match_sel_screen;
        }

        if(strcmp(txt, "Skills Auton") == 0) {
            auton_sel = "Skills Auton";
            team = "N/A";
            side = "N/A";
            wp = "N/A";
            create_confirmation_screen();

            lv_scr_load_anim(confirmation_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
            current_screen = confirmation_screen;
        }
    }
}

static void create_auton_sel_screen() {
    auton_sel_screen = lv_obj_create(NULL);

    auton_btnm = lv_btnmatrix_create(auton_sel_screen);
    lv_obj_set_size(auton_btnm, 400, 170);
    lv_btnmatrix_set_map(auton_btnm, auton_sel_map);
    lv_obj_align(auton_btnm, LV_ALIGN_CENTER, 0, -30);
    lv_obj_add_event_cb(auton_btnm, event_handler, LV_EVENT_ALL, NULL);

    auton_coordinates_label = lv_label_create(auton_sel_screen);
    lv_obj_align(auton_coordinates_label, LV_ALIGN_BOTTOM_LEFT, 10, -10);
}

static void create_match_sel_screen() {
    match_sel_screen = lv_obj_create(NULL);

    match_btnm = lv_btnmatrix_create(match_sel_screen);
    lv_obj_set_size(match_btnm, 400, 170);
    lv_btnmatrix_set_map(match_btnm, match_sel_map);

    // Make first 5 buttons checkable
    for (int i = 0; i < 5; i++) {
        lv_btnmatrix_set_btn_ctrl(match_btnm, i, LV_BTNMATRIX_CTRL_CHECKABLE);
    }

    lv_obj_align(match_btnm, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_add_event_cb(match_btnm, event_handler, LV_EVENT_ALL, NULL);

    lv_obj_t * confirm_btn = lv_btn_create(match_sel_screen);
    lv_obj_t * confirm_btn_label = lv_label_create(confirm_btn);
    lv_label_set_text(confirm_btn_label, "Confirm");
    lv_obj_set_style_bg_color(confirm_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_align(confirm_btn, LV_ALIGN_CENTER, 0, 100);
    lv_obj_add_event_cb(confirm_btn, event_handler, LV_EVENT_ALL, NULL);

    match_coordinates_label = lv_label_create(match_sel_screen);
    lv_obj_align(match_coordinates_label, LV_ALIGN_BOTTOM_LEFT, 10, -10);
}


void lvgl_display_task_fn(void* param) {
    while (true) {
        lv_task_handler();

        if (auton_coordinates_label != nullptr && should_update_coordinates) {
            char coord_text[100];
            snprintf(coord_text, sizeof(coord_text),
                    "X: %.3f\nY: %.3f\nHeading: %.3f°",
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            lv_label_set_text(auton_coordinates_label, coord_text);
        }

        if (match_coordinates_label != nullptr && should_update_coordinates) {
            char coord_text[100];
            snprintf(coord_text, sizeof(coord_text),
                    "X: %.3f\nY: %.3f\nHeading: %.3f°",
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            lv_label_set_text(match_coordinates_label, coord_text);
        }

        if (confirmation_coordinates_label != nullptr && should_update_coordinates) {
            char coord_text[100];
            snprintf(coord_text, sizeof(coord_text),
                    "X: %.3f\nY: %.3f\nHeading: %.3f°",
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            lv_label_set_text(confirmation_coordinates_label, coord_text);
        }

        pros::delay(10);
    }
}

static void create_confirmation_screen() {
    confirmation_screen = lv_obj_create(NULL);

    char text_buffer[200];
    snprintf(text_buffer, sizeof(text_buffer), "%s\nTeam: %s\nSide: %s\nWP: %s\n", auton_sel, team, side, wp);

    lv_obj_t * confirm_label = lv_label_create(confirmation_screen);
    lv_label_set_text(confirm_label, text_buffer);
    lv_obj_align(confirm_label, LV_ALIGN_CENTER, 0, 0);

    confirmation_coordinates_label = lv_label_create(confirmation_screen);
    lv_obj_align(confirmation_coordinates_label, LV_ALIGN_BOTTOM_LEFT, 10, -10);
}

void initialize_display() {
    lv_init();
    
    // Create screens
    create_auton_sel_screen();
    create_match_sel_screen();
    
    lv_scr_load(auton_sel_screen);
    current_screen = auton_sel_screen;
    
    pros::Task lvgl_task(lvgl_display_task_fn, nullptr, "LVGL Task");
}

void initialize() {
    chassis.calibrate();
    
    initialize_display();
    
    
    /*PID Tuning Setup
    chassis.setPose(0,0,0); // coordinates + heading to 0
    chassis.turnToHeading(90,3000);
    chassis.moveToPoint(0, 24, 3000);

    while(1) {
        pros::lcd::print(1, "%f Heading", chassis.getPose().theta);
        pros::lcd::print(2, "%f X Coordinate", chassis.getPose().x);
        pros::lcd::print(3, "%f Y Coordinate", chassis.getPose().y);
        pros::delay(500);
    }
    */

    // Intialize brake mode & postitions
    intake.set_brake_mode(pros::MotorBrake::coast);
    stakemech.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode_all(pros::MotorBrake::coast);
    rightMotors.set_brake_mode_all(pros::MotorBrake::coast);
    stakemech.set_zero_position(0); 
    
}

void autonomous() {



    if (strcmp(auton_sel, "Match Auton") == 0) {

        if (team == nullptr || side == nullptr || wp == nullptr) {
            return;
        }

        if (strcmp(team, "Red") == 0) {
            if (strcmp(side, "Goal Side") == 0) {
                if (strcmp(wp, "Yes") == 0) {

                    // RED GOAL SIDE WITH WIN POINT

                } else {

                    // RED GOAL SIDE NO WIN POINT

                }
            } else {
                if (strcmp(wp, "Yes") == 0) {
                    
                    // RED RING SIDE WITH WIN POINT

                } else {

                    // RED RING SIDE NO WIN POINT

                }
            }
        } else {
            if (strcmp(side, "Goal Side") == 0) {
                if (strcmp(wp, "Yes") == 0) {

                    // BLUE GOAL SIDE WITH WIN POINT

                } else {

                    // BLUE GOAL SIDE NO WIN POINT

                }
            } else {
                if (strcmp(wp, "Yes") == 0) {
                    
                    // BLUE RING SIDE WITH WIN POINT

                } else {

                    // BLUE RING SIDE NO WIN POINT

                }
            }
        }
    } else {

        // SKILLS AUTON

    }






    //red ring
    chassis.setPose(0, 0 , 0);
    mogomech.extend();
    chassis.moveToPoint(0, -32, 3000, {.forwards=false, .maxSpeed=70});
    mogomech.retract(); //clamp mogo #1
    pros::delay(300);
    chassis.turnToHeading(130, 1000); //async??
    pros::delay(300);
    intake.move(127);
    chassis.moveToPoint(14, -45, 2000, {.maxSpeed=70});
    chassis.swingToHeading(90, DriveSide::RIGHT, 1500, {AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(34, -47, 2000, {.maxSpeed=70});
    chassis.turnToHeading(-30, 1500);
    chassis.moveToPoint(28, -30, 3000); //intake but dont score 4th ring
    intake.brake(); 
    mogomech.extend(); //release mogo mech 
    chassis.moveToPoint(-24, 8, 3000);
    chassis.turnToHeading(180, 1500);
    chassis.moveToPoint(-36, 14, 3000, {.forwards=false, .maxSpeed=50}, false); 
    intake.move(127); //score alliance stake 
    pros::delay(500);
    chassis.moveToPoint(-24, -34, 2000);
    //engage stakemech to touch pole
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
            intake.move(127);
        } else {
            intake.move(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-100);
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