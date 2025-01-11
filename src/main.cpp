#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/core/lv_disp.h"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/extra/widgets/meter/lv_meter.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/lv_btnmatrix.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/apix.h"
#include <cstdint>
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

// MOTORS
pros::MotorGroup leftMotors({-19, -3, 2}, pros::MotorGearset::blue);                      // front, top, bottom (left)
pros::MotorGroup rightMotors({11, 9, -8}, pros::MotorGearset::blue);                      // front, top, bottom (right)
pros::Motor stage1(15, pros::MotorGears::blue, pros::v5::MotorUnits::rotations);           // stage1
pros::Motor stage2(1, pros::MotorGears::blue, pros::v5::MotorUnits::rotations);           // stage2
pros::Motor stakemech(12, pros::MotorGears::green, pros::v5::MotorEncoderUnits::counts); // lady brown

// PNEUMATICS
pros::adi::Pneumatics mogomech(1, true); // mobile goal mech
pros::adi::Pneumatics doinker(2, false); // doinker mech
pros::adi::Pneumatics intakeLift(3, false); // intake lift 

// DRIVETRAIN SETTINGS
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.25, lemlib::Omniwheel::NEW_325, 450, 8);

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

lv_obj_t * speed_meter = nullptr;
lv_meter_indicator_t * speed_indic = nullptr;
bool speed_meter_initialized = false;

double prev_x = 0;
double prev_y = 0;
uint32_t prev_time = 0;

lv_obj_t * heading_meter;
lv_meter_indicator_t * indic;
static bool meter_initialized = false;

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
                auton_sel = "Match Auton";

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
    lv_obj_set_size(auton_btnm, 460, 180);
    lv_btnmatrix_set_map(auton_btnm, auton_sel_map);
    lv_obj_align(auton_btnm, LV_ALIGN_CENTER, 0, -12);
    lv_obj_add_event_cb(auton_btnm, event_handler, LV_EVENT_ALL, NULL);

    auton_coordinates_label = lv_label_create(auton_sel_screen);
    lv_obj_align(auton_coordinates_label, LV_ALIGN_BOTTOM_LEFT, 10, -10);
}

static void create_match_sel_screen() {
    match_sel_screen = lv_obj_create(NULL);

    match_btnm = lv_btnmatrix_create(match_sel_screen);
    lv_obj_set_size(match_btnm, 460, 170);
    lv_btnmatrix_set_map(match_btnm, match_sel_map);

    // Make first 5 buttons checkable
    for (int i = 0; i < 5; i++) {
        lv_btnmatrix_set_btn_ctrl(match_btnm, i, LV_BTNMATRIX_CTRL_CHECKABLE);
    }

    lv_obj_align(match_btnm, LV_ALIGN_TOP_MID, 0, 18);
    lv_obj_add_event_cb(match_btnm, event_handler, LV_EVENT_ALL, NULL);

    lv_obj_t * confirm_btn = lv_btn_create(match_sel_screen);
    lv_obj_t * confirm_btn_label = lv_label_create(confirm_btn);
    lv_label_set_text(confirm_btn_label, "Confirm");
    lv_obj_set_style_bg_color(confirm_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_align(confirm_btn, LV_ALIGN_CENTER, 125, 95);
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
                    "X: %.3f     Y: %.3f     Heading: %.3f°",
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            lv_label_set_text(auton_coordinates_label, coord_text);
        }

        if (match_coordinates_label != nullptr && should_update_coordinates) {
            char coord_text[100];
            snprintf(coord_text, sizeof(coord_text),
                    "X: %.3f     Y: %.3f     Heading: %.3f°",
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            lv_label_set_text(match_coordinates_label, coord_text);
        }

        if (confirmation_coordinates_label != nullptr && should_update_coordinates) {
            char coord_text[100];
            snprintf(coord_text, sizeof(coord_text),
                    "X: %.3f     Y: %.3f     Heading: %.3f°",
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
            lv_label_set_text(confirmation_coordinates_label, coord_text);
        }
        
        int heading = (int32_t)chassis.getPose().theta;

        if (heading < 0) {
            heading += 360;
        }

        if (heading > 360) {
            heading -= 360;
        }

        if (meter_initialized) {
            lv_meter_set_indicator_value(heading_meter, indic, heading);
        }
        
        if (speed_meter_initialized) {
            // Get current position and time
            double current_x = chassis.getPose().x;
            double current_y = chassis.getPose().y;
            uint32_t current_time = pros::millis();
            
            // Calculate distance traveled
            double dx = fabs(current_x - prev_x);
            double dy = fabs(current_y - prev_y);
            double distance = sqrt(dx*dx + dy*dy);
            
            
            // Calculate time elapsed in seconds
            double dt = (current_time - prev_time) / 1000.0;
            
            // Calculate speed in inches per second
            double speed = (dt > 0) ? (distance / dt) : 0;
            
            // Update the speed meter
            lv_meter_set_indicator_value(speed_meter, speed_indic, (int32_t)speed);
            
            // Update previous values
            prev_x = current_x;
            prev_y = current_y;
            prev_time = current_time;
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

    // Create heading meter
    heading_meter = lv_meter_create(confirmation_screen);
    lv_obj_align(heading_meter, LV_ALIGN_LEFT_MID, 30, -20);
    lv_meter_scale_t * heading_scale = lv_meter_add_scale(heading_meter);
    indic = lv_meter_add_needle_line(heading_meter, heading_scale, 4, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_scale_range(heading_meter, heading_scale, 0, 360, 360, 270);
    meter_initialized = true;

    lv_obj_t * heading_label = lv_label_create(heading_meter);
    lv_label_set_text(heading_label, "Heading");
    lv_obj_align(heading_label, LV_ALIGN_LEFT_MID, 15, 45);

    // Create speed meter
    speed_meter = lv_meter_create(confirmation_screen);
    lv_obj_align(speed_meter, LV_ALIGN_RIGHT_MID, -30, -20);
    lv_meter_scale_t * speed_scale = lv_meter_add_scale(speed_meter);
    speed_indic = lv_meter_add_needle_line(speed_meter, speed_scale, 4, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_scale_range(speed_meter, speed_scale, 0, 50, 150, 270); 

    lv_obj_t * speed_label = lv_label_create(speed_meter);
    lv_label_set_text(speed_label, "Speed");
    lv_obj_align(speed_label, LV_ALIGN_RIGHT_MID, -20, 45);
    
    
    // Initialize speed tracking variables
    prev_x = chassis.getPose().x;
    prev_y = chassis.getPose().y;
    prev_time = pros::millis();
    
    speed_meter_initialized = true;
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


const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0, 325, 2000};
int currState = 0;
int target = 0;

void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}

void liftControl() {
    double kp = 1;
    double error = target - stakemech.get_position();
    double velocity = kp * error;
    stakemech.move(velocity);
}

void initialize() {
    chassis.calibrate();
    initialize_display();

    pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });

    
    /*PID Tuning Setup
    chassis.setPose(0,0,0); // coordinates + heading to 0
    chassis.turnToHeading(90,3000);
    chassis.moveToPoint(0, 24, 3000);
    */

    // Intialize brake mode & postitions
    stage1.set_brake_mode(pros::MotorBrake::coast);
    stage2.set_brake_mode(pros::MotorBrake::coast);
    stakemech.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.set_brake_mode_all(pros::MotorBrake::coast);
    rightMotors.set_brake_mode_all(pros::MotorBrake::coast);
    stakemech.set_zero_position(0); 

    // RED RING SIDE WITH WIN POINT
    chassis.setPose(0, 0 , 0);
    mogomech.retract();
    chassis.moveToPoint(0, -32, 3000, {.forwards=false, .maxSpeed=70}, false);
    mogomech.extend(); //clamp mogo #1
    pros::delay(200);
    chassis.turnToHeading(130, 1000); //async??
    pros::delay(200);
    stage1.move(127);
    stage2.move(100);
    chassis.moveToPoint(6, -41, 2000, {.maxSpeed=70});
    chassis.turnToHeading(90, 1500, {AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(28, -47, 2000, {.maxSpeed=70});
    chassis.turnToHeading(-28, 1000);
    chassis.moveToPoint(28, -28, 3000, {}, false); //intake but dont score 4th ring
    mogomech.retract(); //release mogo mech 
    chassis.moveToPoint(-21, 8, 3000);
    stage2.brake();
    chassis.turnToHeading(180, 1500);
    chassis.moveToPoint(-32, 15, 3000, {.forwards=false, .maxSpeed=50}, false); 
    stage2.move(100); //score alliance stake 
    pros::delay(800);
    //stakemech
    chassis.moveToPoint(-28, -25, 2000);
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
                    chassis.setPose(0, 0 , 0);
                    mogomech.extend();
                    chassis.moveToPoint(0, -32, 3000, {.forwards=false, .maxSpeed=70}, false);
                    mogomech.retract(); //clamp mogo #1
                    pros::delay(300);
                    chassis.turnToHeading(130, 1000); //async??
                    pros::delay(300);
                    stage1.move(127);
                    chassis.moveToPoint(11, -42, 2000, {.maxSpeed=70});
                    chassis.swingToHeading(90, DriveSide::RIGHT, 1500, {AngularDirection::CCW_COUNTERCLOCKWISE});
                    chassis.moveToPoint(31, -47, 2000, {.maxSpeed=70});
                    chassis.swingToHeading(-30, DriveSide::LEFT, 1500);
                    chassis.moveToPoint(28, -30, 3000, {}, true); //intake but dont score 4th ring
                    stage1.brake(); 
                    mogomech.extend(); //release mogo mech 
                    chassis.moveToPoint(-24, 8, 3000);
                    chassis.turnToHeading(180, 1500);
                    chassis.moveToPoint(-33, 18, 3000, {.forwards=false, .maxSpeed=50}, false); 
                    stage1.move(127); //score alliance stake 
                    pros::delay(500);
                    chassis.moveToPoint(-24, -34, 2000);
                    //engage stakemech to touch pole

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
}

// DRIVER CODE UPDATED & FINISHED FOR SUPERNOVA 11/8/24
void opcontrol() {
    /*
    Mogomech - L1
    Lady brown - L2
    Intake - R1 (in), R2 (out), (raise)
    Doinker - Right Arrow
    Drivetrain - left/right joysticks
    */

    bool stakeMechSet = false;
    bool last_L1_state = false;
    while (true)
    {
        // drivetrain
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        // intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            stage1.move(100);
            stage2.move(95);
        } else {
            stage1.move(0);
            stage2.move(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            stage1.move(-95);
            stage2.move(-100);
        }
        // lady brown
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // set
            nextState();
            pros::delay(200);
        }
        
        // mogomech
        bool current_L1_state = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        if (current_L1_state && !last_L1_state) {
            mogomech.toggle();
        }
        last_L1_state = current_L1_state;
        // doinker
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            doinker.toggle();
            pros::delay(200);
        }
        //lifted intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            intakeLift.toggle();
            pros::delay(200);
        }

        pros::delay(25); //controller debounce delay
    }
}