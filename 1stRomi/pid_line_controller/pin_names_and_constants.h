#ifndef _PIN_NAMES_
#define _PIN_NAMES_
/**
 * This class will serve as a central point for all pin definitions
 * and constants.
 */
#define LINE_SENSOR_COUNT 3 //# how many line sensors we have
#define LINE_SENSOR_CALIBRATION_VALUE_COUNT 50 //# how many calibration values we want to calibrate on (be careful not to overflow the adder)
#define LINE_SENSOR_UPDATE_FREQUENCY 25 //# how fast we want our sensors updated.
#define MAX_CALLBACKS_FOR_TIMER 2 //# how many callbacks can we have for timer3, which also drives line sensor.

/**
 * Line sensor pins
 */
#define LINE_LEFT_PIN A4
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A2

/**
 * Motor control pins
 */
#define RIGHT_MOTOR_DIR 15
#define LEFT_MOTOR_DIR 16
#define RIGHT_MOTOR_RUN 9
#define LEFT_MOTOR_RUN 10
#define MOTOR_FORWARD LOW
#define MOTOR_BACKWARD HIGH

/**
 * Scheduler time constants
 */
#define MOTOR_PID_UPDATE_TIME 10 //# update time in ms for PID controller.
#define REPORT_TIME 100 //# update time in ms for PID controller.
#define HEADING_PID_UPDATE_TIME 25 //# update time in ms for PID controller.
#define KINEMATICS_UPDATE_TIME 25 //# update time in ms for kinematics data.

/**
 * Encoder pins
 */
#define RIGHT_ENCODER_XOR 7
#define RIGHT_ENCODER_PHASE_B 23
#define LEFT_ENCODER_XOR 8
#define USECONDS_IN_1_SECOND 1000000

/**
 * Kinematics
 */
#define PULSES_PER_METER 1541   //# how many encoder pulses are in a meter
#define PULSES_PER_REV 360.0    //# how many encoder pulses are in a full revolution of the wheel.
#define WHEEL_DIAMETER_MM 70.0  //# Wheel diameter is 70mm as per spec: https://www.pololu.com/product/1428
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER_MM * PI)        //# wheel circumference
#define MM_PER_PULSE (WHEEL_CIRCUMFERENCE / PULSES_PER_REV) //# how many mm of distance does 1 pulse mean.

#define CHASSIS_WIDTH 149.0    //# The width of chassis (including rims of the wheels) is 149 mm according to: https://www.pololu.com/docs/0J68/6
#define WHEEL_WIDTH 6.6        //# Width of the wheel is 6.6mm according to https://www.pololu.com/file/0J1708/pololu-wheel-dimensions.pdf

#define WALK_HOME_SPEED 300    //# Speed to use when walking home (back to origin).

/**
 * Romi wheels are then separated (middle of the wheel to middle of the wheel) by this distance
 */
#define WHEEL_SEPARATION (CHASSIS_WIDTH - WHEEL_WIDTH)

/**
 * LED
 */
#define YELLOW_LED 13

#endif
