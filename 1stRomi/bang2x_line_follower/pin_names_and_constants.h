#ifndef _PIN_NAMES_
#define _PIN_NAMES_
/**
 * This class will serve as a central point for all pin definitions
 * and constants.
 */
#define LINE_SENSOR_COUNT 3 //# how many line sensors we have
#define LINE_SENSOR_CALIBRATION_VALUE_COUNT 50 //# how many calibration values we want to calibrate on (be careful not to overflow the adder)
#define LINE_SENSOR_UPDATE_FREQUENCY 25 //# how fast we want our sensors updated.

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
 * Encoder pins
 */
#define RIGHT_ENCODER_XOR 7
#define RIGHT_ENCODER_PHASE_B 23
#define LEFT_ENCODER_XOR 8

/**
 * LED
 */
#define YELLOW_LED 13

#endif
