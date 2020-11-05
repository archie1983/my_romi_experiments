#ifndef _MOTOR_
#define _MOTOR_
#include <Arduino.h>

#include "threshold_callback.h"
#include "encoder.h"
#include "pid.h"

/**
 * We'll need to differentiate our motors so that we know which encoder to use.
 * For that we'll use these defines, which we'll pass in the constructor and
 * save in some variable and later we'll look up the correct encoder by the 
 * value of that variable.
 */
#define RIGHT_MOTOR 1
#define LEFT_MOTOR 2

/**
 * This class will control motor movement.
 */
class Motor : public ThresholdCallback {
  public:
    /**
     * Runs the motor forward for the given time amount with the given power
     */
    void goForwardForGivenTimeAtGivenPower(unsigned int ms, byte power);

    /**
     * Runs the motor forward for the given time amount with the given power
     */
    void goBackwardForGivenTimeAtGivenPower(unsigned int ms, byte power);

    /**
     * Turns the motor at a constant speed controlled by PID for a given amount
     * of ms.
     */
    void goForGivenTimeAtGivenSpeed_PID(unsigned int ms, int motor_speed);

    /**
     * Turns the motor at a constant speed controlled by PID for a given amount
     * of encoder counts.
     */
    void goForGivenClicksAtGivenSpeed_PID(long clicks, int motor_speed);

    /**
     * Turns the motor at a constant speed controlled by PID.
     */
    void goAtGivenSpeed_PID(int motor_speed);

    /**
     * Performs the PID update and compensates motor power accordingly to the
     * PID output.
     */
    void updateMotorPIDcontroller(int current_motor_speed);

    /**
     * Accessor for the last requested motor speed.
     */
    int getLastRequestedMotorSpeed_PID();

    /**
     * Applies a coefficient calculated by the heading PID to the speed that 
     * needs to be requested from the motor PIDs.
     */
    void updateRequestedSpeedByAFactor_PID(float correction_factor);

    /**
     * Sets the target speed for the motro PID.
     */
    void setRequestedSpeed_PID(int new_speed);
    
    /**
     * Runs the motor forward for for the specified amount of encoder counts at 35 PWM power
     */
    void goForwardByCounts(unsigned int counts);

    /**
     * Runs the motor backwards for for the specified amount of encoder counts at 35 PWM power
     */
    void goBackwardByCounts(unsigned int counts);

    /**
     * A convenience function that takes negative or positive power and based on that
     * either calls goForwardByCounts or goBackwardByCounts
     */
    void moveByCounts(unsigned int counts, int power);
  
    /**
     * Runs the motor forward for for the specified amount of encoder counts at the specified PWM power
     */
    void goForwardByCounts(unsigned int counts, byte power);

    /**
     * Runs the motor backwards for for the specified amount of encoder counts at the specified PWM power
     */
    void goBackwardByCounts(unsigned int counts, byte power);

    /**
     * Stops motor and cancels any previous threshold that was sent to the encoder.
     */
    void stopMotorAndCancelPreviousInstruction();

    /**
     * Static accessor for the right motor
     */
    static Motor* getRightMotor();
    
    /**
     * Static accessor for the left motor
     */
    static Motor* getLeftMotor();

    /**
     * Callback function overridden from ThresholdCallback class that's been inherited.
     */
    void callBackFunction();

    /**
     * Return a flag of whether the PID controller of this motor wants updates.
     */
    bool isPIDUpdatesWanted();
    
  private:
    /**
     * References of the left motor and the right motor. We'll initialise them too within motor.h
     * and they will be available through a public static function.
     */
    static Motor* leftMotor;
    static Motor* rightMotor;
    /**
     * direction pin
     */
    byte pinDirection;

    /**
     * Motor run pin
     */
    byte pinRun;

    /**
     * PID controller for this motor
     */
    PID_c* pid_controller;

    /**
     * A differentiator for left and right motors.
     */
    byte whichMotor = 0;

    /**
     * Last requested motor speed when running with PID
     */
    int last_requested_motor_speed = 0;
  
    /**
     * Constructor with a set of pins- directions and run control for the motor.
     * It doesn't have to be public, because we'll only create instances of motor
     * within motor.h and those instances will be available through a static function.
     */
    Motor(byte pinDirection, byte pinRun, byte whichMotor, PID_c* pid_controller);

    /**
     * Returns a pointer to the encoder that belongs to this motor.
     */
    Encoder* getEncoder();
    
    /**
     * Moves the motor to go forward or backward with a given PWM.
     * 
     * @power Positive value means: go forward, negative: go back. The absolute
     * value will be used as the PWM - passed directly to analogWrite().
     * 
     * NOTE: Deadband seems to be 0..8 both forwards and backwards for both motors. At power = 9
     * it just barely moves, but not with lower power. 
     * 
     * Going forward, left motor seems to be able to move at power = 8, but only sometimes, so 
     * I guess it has a little bit narrower deadband (but only if running in air,
     * not on ground). Going backwards, both motors barely move at power -8 in the air, but not on ground.
     * 
     */
    void turnMotor(int power);

    /**
     * Roughly converts motor speed to its corresponding PWM. This is not required
     * to be very precise, because we'll be controlling the PWM with PID anyway,
     * but as the start figure this should be good enough.
     * 
     * Rough idea for the speed limits can be gotten from the following:
     * 
     * Speed values for given motor powers without PID:
     * 
     * power | clicks per second (a.k.a speed)
     * --------------------------
     *  25   |      77 - 90
     *  100  |     280 - 300   
     *  200  |     590 - 636
     *  255  |     800 - 875
     *  
     *  This kind of roughly comes to a relationship of around: 
     *  
     *  power = motor_speed / 3.5
     *  It's not precise, but it will do as a starting point for PID.
     */
    int convertMotorSpeedToPWM(int motor_speed);

    /**
     * Stops the motor.
     */
    void stopMotor();
};

#endif
