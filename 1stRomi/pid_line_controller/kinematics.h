#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <Arduino.h>

/**
 * Converts encoder counts to mm. This is going to assume that one full revolution of a wheel returns 360
 * encoder counts (and not 1440 as stated in Romi documentation: https://www.pololu.com/product/3542).
 * I don't yet know why I get 360 counts instead of 1440 - probably misconfiguration of the pin change
 * interrupt that the encoder relies on. Maybe I'll try to fix it at some point, but for now 360 will have
 * to be good enough.
 */
#define encoderCountsToMM(pulses_to_convert) (MM_PER_PULSE * pulses_to_convert) //# conversion from pulses to mm.
/**
 * Converts mm to encoder counts. Same assumptions as for encoderCountsToMM.
 */
#define mmToEncoderCounts(mm_to_convert) (mm_to_convert / MM_PER_PULSE) //# conversion from pulses to mm.

class Kinematics {
  public:

    // Function Prototypes
    void update();    // update kinematics

    long getCurrentX_raw();

    long getCurrentY_raw();

    long getCurrentX_mm();

    long getCurrentY_mm();
    
    double getCurrentHeading();

    /**
     * Returns the required number of encoder counts for the LEFT wheel
     * that we need for a rotation BY the given angle. The angle is given
     * in radians and the function returns encoder count for the LEFT wheel.
     * The amount for the RIGHT wheel is the same as the amount for the LEFT
     * wheel, but with inverted sign. So for example if this function returns
     * 100, then we're expected to turn the LEFT wheel by 100 counts and the
     * left wheel by -100.
     */
    int getCountsForRotationByAngle(float angle);

    /**
     * Returns angle towards home (0,0).
     */
    float getAngleToGoHome();

    byte turning_power = 100;
    /**
     * Commands the wheels of the robot to turn in such a way
     * that the whole robot turns BY the given angle in radians.
     * 
     * angle:
     * Positive agrument - turns clock wise
     * Negative argument - turns counter clock wise.
     * 
     * bool use_PID - a flag of whether we want to use PID or not.
     */
    void turnByAngle(float angle, bool use_PID);

    /**
     * Commands the wheels of the robot to turn in such a way
     * that the whole robot turns TO the given angle in radians.
     * 
     * angle - angle in radians
     * 
     * bool use_PID - a flag of whether we want to use PID or not.
     */
    void turnToAngle(float angle, bool use_PID);
    
    /**
     * Turns to the angle to go home.
     * 
     * bool use_PID - a flag of whether we want to use PID or not.
     */
    void turnToHomeHeading(bool use_PID);

    /**
     * Runs the motors for the required number of counts to get back
     * home provided that we're already orientated to home. turnToHomeHeading()
     * has to be called and it of course needs to finish before this function.
     */
    void walkDistanceToHome();

    /**
     * Static accessor for our kinematics object.
     */
    static Kinematics* getKinematics();

  private:
    /**
     * The theta angle that we'll be getting, it will be in radians
     * and it may come out as a number greater than Pi or smaller 
     * than -Pi. If that happens, this will take away the additional 
     * value and reduce it to a value between -Pi and Pi.
     */
    float truncate_angle(float* angle);

    /**
     * Private constructor, because we'll access this class via a static getter.
     */
    Kinematics();

    /**
     * Current co-ordinates.
     */
    float current_x = 0.0;
    float current_y = 0.0;
    float current_theta = 0.0;

    /**
     * Previous pulse counts of the encoders
     * and previous angle.
     */
    long prev_pulses_right = 0;
    long prev_pulses_left = 0;
    float previous_theta = 0.0;

    /**
     * Current pulse counts of the encoders.
     */
    long cur_pulses_right = 0;
    long cur_pulses_left = 0;
    long cur_distance_travelled_left_wheel = 0;
    long cur_distance_travelled_right_wheel = 0;
    long cur_distance_travelled = 0;

    /**
     * A static reference to itself, which we'll return via a static getter.
     */
    static Kinematics* kinematics;
}; // End of class definition.

#endif
