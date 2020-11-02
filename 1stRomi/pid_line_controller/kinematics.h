#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "pin_names_and_constants.h"
#include "encoder.h"
#include "motor.h"

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

    // What variables do you need?
    // What is the appropriate type?
    // ...

    // Function Prototypes
    void update();    // update kinematics

    long getCurrentX_raw() {
      return current_x;
    }

    long getCurrentY_raw() {
      return current_y;
    }

    long getCurrentX_mm() {
      return encoderCountsToMM(current_x);
    }

    long getCurrentY_mm() {
      return encoderCountsToMM(current_y);
    }

    double getCurrentHeading() {
      return current_theta;
    }

    /**
     * Returns the required number of encoder counts for the LEFT wheel
     * that we need for a rotation BY the given angle. The angle is given
     * in radians and the function returns encoder count for the LEFT wheel.
     * The amount for the RIGHT wheel is the same as the amount for the LEFT
     * wheel, but with inverted sign. So for example if this function returns
     * 100, then we're expected to turn the LEFT wheel by 100 counts and the
     * left wheel by -100.
     */
    int getCountsForRotationByAngle(float angle) {
      return mmToEncoderCounts(angle * WHEEL_SEPARATION) / 2;
    }

    /**
     * Returns angle towards home (0,0).
     */
    float getAngleToGoHome() {
      long y_home = 0;
      long x_home = 0;
      long y_to_home = y_home - current_y;
      long x_to_home = x_home - current_x;

      return atan2(y_to_home, x_to_home) - PI;
    }

    byte turning_power = 100;
    /**
     * Commands the wheels of the robot to turn in such a way
     * that the whole robot turns BY the given angle in radians.
     * 
     * Positive agrument - turns clock wise
     * Negative argument - turns counter clock wise.
     */
    void turnByAngle(float angle) {
      int left_counts = getCountsForRotationByAngle(angle);
      unsigned int counts = abs(left_counts);
    
      if (left_counts > 0) {
        Motor::getLeftMotor()->moveByCounts(counts, turning_power);
        Motor::getRightMotor()->moveByCounts(counts, -turning_power);
      } else {
        Motor::getLeftMotor()->moveByCounts(counts, -turning_power);
        Motor::getRightMotor()->moveByCounts(counts, turning_power);    
      }
    }

    /**
     * Commands the wheels of the robot to turn in such a way
     * that the whole robot turns TO the given angle in radians.
     */
    void turnToAngle(float angle) {
      turnByAngle(angle - getCurrentHeading());
    }
    
    /**
     * Turns to the angle to go home.
     */
    void turnToHomeHeading() {
      turnToAngle(getAngleToGoHome());
    }

    /**
     * Runs the motors for the required number of counts to get back
     * home provided that we're already orientated to home. turnToHomeHeading()
     * has to be called and it of course needs to finish before this function.
     */
    void walkDistanceToHome() {
      Motor::getRightMotor()->goForwardForGivenTimeAtGivenPower(5000, 100);
      Motor::getLeftMotor()->goForwardForGivenTimeAtGivenPower(5000, 100);
    }

    /**
     * Static accessor for our kinematics object.
     */
    static Kinematics* getKinematics() {
      return kinematics;
    }

  private:
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

    /**
     * The theta angle that we'll be getting, it will be in radians
     * and it may come out as a number greater than Pi or smaller 
     * than -Pi. If that happens, this will take away the additional 
     * value and reduce it to a value between -Pi and Pi.
     */
    float truncate_angle(float* angle) {
      while (*angle > PI) {
        *angle -= (2 * PI);
      }

      while (*angle < -PI) {
        *angle += (2 * PI);
      }

      return *angle;
    }

    /**
     * Private constructor, because we'll access this class via a static getter.
     */
    Kinematics() {
      
    }
}; // End of class definition.

Kinematics* Kinematics::kinematics = new Kinematics();

// Routine to execute the update to
// kinematics 
void Kinematics::update() {
  
  cur_pulses_right = Encoder::getRightEncoder()->getPulseCount();
  cur_pulses_left = Encoder::getLeftEncoder()->getPulseCount();

  /**
   * Distance travelled since last update. This may change, because we are
   * not taking the turning into account.
   */
  cur_distance_travelled_left_wheel = cur_pulses_left - prev_pulses_left;
  cur_distance_travelled_right_wheel = cur_pulses_right - prev_pulses_right;
  
  current_theta = previous_theta + encoderCountsToMM(float(cur_distance_travelled_left_wheel - cur_distance_travelled_right_wheel)) / WHEEL_SEPARATION;

//  Serial.print("current_theta: ");
//  Serial.println(current_theta);
//  Serial.print("previous_theta: ");
//  Serial.println(previous_theta);
//  Serial.print("cur_distance_travelled_left_wheel: ");
//  Serial.println(cur_distance_travelled_left_wheel);
//  Serial.print("cur_distance_travelled_right_wheel: ");
//  Serial.println(cur_distance_travelled_right_wheel);
//  Serial.print("WHEEL_SEPARATION: ");
//  Serial.println(WHEEL_SEPARATION);
//  Serial.print("encoderCountsToMM: ");
//  Serial.println(encoderCountsToMM(float(cur_distance_travelled_left_wheel - cur_distance_travelled_right_wheel)));
//  Serial.print("truncate_angle: ");
  //Serial.println(truncate_angle(&current_theta));

  /**
   * Making sure that we don't get more than the full circle of radians or less than 0.
   */
  truncate_angle(&current_theta);
  
  cur_distance_travelled = ((cur_distance_travelled_left_wheel + cur_distance_travelled_right_wheel) / 2);
  
  /**
   * According to kinematics theory y_new = y_old + d * sin(Theta)
   * 
   * So that's how we'll be updating our current coordinates then.
   * For now we don't yet have Theta.
   */
  current_y = current_y + cur_distance_travelled * sin(previous_theta);
  current_x = current_x + cur_distance_travelled * cos(previous_theta);

  prev_pulses_right = cur_pulses_right;
  prev_pulses_left = cur_pulses_left;
  previous_theta = current_theta;
}

#endif
