#include "kinematics.h"
#include "pin_names_and_constants.h"
#include "encoder.h"
#include "motor.h"

long Kinematics::getCurrentX_raw() {
  return current_x;
}

long Kinematics::getCurrentY_raw() {
  return current_y;
}

long Kinematics::getCurrentX_mm() {
  return encoderCountsToMM(current_x);
}

long Kinematics::getCurrentY_mm() {
  return encoderCountsToMM(current_y);
}

double Kinematics::getCurrentHeading() {
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
int Kinematics::getCountsForRotationByAngle(float angle) {
  return mmToEncoderCounts(angle * WHEEL_SEPARATION) / 2;
}

/**
 * Returns angle towards home (0,0).
 */
float Kinematics::getAngleToGoHome() {
  long y_home = 0;
  long x_home = 0;
  long y_to_home = y_home - current_y;
  long x_to_home = x_home - current_x;

  return atan2(y_to_home, x_to_home);// - PI;
}

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
void Kinematics::turnByAngle(float angle, bool use_PID) {
  int left_counts = getCountsForRotationByAngle(angle);
  int right_counts = -left_counts;
  unsigned int counts = abs(left_counts);

  if (use_PID) {
    Motor::getLeftMotor()->goForGivenClicksAtGivenSpeed_PID(left_counts, (left_counts > 0 ? WALK_HOME_SPEED : -WALK_HOME_SPEED));
    Motor::getRightMotor()->goForGivenClicksAtGivenSpeed_PID(right_counts, (right_counts > 0 ? WALK_HOME_SPEED : -WALK_HOME_SPEED));
  } else {
    if (left_counts > 0) {
      Motor::getLeftMotor()->moveByCounts(counts, turning_power);
      Motor::getRightMotor()->moveByCounts(counts, -turning_power);
    } else {
      Motor::getLeftMotor()->moveByCounts(counts, -turning_power);
      Motor::getRightMotor()->moveByCounts(counts, turning_power);    
    }
  }
}

/**
 * Commands the wheels of the robot to turn in such a way
 * that the whole robot turns TO the given angle in radians.
 */
void Kinematics::turnToAngle(float angle, bool use_PID) {
  turnByAngle(angle - getCurrentHeading(), use_PID);
}

/**
 * Turns to the angle to go home.
 */
void Kinematics::turnToHomeHeading(bool use_PID) {
  turnToAngle(getAngleToGoHome(), use_PID);
}

/**
 * Runs the motors for the required number of counts to get back
 * home provided that we're already orientated to home. turnToHomeHeading()
 * has to be called and it of course needs to finish before this function.
 */
void Kinematics::walkDistanceToHome() {
  long y_home = 0;
  long x_home = 0;
  long y_to_home = y_home - current_y;
  long x_to_home = x_home - current_x;

  /**
   * Pithagorus theorem gives us the direct bee-line home.
   */
  float distance_home = sqrt(x_to_home * x_to_home + y_to_home * y_to_home);
  
  Motor::getRightMotor()->goForGivenClicksAtGivenSpeed_PID(distance_home, WALK_HOME_SPEED);
  Motor::getLeftMotor()->goForGivenClicksAtGivenSpeed_PID(distance_home, WALK_HOME_SPEED);
}

/**
 * Here we just want to set our motors to run at steady pace (with the heading PID disabled) to go straight.
 * We should be looking for the line meanwhile - once the line sensors report a reliable signal, this should
 * be stopped, but that's what we have the state machine for.
 */
void Kinematics::walkStraightLookingForLine() {
  Motor::getRightMotor()->setRequestedSpeed_PID(LOOK_FOR_LINE_SPEED);
  Motor::getLeftMotor()->setRequestedSpeed_PID(LOOK_FOR_LINE_SPEED);
}

/**
 * Here we want to stop the motors and disable their PID controllers.
 */
void Kinematics::fullStop() {
  Motor::getRightMotor()->stopMotorAndCancelPreviousInstruction();
  Motor::getLeftMotor()->stopMotorAndCancelPreviousInstruction();  
}

/**
 * Static accessor for our kinematics object.
 */
Kinematics* Kinematics::getKinematics() {
  return kinematics;
}

/**
 * The theta angle that we'll be getting, it will be in radians
 * and it may come out as a number greater than Pi or smaller 
 * than -Pi. If that happens, this will take away the additional 
 * value and reduce it to a value between -Pi and Pi.
 */
float Kinematics::truncate_angle(float* angle) {
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
Kinematics::Kinematics() {
  
}

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
