#include "motor.h"
#include "state_machine.h"
#include "ir_line_sensor.h"
#include "pin_names_and_constants.h"

/**
 * Runs the motor forward for the given time amount with the given power
 */
void Motor::goForwardForGivenTimeAtGivenPower(unsigned int ms, byte power) {
  turnMotor(power);
  LineSensor::setThreshold(this, ms);
}

/**
 * Runs the motor forward for the given time amount with the given power
 */
void Motor::goBackwardForGivenTimeAtGivenPower(unsigned int ms, byte power) {
  turnMotor(-power);
  LineSensor::setThreshold(this, ms);
}

/**
 * Turns the motor at a constant speed controlled by PID for a given amount
 * of ms.
 */
void Motor::goForGivenTimeAtGivenSpeed_PID(unsigned int ms, int motor_speed) {
  turnMotor(convertMotorSpeedToPWM(motor_speed));
  setRequestedSpeed_PID(motor_speed);
  LineSensor::setThreshold(this, ms);
}

/**
 * Turns the motor at a constant speed controlled by PID for a given amount
 * of encoder counts.
 */
void Motor::goForGivenClicksAtGivenSpeed_PID(long clicks, int motor_speed) {
  stopMotor(); //# to clear and reset PID controller
  turnMotor(convertMotorSpeedToPWM(motor_speed));
  setRequestedSpeed_PID(motor_speed);

  setThreshold(clicks);
  getEncoder()->setThreshold(this);
}

/**
 * Turns the motor at a constant speed controlled by PID.
 */
void Motor::goAtGivenSpeed_PID(int motor_speed) {
  stopMotor(); //# to clear and reset PID controller
  
  turnMotor(convertMotorSpeedToPWM(motor_speed));
  setRequestedSpeed_PID(motor_speed);
}

/**
 * Performs the PID update and compensates motor power accordingly to the
 * PID output.
 */
void Motor::updateMotorPIDcontroller(int current_motor_speed) {
  /**
   * We only want to operate the PID controller if the demand speed is non zero.
   * If it is zero thought, then the motor should have been stopped by now and we
   * are not to start running it again with some residue PID result.
   */
  if (last_requested_motor_speed != 0) {
    float additional_required_speed = this->pid_controller->update(last_requested_motor_speed, current_motor_speed);
    //turnMotor(round(new_motor_power));
//      Serial.print("PID update: ");
//      Serial.println(additional_required_speed);
//      Serial.print("New requested speed: ");
//      Serial.println(additional_required_speed + last_requested_motor_speed);
//      Serial.print("New requested power: ");
//      Serial.println((additional_required_speed + last_requested_motor_speed) / 3.5);

//      Serial.print(current_motor_speed);
//      Serial.print(", ");
//      Serial.println(additional_required_speed);
    turnMotor(convertMotorSpeedToPWM(additional_required_speed + last_requested_motor_speed));
  }
}

/**
 * Accessor for the last requested motor speed.
 */
int Motor::getLastRequestedMotorSpeed_PID() {
  return last_requested_motor_speed;
}

/**
 * Applies a coefficient calculated by the heading PID to the speed that 
 * needs to be requested from the motor PIDs.
 */
void Motor::updateRequestedSpeedByAFactor_PID(float correction_factor) {
  setRequestedSpeed_PID(correction_factor * last_requested_motor_speed);
}

/**
 * Sets the target speed for the motro PID.
 */
void Motor::setRequestedSpeed_PID(int new_speed) {
  this->pid_controller->setUpdatesWanted(true);
  last_requested_motor_speed = new_speed;
}

/**
 * Return a flag of whether the PID controller of this motor wants updates.
 */
bool Motor::isPIDUpdatesWanted() {
  return this->pid_controller->isUpdatesWanted();
}

/**
 * Runs the motor forward for for the specified amount of encoder counts at 35 PWM power
 */
void Motor::goForwardByCounts(unsigned int counts) {
  if (counts > 0) {
    turnMotor(35);
    setThreshold(counts);
    getEncoder()->setThreshold(this);
  }
}

/**
 * Runs the motor backwards for for the specified amount of encoder counts at 35 PWM power
 */
void Motor::goBackwardByCounts(unsigned int counts) {
  if (counts > 0) {
    turnMotor(-35);
    setThreshold((int)-counts); //# need to cast here, otherwise it's interpreted as unsigned int loaded with huge value
    getEncoder()->setThreshold(this);
  }
}

/**
 * A convenience function that takes negative or positive power and based on that
 * either calls goForwardByCounts or goBackwardByCounts
 */
void Motor::moveByCounts(unsigned int counts, int power) {
  if (power > 0) {
    goForwardByCounts(counts, power);
  } else {
    goBackwardByCounts(counts, abs(power));
  }
}

/**
 * Runs the motor forward for for the specified amount of encoder counts at the specified PWM power
 */
void Motor::goForwardByCounts(unsigned int counts, byte power) {
  if (counts > 0) {
    turnMotor(power);
    setThreshold(counts);
    getEncoder()->setThreshold(this);
    
//    Serial.print("SETTING THR F: ");
//    Serial.print(counts);
//    Serial.print(" ADDR: ");
//    Serial.println((long)getEncoder());
  }
}

/**
 * Runs the motor backwards for for the specified amount of encoder counts at the specified PWM power
 */
void Motor::goBackwardByCounts(unsigned int counts, byte power) {
  if (counts > 0) {
    turnMotor(-power);
    setThreshold((int)-counts); //# need to cast here, otherwise it's interpreted as unsigned int loaded with huge value
    getEncoder()->setThreshold(this);

//    Serial.print("SETTING THR B: ");
//    Serial.print(counts);
//    Serial.print(" ADDR: ");
//    Serial.println((long)getEncoder());
  }
}

/**
 * Stops motor and cancels any previous threshold that was sent to the encoder.
 */
void Motor::stopMotorAndCancelPreviousInstruction() {
  clearThreshold();
  stopMotor();
}

/**
 * Static accessor for the right motor
 */
Motor* Motor::getRightMotor() {
  return rightMotor;
}

/**
 * Static accessor for the left motor
 */
Motor* Motor::getLeftMotor() {
  return leftMotor;
}

/**
 * Callback function overridden from ThresholdCallback class that's been inherited.
 */
void Motor::callBackFunction() {
  //Serial.println("CALLBACK###############");
  stopMotor();

  /**
   * After the motor has done its job, we want to notify our state machine so that it can
   * advance.
   */
  StateMachine::getStateMachine()->update();
}

/**
 * Constructor with a set of pins- directions and run control for the motor.
 * It doesn't have to be public, because we'll only create instances of motor
 * within motor.h and those instances will be available through a static function.
 */
Motor::Motor(byte pinDirection, byte pinRun, byte whichMotor, PID_c* pid_controller) {
  this->pinDirection = pinDirection;
  this->pinRun = pinRun;
  this->whichMotor = whichMotor;
  this->pid_controller = pid_controller;

  pinMode(pinDirection, OUTPUT);
  pinMode(pinRun, OUTPUT);
}

/**
 * Returns a pointer to the encoder that belongs to this motor.
 */
Encoder* Motor::getEncoder() {
  if (whichMotor == RIGHT_MOTOR) {
    return Encoder::getRightEncoder();
  } else if (whichMotor == LEFT_MOTOR) {
    return Encoder::getLeftEncoder();
  } else {
    return NULL;
  }
}

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
void Motor::turnMotor(int power) {
  if (power > 0) {
    digitalWrite(pinDirection, MOTOR_FORWARD);
  } else {
    digitalWrite(pinDirection, MOTOR_BACKWARD);
  }

  /**
   * There's no point to turn the motors at all if the the power is less than 9 as
   * that is within deadband. Let's make it 12 to be sure we're out of deadband.
   */
  if (abs(power) > 11) {
    /**
     * If we don't take abs value here, then it looks like values greater than 255 overflow
     * and negative values are treated as unsigned byte, rather than a negative value, 
     * so -1 will come out as 255. Looks like analogWrite function takes in a paramater of unsigned byte.
     */
    analogWrite(pinRun, abs(power));
  }
}

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
int Motor::convertMotorSpeedToPWM(int motor_speed) {
  int motor_power = round(motor_speed / 3.5);
  
  if (motor_power > 255) {
    motor_power = 255;
  } else if (motor_power < -255) {
    motor_power = -255;
  }

  return motor_power;
}

/**
 * Stops the motor.
 */
void Motor::stopMotor() {
  analogWrite(pinRun, 0);
  digitalWrite(pinDirection, LOW);

  /**
   * PID things.
   */
  this->pid_controller->setUpdatesWanted(false);
  this->pid_controller->reset();
  last_requested_motor_speed = 0;
  //Serial.println("MOTOR STOP");
}

/**
 * Instantiating our motors. We're passing the relevant pins and encoder for the motor.
 * Kp = 0.8 give more oscillation on changes, so use 0.6 for now.
 * Kd = 0.2 seems to compensate well enough for Kp caused oscillations.
 * 
 * Kp = 0.2 Kd = 3.0 and Ki = 0.04 look like good candidate- very little oscillation and good adjustments.
 */
Motor* Motor::rightMotor = new Motor(RIGHT_MOTOR_DIR, RIGHT_MOTOR_RUN, RIGHT_MOTOR, new PID_c(0.2, 0.04, 3.0));
Motor* Motor::leftMotor = new Motor(LEFT_MOTOR_DIR, LEFT_MOTOR_RUN, LEFT_MOTOR, new PID_c(0.5, 0.08, 4.0));
