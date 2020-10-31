#ifndef _MOTOR_
#define _MOTOR_

#include "pin_names_and_constants.h"
#include "threshold_callback.h"
#include "encoder.h"
#include "pid.h"

/**
 * This class will control motor movement.
 */
class Motor : public ThresholdCallback {
  public:
    /**
     * Runs the motor forward for the given time amount with the given power
     */
    void goForwardForGivenTimeAtGivenPower(unsigned int ms, byte power) {
      turnMotor(power);
      LineSensor::setThreshold(this, ms);
    }

    /**
     * Runs the motor forward for the given time amount with the given power
     */
    void goBackwardForGivenTimeAtGivenPower(unsigned int ms, byte power) {
      turnMotor(-power);
      LineSensor::setThreshold(this, ms);
    }

    /**
     * Turns the motor at a constant speed controlled by PID for a given amount
     * of ms.
     */
    void goForGivenTimeAtGivenSpeed_PID(unsigned int ms, int motor_speed) {
      last_requested_motor_speed = motor_speed;
      turnMotor(convertMotorSpeedToPWM(motor_speed));
      LineSensor::setThreshold(this, ms);
    }

    /**
     * Turns the motor at a constant speed controlled by PID.
     */
    void goAtGivenSpeed_PID(int motor_speed) {
      last_requested_motor_speed = motor_speed;
      turnMotor(convertMotorSpeedToPWM(motor_speed));
    }

    /**
     * Performs the PID update and compensates motor power accordingly to the
     * PID output.
     */
    void updateMotorPIDcontroller(int current_motor_speed) {
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
    int getLastRequestedMotorSpeed_PID() {
      return last_requested_motor_speed;
    }

    /**
     * Applies a coefficient calculated by the heading PID to the speed that 
     * needs to be requested from the motor PIDs.
     */
    void updateRequestedSpeedByAFactor_PID(float correction_factor) {
      last_requested_motor_speed = correction_factor * last_requested_motor_speed;
    }

    /**
     * Sets the target speed for the motro PID.
     */
    void setRequestedSpeed_PID(int new_speed) {
      last_requested_motor_speed = new_speed;
    }
    
    /**
     * Runs the motor forward for for the specified amount of encoder counts at 35 PWM power
     */
    void goForwardByCounts(unsigned int counts) {
      if (counts > 0) {
        turnMotor(35);
        setThreshold(counts);
        encoder->setThreshold(this);
      }
    }

    /**
     * Runs the motor backwards for for the specified amount of encoder counts at 35 PWM power
     */
    void goBackwardByCounts(unsigned int counts) {
      if (counts > 0) {
        turnMotor(-35);
        setThreshold((int)-counts); //# need to cast here, otherwise it's interpreted as unsigned int loaded with huge value
        encoder->setThreshold(this);
      }
    }

    /**
     * A convenience function that takes negative or positive power and based on that
     * either calls goForwardByCounts or goBackwardByCounts
     */
    void moveByCounts(unsigned int counts, int power) {
      if (power > 0) {
        goForwardByCounts(counts, power);
      } else {
        goBackwardByCounts(counts, abs(power));
      }
    }
  
    /**
     * Runs the motor forward for for the specified amount of encoder counts at the specified PWM power
     */
    void goForwardByCounts(unsigned int counts, byte power) {
      if (counts > 0) {
        turnMotor(power);
        setThreshold(counts);
        encoder->setThreshold(this);
      }
    }

    /**
     * Runs the motor backwards for for the specified amount of encoder counts at the specified PWM power
     */
    void goBackwardByCounts(unsigned int counts, byte power) {
      if (counts > 0) {
        turnMotor(-power);
        setThreshold((int)-counts); //# need to cast here, otherwise it's interpreted as unsigned int loaded with huge value
        encoder->setThreshold(this);
      }
    }

    /**
     * Stops motor and cancels any previous threshold that was sent to the encoder.
     */
    void stopMotorAndCancelPreviousInstruction() {
      this->clearThreshold();
      stopMotor();
    }

    /**
     * Static accessor for the right motor
     */
    static Motor* getRightMotor() {
      return rightMotor;
    }
    
    /**
     * Static accessor for the left motor
     */
    static Motor* getLeftMotor() {
      return leftMotor;
    }

    /**
     * Callback function overridden from ThresholdCallback class that's been inherited.
     */
    void callBackFunction() {
      stopMotor();
    }
  private:
    /**
     * Constructor with a set of pins- directions and run control for the motor.
     * It doesn't have to be public, because we'll only create instances of motor
     * within motor.h and those instances will be available through a static function.
     */
    Motor(byte pinDirection, byte pinRun, Encoder* encoder, PID_c* pid_controller) {
      this->pinDirection = pinDirection;
      this->pinRun = pinRun;
      this->encoder = encoder;
      this->pid_controller = pid_controller;

      pinMode(pinDirection, OUTPUT);
      pinMode(pinRun, OUTPUT);
    }
    
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
     * Encoder for this motor.
     */
    Encoder* encoder;

    /**
     * PID controller for this motor
     */
    PID_c* pid_controller;

    /**
     * Last requested motor speed when running with PID
     */
    int last_requested_motor_speed = 0;
    
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
    void turnMotor(int power) {
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
    int convertMotorSpeedToPWM(int motor_speed) {
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
    void stopMotor() {
      analogWrite(pinRun, 0);
      digitalWrite(pinDirection, LOW);

      /**
       * PID things.
       */
      this->pid_controller->reset();
      last_requested_motor_speed = 0;
    }
};

/**
 * Instantiating our motors. We're passing the relevant pins and encoder for the motor.
 * Kp = 0.8 give more oscillation on changes, so use 0.6 for now.
 * Kd = 0.2 seems to compensate well enough for Kp caused oscillations.
 * 
 * Kp = 0.2 Kd = 3.0 and Ki = 0.04 look like good candidate- very little oscillation and good adjustments.
 */
Motor* Motor::rightMotor = new Motor(RIGHT_MOTOR_DIR, RIGHT_MOTOR_RUN, Encoder::getRightEncoder(), new PID_c(0.2, 0.04, 3.0));
Motor* Motor::leftMotor = new Motor(LEFT_MOTOR_DIR, LEFT_MOTOR_RUN, Encoder::getLeftEncoder(), new PID_c(0.5, 0.08, 4.0));

#endif
