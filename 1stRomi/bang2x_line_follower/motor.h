#include "pin_names_and_constants.h"

/**
 * This class will control motor movement.
 */
class Motor {
  public:
    /**
     * Runs the motor forward for 1 second at half PWM power.
     */
    void goForward_1Second() {
      turnMotor(127);
      delay(1000);
      stopMotor();
    }

    static Motor* getRightMotor() {
      return rightMotor;
    }

    static Motor* getLeftMotor() {
      return leftMotor;
    }
  
  private:
    /**
     * Constructor with a set of pins- directions and run control for the motor.
     * It doesn't have to be public, because we'll only create instances of motor
     * within motor.h and those instances will be available through a static function.
     */
    Motor(byte pinDirection, byte pinRun) {
      this->pinDirection = pinDirection;
      this->pinRun = pinRun;

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
        digitalWrite(pinDirection, HIGH);
      } else {
        digitalWrite(pinDirection, LOW);
      }
  
      /**
       * There's no point to turn the motors at all if the the power is less than 9 as
       * that is within deadband.
       */
      if (abs(power) > 8) {
        /**
         * If we don't take abs value here, then it looks like values greater than 255 overflow
         * and negative values are treated as unsigned byte, rather than a negative value, 
         * so -1 will come out as 255. Looks like analogWrite function takes in a paramater of unsigned byte.
         */
        analogWrite(pinRun, abs(power));
      }
    }

    /**
     * Stops the motor.
     */
    void stopMotor() {
      analogWrite(pinRun, 0);
      digitalWrite(pinDirection, LOW);
    }
};

/**
 * Instantiating our motors.
 */
Motor* Motor::rightMotor = new Motor(RIGHT_MOTOR_DIR, RIGHT_MOTOR_RUN);
Motor* Motor::leftMotor = new Motor(LEFT_MOTOR_DIR, LEFT_MOTOR_RUN);
