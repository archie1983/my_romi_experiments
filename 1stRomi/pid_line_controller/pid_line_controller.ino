#include "pin_names_and_constants.h"
#include "ir_line_sensor.h"
#include "motor.h"
#include "pid.h"
#include "kinematics.h"
#include "state_machine.h"


/**
 * This will be our heading PID controller. The input will be the weighed line sensor value [-1.0, 1.0].
 * The output naturally will also be a value with boundaries [-1.0, 1.0]. We'll be setting the bias to
 * 0 to get the robot to go straight and to some positive or negative value withing the same boundaries
 * [-1.0, 1.0] to get it to turn left or right.
 * 
 * Obviously the non-0 output should increase the speed of one of the wheels and de-crease the speed of
 * the other.
 */
//PID_c heading_pid(0.7, 0.0008, 3.0);
PID_c heading_pid(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D);

void setup() {
  // Start Serial monitor and print "reset"
  // so we know if the board is reseting
  // unexpectedly.
  Serial.begin(9600);
  Serial.setTimeout(100);

/**
 * Only wait for serial to connect if we're running with a cable
 */
  if(OPER_MODE == DEBUG_MODE) {
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  } else {
    /**
     * If we're running without a cable, then wait a little after switched on
     * and then start moving.
     */
     delay(3000);
     lookForLine();
  }
  Serial.println("***RESET***");

  /**
   * Turns out that if we do this inside the constructor of LineSensor, then our timer 
   * configuration gets overwritten later, because our instance is constructed before 
   * Arduino has initialised its stuff. So we have no choice but to call this function 
   * in the setup section of the main code.
   */
  LineSensor::reInitTimer(LINE_SENSOR_UPDATE_FREQUENCY);

  StateMachine::getStateMachine()->setHeadingPID(&heading_pid);
  StateMachine::getStateMachine()->setState(StateMachine::LineFollowingStates::IDLING);
}

void loop() {  
  act_on_commands();
  event_scheduler();
}

long time_now = 0;
long time_last_report = 0;
long time_last_kinematics_update = 0;
long time_last_mpid = 0;
long time_last_hpid = 0;
long time_last_state_mach_update = 0;

void event_scheduler() {
  time_now = millis();

  if (time_now - time_last_report >= REPORT_TIME) {
    talk_about_it(true);
    time_last_report = time_now;
  }

  /**
   * These motor PIDs may need to be updated even if we're not following a line.
   * We may be just going forward or back for a given time.
   */
  if ((Motor::getLeftMotor()->isPIDUpdatesWanted() || Motor::getRightMotor()->isPIDUpdatesWanted()) && time_now - time_last_mpid >= MOTOR_PID_UPDATE_TIME) {
    
    if (Motor::getLeftMotor()->isPIDUpdatesWanted()){
      Motor::getLeftMotor()->updateMotorPIDcontroller(getLeftWheelSpeed());
    }

    if (Motor::getRightMotor()->isPIDUpdatesWanted()){
      Motor::getRightMotor()->updateMotorPIDcontroller(getRightWheelSpeed());
    }

    time_last_mpid = time_now;
  }

  /*
   * If that's what user wants and it's time for that, then follow the line
   */
  if (heading_pid.isUpdatesWanted() && time_now - time_last_hpid >= HEADING_PID_UPDATE_TIME) {
    nested_pid_weighed_sensors();
    time_last_hpid = time_now;
  }

  /**
   * Updating kinematics if it's time.
   */
  if (time_now - time_last_kinematics_update >= KINEMATICS_UPDATE_TIME){
    Kinematics::getKinematics()->update();
    time_last_kinematics_update = time_now;
  }

  /**
   * Updating state machine with line sensor data.
   */
  if (time_now - time_last_state_mach_update >= STATE_MACH_UPDATE_TIME){
    bool sensor_r = LineSensor::getRightSensor()->overLine();
    bool sensor_c = LineSensor::getCentreSensor()->overLine();
    bool sensor_l = LineSensor::getLeftSensor()->overLine();
    
    StateMachine::getStateMachine()->update(sensor_l, sensor_c, sensor_r);
    time_last_state_mach_update = time_now;
  }
}

/**
 * Calculates the weighed line sensor value as described in exercise 3 and returns it.
 */
float weighted_line_sensor() {
  unsigned int left_value = LineSensor::getLeftSensor()->getCurrentSensorValueWhenReliableSignal();
  unsigned int centre_value = LineSensor::getCentreSensor()->getCurrentSensorValueWhenReliableSignal();
  unsigned int right_value = LineSensor::getRightSensor()->getCurrentSensorValueWhenReliableSignal();
  
  //float total_magnitude = left_value + centre_value + right_value;
  float total_magnitude = left_value + centre_value + right_value;

  if (total_magnitude > 0) {
    float left_proportional = left_value / total_magnitude;
    float right_proportional = right_value / total_magnitude;
    
    return left_proportional - right_proportional;
  } else {
    return 0.0;
  }
}

int new_speed_right = 0;
int new_speed_left = 0;

/**
 * Weighted line sensor nested PID controller. Here we have an instance of a PID controller,
 * which drives another two PID instances- one for each wheel.
 */
void nested_pid_weighed_sensors() {

  /**
   * wls will be our measurement.
   */
  float wls = weighted_line_sensor();
  
  /**
   * Demand of 0.0 should keep it going straight.
   * 
   * The heading_correction will be our PID output.
   */
  float heading_correction = heading_pid.update(0.0, wls);

  /**
   * The rest should follow almost the same algorithm as weighed sensor bang-bang did,
   * setting the demanded speed to either of the wheels. The only difference is that
   * heading_correction is a correction measure, not an absolute value, so we should
   * update the current speed of either motor with regards to the heading_correction.
   */
//  bool sensor_r = LineSensor::getRightSensor()->overLine();
//  bool sensor_c = LineSensor::getCentreSensor()->overLine();
//  bool sensor_l = LineSensor::getLeftSensor()->overLine();
//
//  byte number_of_sensors_over_line = 0;
//  if (sensor_r) number_of_sensors_over_line++;
//  if (sensor_c) number_of_sensors_over_line++;
//  if (sensor_l) number_of_sensors_over_line++;

  /**
   * If heading correction is positive, then we'll supply more power to the right motor
   * and less power to the left one.
   * If heading correction is negative, then we'll supply more power to the left motor
   * and less power to the right one, thus causing a turn.
   */
  new_speed_right = WLS_TURNING_SPEED * heading_correction * -1;
  new_speed_left = WLS_TURNING_SPEED * heading_correction;

  if (abs(wls) > HEADING_TOLERANCE) {
  //if ((sensor_l || sensor_c || sensor_r) && abs(wls) > 0.06) {
  //if (number_of_sensors_over_line < 3 && number_of_sensors_over_line > 0 && abs(wls) > 0.06) {
    if(OPER_MODE == DEBUG_MODE) {
//      /*
//       * turning
//       */
//      Serial.print("Turning ");
//      /**
//       * If left motor is being driven stronger than the right, then we're turning right
//       * and vice versa.
//       */
//      Serial.print(new_speed_right < new_speed_left ? "right : " : "left : ");
//      Serial.print(new_speed_right);
//      Serial.print(" # ");
//      Serial.print(new_speed_left);
//      Serial.print(" wls=");
//      Serial.print(wls);
//      Serial.print(" heading_correction=");
//      Serial.println(heading_correction);
    }

    Motor::getRightMotor()->updateRequestedSpeedByAdjustment_PID(new_speed_right);
    Motor::getLeftMotor()->updateRequestedSpeedByAdjustment_PID(new_speed_left);
//    Motor::getRightMotor()->setRequestedSpeed_PID(new_speed_right);
//    Motor::getLeftMotor()->setRequestedSpeed_PID(new_speed_left);
  } else {
    /*
     * going straight 
     */
    if(OPER_MODE == DEBUG_MODE) {
      //Serial.println("Going straight");
    }
    //Serial.println("Going straight");
    //heading_pid.reset();
    Motor::getRightMotor()->setRequestedSpeed_PID(TRAVEL_LINE_SPEED);
    Motor::getLeftMotor()->setRequestedSpeed_PID(TRAVEL_LINE_SPEED);
  }
}

/**
 * Reads a command from the Serial connection and acts on it
 */
void act_on_commands() {
  if(OPER_MODE == DEBUG_MODE) {
    int steps_to_move = 200;
    int steps_to_turn = 100;
    int time_to_turn = 200;
    int initial_pid_speed = 600;
    bool tpid_dir = false;
    //This line checks whether there is anything to read
    if ( Serial.available() ) {
      String in_cmd = Serial.readString();
  
      if (in_cmd.indexOf("opid") > -1) { //# Go one way with PID and when receive this again, then switch direction.
        if (getRightWheelSpeed() > 0) {
          if (tpid_dir) {
            Motor::getRightMotor()->setRequestedSpeed_PID(TURNING_SPEED);
          } else {
            Motor::getRightMotor()->setRequestedSpeed_PID(-TURNING_SPEED);
          }
          tpid_dir = !tpid_dir;
        } else {
          Motor::getRightMotor()->goAtGivenSpeed_PID(TURNING_SPEED);
        }
      } else if (in_cmd.indexOf("stop") > -1) { //# Stopping completely
        //Serial.println("STOPPING");
        stopAllMotion();
      } else if (in_cmd.indexOf("line") > -1) { //# Starting to look for line
        Serial.println("Looking for line");
        lookForLine();
      } else if (in_cmd.indexOf("gohome") > -1) { //# Going home
        Serial.println("Going home");
        goHome();
      } else if (in_cmd.indexOf("ta1") > -1) { //# turning experiment
        Serial.println("Turning to pi/2");
        Kinematics::getKinematics()->turnToAngle(PI / 2, false);
      } else if (in_cmd.indexOf("ta2") > -1) { //# turning experiment
        Serial.println("Turning to pi");
        Kinematics::getKinematics()->turnToAngle(PI, false);
      } else if (in_cmd.indexOf("ta3") > -1) { //# turning experiment
        Serial.println("Turning to -pi/2");
        Kinematics::getKinematics()->turnToAngle(-PI / 2, false);
      } else if (in_cmd.indexOf("ta4") > -1) { //# turning experiment
        Serial.println("Turning to -pi");
        Kinematics::getKinematics()->turnToAngle(-PI, false);
      } else if (in_cmd.indexOf("ta5") > -1) { //# turning experiment
        Serial.println("Turning to 0");
        Kinematics::getKinematics()->turnToAngle(0, false);
      } else if (in_cmd.indexOf("ta6") > -1) { //# turning experiment
        Serial.println("Turning to 2*pi");
        Kinematics::getKinematics()->turnToAngle(2 * PI, false);
      } else if (in_cmd.indexOf("t1") > -1) { //# turning experiment
        Serial.println("Turning by pi/2");
        Kinematics::getKinematics()->turnByAngle(PI / 2, false);
      } else if (in_cmd.indexOf("t2") > -1) { //# turning experiment
        Serial.println("Turning by pi");
        Kinematics::getKinematics()->turnByAngle(PI, false);
      } else if (in_cmd.indexOf("t3") > -1) { //# turning experiment
        Serial.println("Turning by -pi/2");
        Kinematics::getKinematics()->turnByAngle(-PI / 2, false);
      } else if (in_cmd.indexOf("t4") > -1) { //# turning experiment
        Serial.println("Turning by -pi");
        Kinematics::getKinematics()->turnByAngle(-PI, false);
      } else if (in_cmd.indexOf("t5") > -1) { //# turning experiment
        Serial.println("Turning by 0");
        Kinematics::getKinematics()->turnByAngle(0, false);
      } else if (in_cmd.indexOf("t6") > -1) { //# turning experiment
        Serial.println("Turning by 2*pi");
        Kinematics::getKinematics()->turnByAngle(2 * PI, false);
      } else if (in_cmd.indexOf("pid+") > -1) { //# PID experiment moving forward
        Serial.println("PID experiment forw");
        //Motor::getRightMotor()->goAtGivenSpeed_PID(100);
        Motor::getRightMotor()->goForGivenTimeAtGivenSpeed_PID(5000, initial_pid_speed);
        Motor::getLeftMotor()->goForGivenTimeAtGivenSpeed_PID(5000, initial_pid_speed);
      } else if (in_cmd.indexOf("pid-") > -1) { //# PID experiment moving back
        Serial.println("PID experiment back");
        //Motor::getRightMotor()->goAtGivenSpeed_PID(-100);
        Motor::getRightMotor()->goForGivenTimeAtGivenSpeed_PID(5000, -initial_pid_speed);
        Motor::getLeftMotor()->goForGivenTimeAtGivenSpeed_PID(5000, -initial_pid_speed);
      } else if (in_cmd.indexOf("left") > -1) { //# if we want to drive the left motor
        Serial.println("Turning left");
        Motor::getRightMotor()->moveByCounts(steps_to_turn, 50);
        Motor::getLeftMotor()->moveByCounts(steps_to_turn, -50);
      } else if(in_cmd.indexOf("right") > -1) { //# if we want to drive the right motor
        Serial.println("Turning right");
        Motor::getRightMotor()->moveByCounts(steps_to_turn, -50);
        Motor::getLeftMotor()->moveByCounts(steps_to_turn, 50);
      } else if(in_cmd.indexOf("bothb") > -1) { //# if we want to drive both motors back
        Serial.println("Driving BOTH motors back");
        Motor::getRightMotor()->goBackwardByCounts(steps_to_move);
        Motor::getLeftMotor()->goBackwardByCounts(steps_to_move);
      } else if(in_cmd.indexOf("bothf25") > -1) { //# if we want to drive both motors forward
        Serial.println("Driving BOTH motors forward");
        Motor::getRightMotor()->goForwardForGivenTimeAtGivenPower(5000, 25);
        Motor::getLeftMotor()->goForwardForGivenTimeAtGivenPower(5000, 25);
      } else if(in_cmd.indexOf("bothf100") > -1) { //# if we want to drive both motors forward
        Serial.println("Driving BOTH motors forward");
        Motor::getRightMotor()->goForwardForGivenTimeAtGivenPower(5000, 100);
        Motor::getLeftMotor()->goForwardForGivenTimeAtGivenPower(5000, 100);
      } else if(in_cmd.indexOf("bothf200") > -1) { //# if we want to drive both motors forward
        Serial.println("Driving BOTH motors forward");
        Motor::getRightMotor()->goForwardForGivenTimeAtGivenPower(5000, 200);
        Motor::getLeftMotor()->goForwardForGivenTimeAtGivenPower(5000, 200);
      } else if(in_cmd.indexOf("bothfMax") > -1) { //# if we want to drive both motors forward
        Serial.println("Driving BOTH motors forward");
        Motor::getRightMotor()->goForwardForGivenTimeAtGivenPower(5000, 255);
        Motor::getLeftMotor()->goForwardForGivenTimeAtGivenPower(5000, 255);
      } else if(in_cmd.indexOf("bothf") > -1) { //# if we want to drive both motors forward
        Serial.println("Driving BOTH motors forward");
        Motor::getRightMotor()->goForwardByCounts(steps_to_move, 35);
        Motor::getLeftMotor()->goForwardByCounts(steps_to_move, 35);
      } else if(in_cmd.indexOf("follow") > -1) { //# if we want to follow the line
        heading_pid.setUpdatesWanted(!heading_pid.isUpdatesWanted());
  
        if (heading_pid.isUpdatesWanted()) {
          heading_pid.reset();
          Serial.println("Following line");
        } else {
          Serial.println("Stopped following line");
          Motor::getRightMotor()->stopMotorAndCancelPreviousInstruction();
          Motor::getLeftMotor()->stopMotorAndCancelPreviousInstruction();
          heading_pid.reset();
        }
      } else if(in_cmd.indexOf("calib") > -1) { //# if we want to recalibrate line sensors
        Serial.println("Re-calibrating line sensors.");
        LineSensor::getRightSensor()->resetCalibration();
        LineSensor::getCentreSensor()->resetCalibration();
        LineSensor::getLeftSensor()->resetCalibration();
      } else if(in_cmd.indexOf("reinit1") > -1) {
        LineSensor::reInitTimer(5);
      } else if(in_cmd.indexOf("reinit2") > -1) {
        LineSensor::reInitTimer(10);
      } else if(in_cmd.indexOf("reinit3") > -1) {
        LineSensor::reInitTimer(25);
      }
    }
  }
}

/**
 * Prints out information about line sensors.
 * full_info : a flag of wether we want to print out all info (including wether the sensor is over line or not) or just the sensor values (good for plotting)
 */
void talk_about_it(bool full_info) {
  if(OPER_MODE == DEBUG_MODE) {
    if (full_info) {
  //    Serial.print("Weighed sensor: ");
  //    Serial.println(weighted_line_sensor());
  //    
  //    if (LineSensor::getRightSensor()->overLine()) {
  //      Serial.println("Right sensor over line");
  //    }
  //    
  //    if (LineSensor::getCentreSensor()->overLine()) {
  //      Serial.println("Centre sensor over line");
  //    }
  //  
  //    if (LineSensor::getLeftSensor()->overLine()) {
  //      Serial.println("Left sensor over line");
  //    }
  //
  //    Serial.print("Left wheel speed (line sens): ");
  //    Serial.println(LineSensor::getLeftWheelSpeed_ms());
  //
  //    Serial.print("Right wheel speed (line sens): ");
  //    Serial.println(LineSensor::getRightWheelSpeed_ms());
  //
  //    Serial.print("Left wheel speed raw (line sens): ");
  //    Serial.println(LineSensor::getLeftWheelSpeed());
  //
  //    Serial.print("Right wheel speed raw (line sens): ");
  //    Serial.println(LineSensor::getRightWheelSpeed());
  
  //    Serial.print("Left wheel speed (encoder): ");
  //    Serial.println(Encoder::getLeftEncoder()->getWheelSpeed());
  
  //    Serial.print("Right wheel speed (encoder): ");
  //    Serial.println(Encoder::getRightEncoder()->getWheelSpeed());
  //
  //    Serial.print("Right encoder: ");
  //    Serial.println(Encoder::getRightEncoder()->getPulseCount());
  //
  //    Serial.print("Left encoder: ");
  //    Serial.println(Encoder::getLeftEncoder()->getPulseCount());
  
  //    Serial.print("Right wheel speed: ");
  //    Serial.println(getRightWheelSpeed());
  
  //    Serial.print("Kinematics: ");
  //    Serial.print(Kinematics::getKinematics()->getCurrentX_raw());
  //    Serial.print(", ");
  //    Serial.print(Kinematics::getKinematics()->getCurrentY_raw());
  //    Serial.print(", ");
  //    Serial.print(Kinematics::getKinematics()->getCurrentX_mm());
  //    Serial.print(", ");
  //    Serial.print(Kinematics::getKinematics()->getCurrentY_mm());
  //    Serial.print(", ");
  //    Serial.println(Kinematics::getKinematics()->getCurrentHeading());
  
      /**
       * For charting:.
       */
  //    Serial.print(LineSensor::getLeftWheelSpeed_ms());
  //    Serial.print(", ");
  //    Serial.print(LineSensor::getRightWheelSpeed_ms());
  //    Serial.print(", ");
  //    Serial.print(Encoder::getLeftEncoder()->getWheelSpeed());
  //    Serial.print(", ");
  //    Serial.println(Encoder::getRightEncoder()->getWheelSpeed());
    }
    
  //  Serial.print(LineSensor::getLeftSensor()->getCurrentSensorValue());
  //  Serial.print(", ");
  //  Serial.print(LineSensor::getCentreSensor()->getCurrentSensorValue());
  //  Serial.print(", ");
  //  Serial.println(LineSensor::getRightSensor()->getCurrentSensorValue());
    
  //  Serial.print(", ");
  //
  //  Serial.println(LineSensor::getLeftSensor()->getBias());
  //  Serial.print(", ");
  //  Serial.print(LineSensor::getCentreSensor()->getBias());
  //  Serial.print(", ");
  //  Serial.print(LineSensor::getRightSensor()->getBias());
  }
}

/**
 * Returns current speed for the right wheel. It combines 
 * wheel speed from the line sensor which is less accurate, but
 * correctly detects 0 speed and the wheel speed from encoder
 * which is more accurate but can't tell when speed is actually 0.
 */
long getRightWheelSpeed() {
  if (LineSensor::getRightWheelSpeed() == 0) {
    return 0;
  } else {
    return Encoder::getRightEncoder()->getWheelSpeed();
  }
}

/**
 * Returns current speed for the left wheel. It combines 
 * wheel speed from the line sensor which is less accurate, but
 * correctly detects 0 speed and the wheel speed from encoder
 * which is more accurate but can't tell when speed is actually 0.
 */
long getLeftWheelSpeed() {
  if (LineSensor::getLeftWheelSpeed() == 0) {
    return 0;
  } else {
    return Encoder::getLeftEncoder()->getWheelSpeed();
  }
}

/**
 * Starts the process of going home by intiating the state machine
 * appropriately and starting with the turn. When turn is finished,
 * the state machine will be advanced to the next state and next
 * operation will be executed.
 */
void goHome() {
  /**
   * First we'll make the turn and the rest will follow in the state machine.
   */
  StateMachine::getStateMachine()->setState(StateMachine::LineFollowingStates::TURNING_TO_GO_HOME);
}

/**
 * Starts the process of looking for line.
 */
void lookForLine() {
  /**
   * Let's get to the first step of looking for line.
   */
  StateMachine::getStateMachine()->setState(StateMachine::LineFollowingStates::LOOKING_FOR_LINE_MOVING_FORWARD);
}

/**
 * Stops all motion.
 */
void stopAllMotion() {
  /**
   * Let's get to the first step of looking for line.
   */
  StateMachine::getStateMachine()->setState(StateMachine::LineFollowingStates::IDLING);
  heading_pid.setUpdatesWanted(false);
  heading_pid.reset();
  Kinematics::getKinematics()->fullStop();
}
