#include "pin_names_and_constants.h"
#include "ir_line_sensor.h"
#include "motor.h"

void setup() {
  // Start Serial monitor and print "reset"
  // so we know if the board is reseting
  // unexpectedly.
  Serial.begin(9600);
  Serial.setTimeout(100);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("***RESET***");
}

/**
 * A flag of whether we want to follow the line or not.
 */
bool follow = false;

void loop() {
  //talk_about_it();
  
  /*
   * If that's what user wants, follow the line
   */
  if (follow) {
    //bang2x();
    bang2x_w();
    //bang2x_w_timed();
    //bang2x_w_variable_power();
  }
  
  act_on_commands();
}

/**
 * Weighted line sensor bang-bang that drive wheels at non-constant power.
 */
void bang2x_w_variable_power() {
  delay(120);
  byte counts_to_run = 10;
  byte counts_to_turn = 10;
  byte power_to_go_straight = 36;
  
  float wls = weighted_line_sensor();
  int power_right = wls * 255;
  int power_left = wls * 255 * -1;

  bool sensor_r = LineSensor::getRightSensor()->overLine();
  bool sensor_c = LineSensor::getCentreSensor()->overLine();
  bool sensor_l = LineSensor::getLeftSensor()->overLine();

  byte number_of_sensors_over_line = 0;
  if (sensor_r) number_of_sensors_over_line++;
  if (sensor_c) number_of_sensors_over_line++;
  if (sensor_l) number_of_sensors_over_line++;

  talk_about_it();

  if (abs(wls) > 0.1) { //# this only give chaotic movement
  //if ((sensor_l || sensor_c || sensor_r) && abs(wls) > 0.06) {
  //if (number_of_sensors_over_line < 3 && number_of_sensors_over_line > 0 && abs(wls) > 0.06) {
    /*
     * turning
     */
    Serial.print("Turning ");
    /*
     * If left motor is being driven stronger than the right, then we're turning right
     * and vice versa.
     */
    Serial.print(power_right < power_left ? "right : " : "left : ");
    Serial.print(power_right);
    Serial.print(" # ");
    Serial.print(power_left);
    Serial.print(" wls=");
    Serial.println(wls);
    Motor::getRightMotor()->moveByCounts(counts_to_turn, power_right);
    Motor::getLeftMotor()->moveByCounts(counts_to_turn, power_left);
  } else {
    /*
     * going straight 
     */
    Serial.println("Going straight");
    Motor::getRightMotor()->moveByCounts(counts_to_run, power_to_go_straight);
    Motor::getLeftMotor()->moveByCounts(counts_to_run, power_to_go_straight);
  }
}

/**
 * Weighted line sensor bang-bang with timed motor runs (not counting pulses)
 */
void bang2x_w_timed() {
  byte time_to_run = 10;
  byte time_to_turn = 20;
  byte power_to_go_straight = 36;
  byte power_to_turn = 36;
  
  float wls = 0.0;

  bool sensor_r = LineSensor::getRightSensor()->overLine();
  bool sensor_c = LineSensor::getCentreSensor()->overLine();
  bool sensor_l = LineSensor::getLeftSensor()->overLine();

  if (sensor_l || sensor_c || sensor_r) {
    wls = weighted_line_sensor();
  }

  if(wls < -0.1) {
    Motor::getLeftMotor()->goForwardForGivenTimeAtGivenPower(time_to_turn, power_to_turn);
    Motor::getRightMotor()->stopMotorAndCancelPreviousInstruction();
    Serial.println("Turn right");
  } else if(wls > 0.1) {
    Motor::getRightMotor()->goForwardForGivenTimeAtGivenPower(time_to_turn, power_to_turn);
    Motor::getLeftMotor()->stopMotorAndCancelPreviousInstruction();
    Serial.println("Turn left");
  } else {
    Motor::getRightMotor()->goForwardForGivenTimeAtGivenPower(time_to_run, power_to_go_straight);
    Motor::getLeftMotor()->goForwardForGivenTimeAtGivenPower(time_to_run, power_to_go_straight);
    Serial.println("Go straight");
  }
}

/**
 * Weighted line sensor bang-bang
 */
void bang2x_w() {
  int steps_to_move = 10;
  int steps_to_turn = 20;
  float wls = 0.0;

  bool sensor_r = LineSensor::getRightSensor()->overLine();
  bool sensor_c = LineSensor::getCentreSensor()->overLine();
  bool sensor_l = LineSensor::getLeftSensor()->overLine();

  if (sensor_l || sensor_c || sensor_r) {
    wls = weighted_line_sensor();
  }

  if(wls < -0.1) {
    Motor::getLeftMotor()->goForwardByCounts(steps_to_turn);
    Motor::getRightMotor()->stopMotorAndCancelPreviousInstruction();
    Serial.println("Turn right");
  } else if(wls > 0.1) {
    Motor::getRightMotor()->goForwardByCounts(steps_to_turn);
    Motor::getLeftMotor()->stopMotorAndCancelPreviousInstruction();
    Serial.println("Turn left");
  } else {
    Motor::getRightMotor()->goForwardByCounts(steps_to_move);
    Motor::getLeftMotor()->goForwardByCounts(steps_to_move);
    Serial.println("Go straight");
  }
}

/**
 * Bang-bang controller. A simple reaction to the current readings of the sensor.
 */
void bang2x() {
  delay(50);
  bool sensor_r = LineSensor::getRightSensor()->overLine();
  bool sensor_c = LineSensor::getCentreSensor()->overLine();
  bool sensor_l = LineSensor::getLeftSensor()->overLine();
  /*
   * Ok, so we have 3 sensors and each of them can either be over the line or off line. That's 8 combinations.
   * 1) If no sensors are over the line, then we need to find the line. Drive forward for now very slowly.
   * 2) If left sensor only is over the line, then we want to turn more left
   * 3) If only centre sensor is over the line, then carry on straight. This is an error state and shouldn't normally happen.
   * 4) If Left and and centre sensor is over the line, then still turn more left, but less so or slower.
   * 5) If right sensor only is over the line, then we want to turn more right.
   * 6) If both right and left sensors are over the line, but not the centre one, then just go forward, this is another error state and shouldn't happen normally.
   * 7) If right and centre sensors are over the line, then still turn more right, but less so or slower.
   * 8) If all sensors are over the line, then carry on straight, we're doing a good job.
   */
  
  if (!sensor_r && !sensor_c && !sensor_l) { //# rule 1
    Motor::getRightMotor()->goForwardByCounts(5);
    Motor::getLeftMotor()->goForwardByCounts(5);
    Serial.println("rule1");
  } else if(!sensor_r && !sensor_c && sensor_l) {//# rule 2
    Motor::getRightMotor()->goForwardByCounts(25);
    Motor::getLeftMotor()->stopMotorAndCancelPreviousInstruction();
    Serial.println("rule2");
  } else if(!sensor_r && sensor_c && !sensor_l) {//# rule 3
    Motor::getRightMotor()->goForwardByCounts(25);
    Motor::getLeftMotor()->goForwardByCounts(25);
    Serial.println("rule3");
  } else if(!sensor_r && sensor_c && sensor_l) {//# rule 4
    Motor::getRightMotor()->goForwardByCounts(12);
    Motor::getLeftMotor()->stopMotorAndCancelPreviousInstruction();
    Serial.println("rule4");
  } else if(sensor_r && !sensor_c && !sensor_l) {//# rule 5
    Motor::getLeftMotor()->goForwardByCounts(25);
    Motor::getRightMotor()->stopMotorAndCancelPreviousInstruction();
    Serial.println("rule5");
  } else if(sensor_r && !sensor_c && sensor_l) {//# rule 6
    Motor::getRightMotor()->goForwardByCounts(25);
    Motor::getLeftMotor()->goForwardByCounts(25);
    Serial.println("rule6");
  } else if(sensor_r && sensor_c && !sensor_l) {//# rule 7
    Motor::getLeftMotor()->goForwardByCounts(12);
    Motor::getRightMotor()->stopMotorAndCancelPreviousInstruction();
    Serial.println("rule7");
  } else if(sensor_r && sensor_c && sensor_l) {//# rule 8
    Motor::getRightMotor()->goForwardByCounts(25);
    Motor::getLeftMotor()->goForwardByCounts(25);
    Serial.println("rule8");
  }
}

/**
 * Calculates the weighed line sensor value as described in exercise 3 and returns it.
 */
float weighted_line_sensor() {
  unsigned int left_value = LineSensor::getLeftSensor()->getCurrentSensorValue();
  unsigned int centre_value = LineSensor::getCentreSensor()->getCurrentSensorValue();
  unsigned int right_value = LineSensor::getRightSensor()->getCurrentSensorValue();
  
  float total_magnitude = left_value + centre_value + right_value;

  if (total_magnitude > 0) {
    float left_proportional = left_value / total_magnitude;
    float right_proportional = right_value / total_magnitude;
    
    return left_proportional - right_proportional;
  } else {
    return 0.0;
  }
}

/**
 * Reads a command from the Serial connection and acts on it
 */
void act_on_commands() {
  int steps_to_move = 200;
  int steps_to_turn = 100;
  int time_to_turn = 200;
  //This line checks whether there is anything to read
  if ( Serial.available() ) {
    String in_cmd = Serial.readString();

    if (in_cmd.indexOf("left") > -1) { //# if we want to drive the left motor
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
    } else if(in_cmd.indexOf("bothf") > -1) { //# if we want to drive both motors forward
      Serial.println("Driving BOTH motors forward");
      Motor::getRightMotor()->goForwardByCounts(steps_to_move, 35);
      Motor::getLeftMotor()->goForwardByCounts(steps_to_move, 35);
    } else if(in_cmd.indexOf("follow") > -1) { //# if we want to follow the line
      follow = !follow;

      if (follow) {
        Serial.println("Following line");
      } else {
        Serial.println("Stopped following line");
      }
    } else if(in_cmd.indexOf("calib") > -1) { //# if we want to recalibrate line sensors
      Serial.println("Re-calibrating line sensors.");
      LineSensor::getRightSensor()->resetCalibration();
      LineSensor::getCentreSensor()->resetCalibration();
      LineSensor::getLeftSensor()->resetCalibration();
    }
  }
}

void talk_about_it() {
  //delay(300);

//  Serial.print("Weighed sensor: ");
  Serial.print(weighted_line_sensor());
  Serial.print(", ");
  
  if (LineSensor::getRightSensor()->overLine()) {
    Serial.println("Right sensor over line");
  }
  
  if (LineSensor::getCentreSensor()->overLine()) {
    Serial.println("Centre sensor over line");
  }

  if (LineSensor::getLeftSensor()->overLine()) {
    Serial.println("Left sensor over line");
  }
  
  Serial.print(LineSensor::getLeftSensor()->getCurrentSensorValue());
  Serial.print(", ");
  Serial.print(LineSensor::getCentreSensor()->getCurrentSensorValue());
  Serial.print(", ");
  Serial.println(LineSensor::getRightSensor()->getCurrentSensorValue());
  
//  Serial.print(", ");
//
//  Serial.println(LineSensor::getLeftSensor()->getBias());
//  Serial.print(", ");
//  Serial.print(LineSensor::getCentreSensor()->getBias());
//  Serial.print(", ");
//  Serial.print(LineSensor::getRightSensor()->getBias());
}
