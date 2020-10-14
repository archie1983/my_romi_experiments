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
  // put your main code here, to run repeatedly:
//  delay(300);

  Serial.print("Weighed sensor: ");
  Serial.println(weighted_line_sensor());
  
//
//  if (LineSensor::getRightSensor()->overLine()) {
//    Serial.println("Right sensor over line");
//  }
//  
//  if (LineSensor::getCentreSensor()->overLine()) {
//    Serial.println("Centre sensor over line");
//  }
//
//  if (LineSensor::getLeftSensor()->overLine()) {
//    Serial.println("Left sensor over line");
//  }
//
//  Serial.print(LineSensor::getRightSensor()->getCurrentSensorValue());
//  Serial.print(", ");
//  Serial.print(LineSensor::getRightSensor()->getBias());
//  Serial.print(", ");
//  Serial.print(LineSensor::getCentreSensor()->getCurrentSensorValue());
//  Serial.print(", ");
//  Serial.print(LineSensor::getCentreSensor()->getBias());
//  Serial.print(", ");
//  Serial.print(LineSensor::getLeftSensor()->getCurrentSensorValue());
//  Serial.print(", ");
//  Serial.println(LineSensor::getLeftSensor()->getBias());

  /*
   * If that's what user wants, follow the line
   */
  if (follow) {
    //bang2x();#
    bang2x_w();
  }
  
  act_on_commands();
}

/**
 * Weighted line sensor bang-bang that drive wheels at non-constant power.
 */
void bang2x_w_variable_power() {
  float wls = weighted_line_sensor();
  int power_right = wls * 255 * -1;
  int power_left = wls * 255;
  
  Motor::getRightMotor()->goForwardByCounts(5);
  Motor::getLeftMotor()->goForwardByCounts(5);
}

/**
 * Weighted line sensor bang-bang
 */
void bang2x_w() {
  float wls = weighted_line_sensor();

  if(wls > 0.1) {
    Motor::getLeftMotor()->goForwardByCounts(5);
    Motor::getRightMotor()->stopMotorAndCancelPreviousInstruction();
  } else if(wls < -0.1) {
    Motor::getRightMotor()->goForwardByCounts(5);
    Motor::getLeftMotor()->stopMotorAndCancelPreviousInstruction();
  } else {
    Motor::getRightMotor()->goForwardByCounts(5);
    Motor::getLeftMotor()->goForwardByCounts(5);
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
  int steps_to_move = 10;
  //This line checks whether there is anything to read
  if ( Serial.available() ) {
    String in_cmd = Serial.readString();

    if (in_cmd.indexOf("left") > -1) { //# if we want to drive the left motor
      Serial.println("Driving LEFT motor");
      Motor::getLeftMotor()->goBackwardByCounts(steps_to_move, 35);
    } else if(in_cmd.indexOf("right") > -1) { //# if we want to drive the right motor
      Serial.println("Driving RIGHT motor");
      Motor::getRightMotor()->goBackwardByCounts(steps_to_move, 35);
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
