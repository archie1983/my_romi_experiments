#include "pin_names_and_constants.h"
#include "ir_line_sensor.h"
#include "motor.h"

LineSensor line_left(LINE_LEFT_PIN);
LineSensor line_centre(LINE_CENTRE_PIN);
LineSensor line_right(LINE_RIGHT_PIN);

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

void loop() {
  // put your main code here, to run repeatedly:
  delay(300);

  if (line_right.overLine()) {
    Serial.println("Right sensor over line");
  }
  
  if (line_centre.overLine()) {
    Serial.println("Centre sensor over line");
  }

  if (line_left.overLine()) {
    Serial.println("Left sensor over line");
  }

  Serial.print(line_right.getCurrentSensorValue());
  Serial.print(", ");
  Serial.print(line_centre.getCurrentSensorValue());
  Serial.print(", ");
  Serial.print(line_left.getCurrentSensorValue());
  Serial.println("");

  act_on_commands();
}

/**
 * Reads a command from the Serial connection and acts on it
 */
void act_on_commands() {
  //This line checks whether there is anything to read
  if ( Serial.available() ) {
    String in_cmd = Serial.readString();

    if (in_cmd.indexOf("left") > -1) { //# if we want to drive the LED
      Serial.println("Driving LEFT motor");
      Motor::getLeftMotor()->goForward_1Second();
    } else if(in_cmd.indexOf("right") > -1) { //# if we want to drive the motor
      Serial.println("Driving RIGHT motor");
      Motor::getRightMotor()->goForward_1Second();
    }
  }
}
