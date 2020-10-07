#include "pin_names.h"
#include "ir_line_sensor.h"

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

  LineSensor line_left(LINE_LEFT_PIN);
  LineSensor line_centre(LINE_CENTRE_PIN);
  LineSensor line_right(LINE_RIGHT_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:

}
