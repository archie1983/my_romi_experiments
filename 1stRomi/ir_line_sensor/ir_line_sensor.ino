#define LINE_LEFT_PIN A4
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A2

void setup() {
  pinMode(LINE_LEFT_PIN, INPUT);
  pinMode(LINE_CENTRE_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN, INPUT);

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

void loop() 
{
  // To store result.
  int l_value; // left sensor
  int c_value; // centre sensor
  int r_value; // right sensor

  // Read analog voltages
  l_value = analogRead( LINE_LEFT_PIN );
  c_value = analogRead( LINE_CENTRE_PIN );
  r_value = analogRead( LINE_RIGHT_PIN );

  // To send data back to your computer.
  // You can open either Serial monitor or plotter.
  Serial.print( l_value );
  Serial.print( ", " );
  Serial.print( c_value );
  Serial.print( ", " );
  Serial.print( r_value );
  Serial.print( "\n" );

  delay(50);
}
