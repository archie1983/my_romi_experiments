#define BUZZER 6
#define RIGHT_MOTOR_DIR 15
#define LEFT_MOTOR_DIR 16
#define RIGHT_MOTOR_RUN 9
#define LEFT_MOTOR_RUN 10

#define BAUD_RATE 9600

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_RUN, OUTPUT);
  pinMode(LEFT_MOTOR_RUN, OUTPUT);

  //Start a serial connection
  Serial.begin(BAUD_RATE);

  // Wait for stable connection, report reset.
  delay(1000);
  Serial.println("***RESET***");
}

// the loop function runs over and over again forever
void loop() {
  act_on_commands();
}

/**
 * Reads a command from the Serial connection and acts on it
 */
void act_on_commands() {
  //This line checks whether there is anything to read
  if ( Serial.available() ) {
      char inChar = Serial.read(); //This reads one byte
      switch (inChar) {
        case 'f':
          Serial.println("Going forward");
          go_forward();
          break;
        case 'l':
          Serial.println("Flashing LED");
          flash_led();
          break;
        case 'p':
          Serial.println("Playing tone");
          play_tone(10);
          break;
        default:
          break;
      }
  }
}

void flash_led() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

/**
 * Plays a tone
 */
void play_tone(int volume) {
    analogWrite(BUZZER, volume);
    delay(1000);
    analogWrite(BUZZER, 0);
}

/**
 * Moves the motors to go forward
 */
void go_forward() {
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  digitalWrite(LEFT_MOTOR_DIR, HIGH);

  analogWrite(RIGHT_MOTOR_RUN, 25);
  analogWrite(LEFT_MOTOR_RUN, 25);

  delay(1000);

  analogWrite(RIGHT_MOTOR_RUN, 0);
  analogWrite(LEFT_MOTOR_RUN, 0);  
  digitalWrite(RIGHT_MOTOR_DIR, LOW);
  digitalWrite(LEFT_MOTOR_DIR, LOW);
}
