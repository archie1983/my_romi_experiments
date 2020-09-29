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

  play_tone(10);
  //play_tone(200);

  go_forward();

  //Start a serial connection
  Serial.begin(BAUD_RATE);

  // Wait for stable connection, report reset.
  delay(1000);
  Serial.println("***RESET***");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}


void play_tone(int volume) {
    analogWrite(BUZZER, volume);
    delay(1000);
    analogWrite(BUZZER, 0);
}

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
