
#define RIGHT_MOTOR_DIR 15
#define LEFT_MOTOR_DIR 16
#define RIGHT_MOTOR_RUN 9
#define LEFT_MOTOR_RUN 10
#define LED 13
#define BAUD_RATE 9600

void setup() {
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_RUN, OUTPUT);
  pinMode(LEFT_MOTOR_RUN, OUTPUT);

  //Start a serial connection
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(100);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("***RESET***");
}

void loop() {
  act_on_commands();
}

/**
 * If this is true, then we want to drive the LED with the PWM instead
 * of a motor (good to keep things quiet when my kids sleep).
 */
bool led_or_motor = true;

/**
 * Reads a command from the Serial connection and acts on it
 */
void act_on_commands() {
  //This line checks whether there is anything to read
  if ( Serial.available() ) {
    String in_cmd = Serial.readString();

    if (in_cmd.indexOf("led") > -1) { //# if we want to drive the LED
      led_or_motor = true;
      Serial.println("Driving LED");
    } else if(in_cmd.indexOf("motor") > -1) { //# if we want to drive the motor
      led_or_motor = false;
      Serial.println("Driving MOTORS");
    } else { //# this should only be an integer value that we want to drive our LED or motor with.
      if (led_or_motor){
        analogWrite(LED, in_cmd.toInt());
      } else {
        turn_motors(in_cmd.toInt());
      }
    }
  }
}

/**
 * Moves the motors to go forward or backward with a given PWM. The motors will be
 * switched off after 1s.
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
void turn_motors(int power) {
  if (power > 0) {
    digitalWrite(RIGHT_MOTOR_DIR, HIGH);
    digitalWrite(LEFT_MOTOR_DIR, HIGH);
  } else {
    digitalWrite(RIGHT_MOTOR_DIR, LOW);
    digitalWrite(LEFT_MOTOR_DIR, LOW);
  }

  /**
   * If we don't take abs value here, then it looks like values greater than 255 overflow
   * and negative values are treated as unsigned byte, rather than a negative value, 
   * so -1 will come out as 255. Looks like analogWrite function takes in a paramater of unsigned byte.
   */
  analogWrite(RIGHT_MOTOR_RUN, abs(power));
  analogWrite(LEFT_MOTOR_RUN, abs(power));

  delay(1000);

  analogWrite(RIGHT_MOTOR_RUN, 0);
  analogWrite(LEFT_MOTOR_RUN, 0);  
  digitalWrite(RIGHT_MOTOR_DIR, LOW);
  digitalWrite(LEFT_MOTOR_DIR, LOW);
}
