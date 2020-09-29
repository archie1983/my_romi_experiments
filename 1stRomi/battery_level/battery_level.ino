#define BUZZER 6
#define BAUD_RATE 9600
#define BATTLEV 19
#define ADC_REF_VAL (5.0/1024.0) //# The AREF pin is 5V and the ADC is 10-bit, so a value of 0 means 0V, and a value of 2^10
// means 5V. So one unit of the ADC returned value will be (5/(2^10)).
#define VOLT_DIV 3 // we also know that there is a voltage divider on the VSW pin, so whatever we read will be just 1/3 of the real voltage.

//# for the motor:
#define RIGHT_MOTOR_DIR 15
#define LEFT_MOTOR_DIR 16
#define RIGHT_MOTOR_RUN 9
#define LEFT_MOTOR_RUN 10

void setup() {
  // put your setup code here, to run once:
  pinMode(BATTLEV, INPUT);

  //# motor
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

unsigned long time_now = 0;
unsigned long elapsed_time = 0;
unsigned long last_timestamp = 0;
unsigned int batt_lev_measure_time = 50; // we'll measure battery every 50 ms.
unsigned int last_timestamp_measurement = 0;
double batt_level = 0;
double analog_val = 0;
unsigned int motor_on_off = 2000; // we'll run motors every 2s and then switch off for 2s.
unsigned int last_timestamp_motors = 0;

boolean motors_running = false;
boolean motors_must_run = false;

void loop() {
  // Implement a millis() or micros() task block
  // to toggle the state of the buzzer.

    act_on_commands();

    // Get how much time has passed right now.
    time_now = millis();

    // See if 10000 milliseconds have elapsed
    // If not, this block is skipped.
    if(time_now - last_timestamp_measurement > batt_lev_measure_time) {

        // Since 10000ms elapsed, we overwrite our last_timestamp with 
        // the new current time so that another 10000ms is needed to pass.
        // !! NOT RESETING THE TIME STAMP IS AN EXTREMELY COMMON BUG !!
        last_timestamp_measurement = millis();

        analog_val = analogRead(BATTLEV);
        batt_level = analog_val * ADC_REF_VAL * VOLT_DIV;
        Serial.println(batt_level);
    }

    if (time_now - last_timestamp_motors > motor_on_off && motors_must_run) {
      if (motors_running) {
        stop_motors();
        motors_running = false;
      } else {
        start_motors();
        motors_running = true;
      }
      last_timestamp_motors = millis();
    } else if(!motors_must_run) {
      stop_motors();
    }

    // Code outside the above if{} will run on every loop!
    // Therefore code here is no longer stopped waiting for a delay()
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
          Serial.println("Motor Forward");
          go_forward();
          break;
        case 's':
          Serial.println("Toggling motor run state");
          motors_must_run = !motors_must_run;
          break;
        default:
          break;
      }
  }
}

/**
 * Starts the motors to go forward
 */
void start_motors() {
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  digitalWrite(LEFT_MOTOR_DIR, HIGH);

  analogWrite(RIGHT_MOTOR_RUN, 250);
  analogWrite(LEFT_MOTOR_RUN, 250);
}

/**
 * Stops motors.
 */
void stop_motors() {
  analogWrite(RIGHT_MOTOR_RUN, 0);
  analogWrite(LEFT_MOTOR_RUN, 0);  
  digitalWrite(RIGHT_MOTOR_DIR, LOW);
  digitalWrite(LEFT_MOTOR_DIR, LOW);
}

/**
 * Moves the motors to go forward
 */
void go_forward() {
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  digitalWrite(LEFT_MOTOR_DIR, HIGH);

  analogWrite(RIGHT_MOTOR_RUN, 250);
  analogWrite(LEFT_MOTOR_RUN, 250);

  delay(1000);

  analogWrite(RIGHT_MOTOR_RUN, 0);
  analogWrite(LEFT_MOTOR_RUN, 0);  
  digitalWrite(RIGHT_MOTOR_DIR, LOW);
  digitalWrite(LEFT_MOTOR_DIR, LOW);
}
