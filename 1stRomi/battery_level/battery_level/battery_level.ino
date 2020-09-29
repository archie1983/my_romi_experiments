#define BUZZER 6
#define BAUD_RATE 9600
#define BATTLEV 19
#define ADC_REF_VAL 5/(2^10) //# The AREF pin is 5V and the ADC is 10-bit, so a value of 0 means 0V, and a value of 2^10
// means 5V. So one unit of the ADC returned value will be (5/(2^10)).
#define VOLT_DIV 3 // we also know that there is a voltage divider on the VSW pin, so whatever we read will be just 1/3 of the real voltage.


void setup() {
  // put your setup code here, to run once:
  pinMode(BATTLEV, INPUT);

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
int batt_level = 0;
int analog_val = 0;

void loop() {
  // Implement a millis() or micros() task block
  // to toggle the state of the buzzer.

    act_on_commands();

    // Get how much time has passed right now.
    time_now = millis();

    // Work out how many milliseconds have gone passed by subtracting
    // our two timestamps.  time_now will always be bigger than the
    // time_of_read (except when millis() overflows after 50 days).
    elapsed_time = time_now - last_timestamp;

    // See if 10000 milliseconds have elapsed
    // If not, this block is skipped.
    if(elapsed_time > batt_lev_measure_time) {

        // Since 10000ms elapsed, we overwrite our last_timestamp with 
        // the new current time so that another 10000ms is needed to pass.
        // !! NOT RESETING THE TIME STAMP IS AN EXTREMELY COMMON BUG !!
        last_timestamp = millis();

        analog_val = analogRead(BATTLEV);
        batt_level = analog_val * ADC_REF_VAL * VOLT_DIV;
        Serial.println(batt_level);
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
        case '1':
          Serial.println("Pitch 1");
          //pwm_cnt = 1;
          break;
        case '2':
          Serial.println("Pitch 2");
          //pwm_cnt = 5;
          break;
        case '3':
          Serial.println("Pitch 3");
          //pwm_cnt = 10;
          break;
        case '4':
          Serial.println("Silent");
          //pwm_cnt = 1000;
          break;
        default:
          break;
      }
  }
}
