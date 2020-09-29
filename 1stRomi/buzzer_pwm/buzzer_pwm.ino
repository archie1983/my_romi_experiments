#define BUZZER 6
#define BAUD_RATE 9600

void setup() {
  // put your setup code here, to run once:
  pinMode(BUZZER, OUTPUT);

  //Start a serial connection
  Serial.begin(BAUD_RATE);

  // Wait for stable connection, report reset.
  delay(1000);
  Serial.println("***RESET***");
}

bool buzz_on = false;
unsigned long time_now = 0;
unsigned long elapsed_time = 0;
unsigned long last_timestamp = 0;

void loop() {
  // Implement a millis() or micros() task block
  // to toggle the state of the buzzer.


    // Get how much time has passed right now.
    time_now = millis();     

    // Work out how many milliseconds have gone passed by subtracting
    // our two timestamps.  time_now will always be bigger than the
    // time_of_read (except when millis() overflows after 50 days).
    elapsed_time = time_now - last_timestamp;



    // See if 10000 milliseconds have elapsed
    // If not, this block is skipped.
    if(elapsed_time > 3) {

        // Since 10000ms elapsed, we overwrite our last_timestamp with 
        // the new current time so that another 10000ms is needed to pass.
        // !! NOT RESETING THE TIME STAMP IS AN EXTREMELY COMMON BUG !!
        last_timestamp = millis();

        if (buzz_on) {
          digitalWrite(BUZZER, LOW);
          buzz_on = false;
        } else {
          digitalWrite(BUZZER, HIGH);
          buzz_on = true;
        }

    }

    // Code outside the above if{} will run on every loop!
    // Therefore code here is no longer stopped waiting for a delay()
}
