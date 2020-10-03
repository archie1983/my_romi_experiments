//#define LED 6 //# this is for buzzer
#define LED 13 //# this is for LED
#define RIGHT_ENCODER_XOR 7
#define RIGHT_ENCODER_PHASE_B 23

// Global variable to remember the
// on/off state of the LED.  
volatile boolean led_state = false; //# needs to be volatile because it will be changed by the ISR and not the main loop.
volatile int right_encoder_pulse_cnt = 0;

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the 
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR( TIMER3_COMPA_vect ) {

  // Invert LED state.
  led_state = !led_state;

  // Enable/disable LED
  digitalWrite(LED, led_state);

}

/**
 * Right wheel encoder pin phaseA and phaseB XOR-ed. We can use that as a detection for wheel position change.
 * We'll need to read one of the original phaseB or phaseB pins with the digitalRead() function to decide whether
 * the movement was forward or backward.
 */
ISR( INT6_vect ) {
 // ....
 // ...ISR code - keep it short!...
 // ....
 right_encoder_pulse_cnt++;
}

/**
 * Left wheel encoder pin phaseA and phaseB XOR-ed. We can use that as a detection for wheel position change.
 * We'll need to read one of the original phaseB or phaseB pins with the digitalRead() function to decide whether
 * the movement was forward or backward.
 */
ISR( PCINT4_vect ) {
 // ....
 // ...ISR code - keep it short!...
 // ....
}

void setup() {
  pinMode(LED, OUTPUT);

  // Start Serial monitor and print "reset"
  // so we know if the board is reseting
  // unexpectedly.
  Serial.begin(9600);
  Serial.setTimeout(100);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("***RESET***");
  
  setupTimer3(25);

  // These two function set up the pin
  // change interrupts for the encoders.
  // If you want to know more, find them
  // at the end of this file.  
  setupRightEncoder();
  //setupEncoder1();
}

void loop() {
  // put your main code here, to run repeatedly:
  act_on_commands();
  Serial.println(right_encoder_pulse_cnt);
  delay(50);
}

/**
 * If:
 * 
 * Channel A goes from LOW to HIGH
 * and Channel B is LOW
 * You are going CLOCKWISE!
 * Or if:
 * 
 * Channel A goes from LOW to HIGH
 * and Channel B is HIGH
 * You are going COUNTER-CLOCKWISE!
 */

void setupRightEncoder() {
  // Setup pins for right encoder
  pinMode( RIGHT_ENCODER_XOR, INPUT );
  pinMode( RIGHT_ENCODER_PHASE_B, INPUT );

  // Now to set up PE6 as an external interupt (INT6), which means it can
  // have its own dedicated ISR vector INT6_vector

  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit low, preserve other bits
  EIMSK = EIMSK & ~(1<<INT6);
  //EIMSK = EIMSK & B1011111; // Same as above.

  // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
  // Used to set up INT6 interrupt
  EICRB |= (1 << ISC60) | (1 << ISC61);  // using header file names, push 1 to bit ISC60 and ISC61 to get triggered on rising edge only
  //EICRB |= B00010000; // does same as above

  // Page 90, 11.1.4 External Interrupt Flag Register – EIFR
  // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
  EIFR |= ( 1 << INTF6 );
  //EIFR |= B01000000;  // same as above

  // Now that we have set INT6 interrupt up, we can enable
  // the interrupt to happen
  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit high, preserve other bits
  EIMSK |= ( 1 << INT6 );
  //EIMSK |= B01000000; // Same as above
}

/**
 * Reads a command from the Serial connection and acts on it
 */
void act_on_commands() {
  //This line checks whether there is anything to read
  if ( Serial.available() ) {
    String in_cmd = Serial.readString();

    if (in_cmd.indexOf("led") > -1) { //# if we want to drive the LED
      Serial.println("Driving LED");
    } else if(in_cmd.indexOf("motor") > -1) { //# if we want to drive the motor
      Serial.println("Driving MOTORS");
    } else { //# this should only be an integer value that we want to drive our LED or motor with.
      setupTimer3(in_cmd.toInt());
    }
  }
}

/**
 * Routine to setupt timer3 to run 
 * 
 * @desired_frequency - the desired frequency for the timer to trigger the ISR at.
 */
void setupTimer3(long desired_frequency) {

  // disable global interrupts
  cli();          

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual. 
  //TCCR3B = TCCR3B | (1 << CS32);
  //TCCR3B = TCCR3B | (1 << CS32) | (1 << CS30); //# setting pre-scaler to 1024 so that we get 16kHz clock 
                                               //# which we'll count to get 25Hz flash
  
  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).
  //OCR3A = 31250;
  //OCR3A = 625; //1; //# setting the counter to 625 to achieve 25Hz flash.

  /**
   * So we have 8 options for the pre-scaler and we have a 16bit counter, so up to 65536.
   * 
   * With the following python expression we can get available timer3 trigger frequencies for each
   * pre-scaled frequency. The example is given for 16KHz pre-scaled frequency:
   * 
   * [int(16000 / a) for a in range(1, 65537) if 16000 % a == 0]
   * 
   * 000: timer stopped
   * 001: no pre-scaling, so running at 16MHz, giving us the following choice of frequencies: [16000000, 8000000, 4000000, 3200000, 2000000, 1600000, 1000000, 800000, 640000, 500000, 400000, 320000, 250000, 200000, 160000, 128000, 125000, 100000, 80000, 64000, 62500, 50000, 40000, 32000, 31250, 25600, 25000, 20000, 16000, 15625, 12800, 12500, 10000, 8000, 6400, 6250, 5120, 5000, 4000, 3200, 3125, 2560, 2500, 2000, 1600, 1280, 1250, 1024, 1000, 800, 640, 625, 512, 500, 400, 320, 256, 250]
   * 010: F / 8 pre-scaler, so running at 2MHz, giving us: [2000000, 1000000, 500000, 400000, 250000, 200000, 125000, 100000, 80000, 62500, 50000, 40000, 31250, 25000, 20000, 16000, 15625, 12500, 10000, 8000, 6250, 5000, 4000, 3200, 3125, 2500, 2000, 1600, 1250, 1000, 800, 640, 625, 500, 400, 320, 250, 200, 160, 128, 125, 100, 80, 64, 50, 40, 32]
   * 011: F / 64. so running at 250KHz, giving us: [250000, 125000, 62500, 50000, 31250, 25000, 15625, 12500, 10000, 6250, 5000, 3125, 2500, 2000, 1250, 1000, 625, 500, 400, 250, 200, 125, 100, 80, 50, 40, 25, 20, 16, 10, 8, 5, 4]
   * 100: F / 256, so running at 62.5KHz, giving us: [62500, 31250, 15625, 12500, 6250, 3125, 2500, 1250, 625, 500, 250, 125, 100, 50, 25, 20, 10, 5, 4, 2, 1]
   * 101: F/ 1024, so running at 16KHz, giving us: [16000, 8000, 4000, 3200, 2000, 1600, 1000, 800, 640, 500, 400, 320, 250, 200, 160, 128, 125, 100, 80, 64, 50, 40, 32, 25, 20, 16, 10, 8, 5, 4, 2, 1]
   * 110: External clock, falling edge
   * 111: External clock, rising edge.
   * 
   * We could just create 5 arrays of frequencies and search through them,
   * but that would waste precious memory on a micro. So perhaps we'll better be claculating on the fly instead of stuffing RAM with these 32 bit numbers - and
   * it will have to be 32 bit numbers, because the biggest one is 16 000 000. Another thing to remember is that our ATMEL device is only an 8-bit micro and
   * the only reason why we can operate with 16-bit and even 32-bit numbers (and I think even 64-bit) is the fact that our compiler is clever enough to implement
   * the byte shifting operations in the code for us behind the scenes.
   * 
   * Also, these are just whole number frequencies. We could just divide the pre-scaled frequency with the desired frequency and we'll get the counter value
   * that we need to count up to. And perhaps go through all the pre-scaler frequencies and see which of the divisions is the cleanest (the least value after
   * the decimal point).
   */
  unsigned long pre_scaled_frequencies[] = {16000000, 2000000, 250000, 62500, 16000};
  unsigned short cnt = 0;
  
  /**
   * Here we'll keep the closest frequency we've gotten so far while checking them.
   */
  double best_candidate_frequency = 0.0;
  unsigned short best_candidate_pre_scaler = 0;
  unsigned int best_candidate_counter = 0;
  double current_candidate_frequency = 0.0;
  unsigned int current_candidate_counter = 0;

  for (cnt = 0; cnt < sizeof(pre_scaled_frequencies) / sizeof(long); cnt++) {
    /**
     * So if we divide the pre-scaled frequency with the desired frequency, we'll get the counter value,
     * but it of course has to be no more than 65536 and it has to be a whole number.
     */
    current_candidate_counter = round((double)pre_scaled_frequencies[cnt] / (double)desired_frequency);
    current_candidate_frequency = (double)pre_scaled_frequencies[cnt] / current_candidate_counter;

    Serial.print(" current_candidate_frequency: ");
    Serial.print(current_candidate_frequency);
    Serial.print(" current_candidate_counter: ");
    Serial.println(current_candidate_counter);

    //# Minimising the deviation from the desired frequency accross different pre-scale values
    if (abs(current_candidate_frequency - desired_frequency) < abs(best_candidate_frequency - desired_frequency) && current_candidate_counter <= 65536) {

//      Serial.print(current_candidate_frequency);
//      Serial.print(" - ");
//      Serial.print(desired_frequency);
//      Serial.print(" > ");
//      Serial.print(best_candidate_frequency);
//      Serial.print(" - ");
//      Serial.println(desired_frequency);
      
      best_candidate_frequency = current_candidate_frequency;
      best_candidate_pre_scaler = cnt;
      best_candidate_counter = current_candidate_counter;
    }
  }
    
  Serial.print("FREQ: desired: ");
  Serial.print(desired_frequency);
  Serial.print(" best found: ");
  Serial.print(best_candidate_frequency);
  Serial.print(" prescaler: ");
  Serial.print(pre_scaled_frequencies[best_candidate_pre_scaler]);
  Serial.print(" counter: ");
  Serial.println(best_candidate_counter);

  /*
   * Now let's load the registers with the found values.
   */
  if (best_candidate_frequency > 0) {
    switch (best_candidate_pre_scaler) {
      case 0:
        TCCR3B = TCCR3B | (1 << CS30); //# no prescaling
        break;
      case 1:
        TCCR3B = TCCR3B | (1 << CS31); //# /8 prescaling
        break;
      case 2:
        TCCR3B = TCCR3B | (1 << CS31) | (1 << CS30); //# /64 prescaling
        break;
      case 3:
        TCCR3B = TCCR3B | (1 << CS32); //# /256 prescaling
        break;
      case 4:
        TCCR3B = TCCR3B | (1 << CS32) | (1 << CS30); //# /1024 prescaling
        break;
    }

    OCR3A = best_candidate_counter;
  }
   
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei(); 

}
