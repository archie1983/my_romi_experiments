//#define LED 6 //# this is for buzzer
#define LED 13 //# this is for LED

// Global variable to remember the
// on/off state of the LED.  
volatile boolean led_state = false; //# needs to be volatile because it will be changed by the ISR and not the main loop.

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

void setup() {
  pinMode(LED, OUTPUT);

  // Start Serial monitor and print "reset"
  // so we know if the board is reseting
  // unexpectedly.
  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");
  
  setupTimer3();
}

void loop() {
  // put your main code here, to run repeatedly:

}

/**
 * Routine to setupt timer3 to run 
 * 
 * @frequency - the desired frequency for the timer to trigger the ISR at.
 */
void setupTimer3(frequency) {

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
   * So first we'll want to find the closest frequency to the ones we've calculated here.
   */
   int[] pre1;
   
  TCCR3B = TCCR3B | (1 << CS32) | (1 << CS30); //# setting pre-scaler to 1024 so that we get 16kHz clock 
  OCR3A = 625; //1; //# setting the counter to 625 to achieve 25Hz flash.

  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei(); 

}

// Routine to setupt timer3 to run 
void setupTimer3() {

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
  TCCR3B = TCCR3B | (1 << CS32) | (1 << CS30); //# setting pre-scaler to 1024 so that we get 16kHz clock 
                                               //# which we'll count to get 25Hz flash
  
  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).
  //OCR3A = 31250;
  OCR3A = 625; //1; //# setting the counter to 625 to achieve 25Hz flash.

  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei(); 

}
