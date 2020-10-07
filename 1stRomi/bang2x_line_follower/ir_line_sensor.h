#include "pin_names_and_constants.h"

/**
 * This class will be responsible for reading one of the IR sensors,
 * calibrating it and providing usable values of the reading. The value
 * read will be an analogue value coming from a 10-bit ADC.
 */
class LineSensor {
  public:

    /**
     * Constructor will take in one pin, which is an ADC pin on the Atmel microcontroller.
     */
    LineSensor(unsigned short pin) {
//      Serial.print("Setting up line sensor pin: ");
//      Serial.println(pin);
      if (initialisedSensors >= LINE_SENSOR_COUNT) {
        Serial.println("Attempting to initialise too many line sensors. Please check your code.");
      } else {
        adc_pin = pin;
        pinMode(adc_pin, INPUT);
        initialiseTimer3(1);
        allLineSensors[initialisedSensors] = *this;
        initialisedSensors++;
      }
    }

    /**
     * Take in a fresh reading for each initialised sensor.
     */
    static void updateAllInitialisedSensors() {
      for(int i = 0; i < initialisedSensors; i++) {
        allLineSensors[i].readCurrentValue();
      }
    }

  private:
    /**
     * The ADC pin that this sensor will use.
     */
    byte adc_pin;

    /**
     * A flag of whether the sensor has been initialised.
     */
    static bool timer_initialised;

    /**
     * A reference collection of all the sensors, so that we know what to update
     * in timer3.
     */
    static LineSensor* allLineSensors;

    /**
     * How many sensors have been initialised.
     */
    static byte initialisedSensors;

    /**
     * Current reading of the sensor.
     */
    unsigned int currentReading;

    /**
     * A function to initialise timer3 to start the measurements.
     */
    void initialiseTimer3(long desired_frequency);

    /**
     * A function to read the current ADC sensor value.
     */
    void readCurrentValue() {
      currentReading = analogRead(adc_pin);
      Serial.print( currentReading );
      Serial.print( ", " );
    }
};

/*
 * In the beginning timer is NOT initialised and no sensors have been initialised.
 */
bool LineSensor::timer_initialised = false;
byte LineSensor::initialisedSensors = 0;
LineSensor* LineSensor::allLineSensors = malloc(LINE_SENSOR_COUNT * sizeof(LineSensor));

/**
 * We'll use timer3 to read the sensors.
 * 
 * The name TIMER3_COMPA_vect is a special flag to the 
 * compiler.  It automatically associates with Timer3 in
 * CTC mode.
 */
ISR( TIMER3_COMPA_vect ) {
  /*
   * Let's update all initialised sensors.
   */
  LineSensor::updateAllInitialisedSensors();
  Serial.print( "\n" );
}

/**
 * Routine to setupt timer3 to run 
 * 
 * @desired_frequency - the desired frequency for the timer to trigger the ISR at.
 */
void LineSensor::initialiseTimer3(long desired_frequency) {
  /**
   * We only want to initialise this once.
   */
  if (!LineSensor::timer_initialised) {
    //Serial.print("Setting timer for sensor readings");
    LineSensor::timer_initialised = true;

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
  
    /**
     * So we have 8 options for the pre-scaler and we have a 16bit counter, so up to 65536.
     * 
     * 000: timer stopped
     * 001: no pre-scaling, so running at 16MHz
     * 010: F / 8 pre-scaler, so running at 2MHz
     * 011: F / 64. so running at 250KHz
     * 100: F / 256, so running at 62.5KHz
     * 101: F/ 1024, so running at 16KHz
     * 110: External clock, falling edge
     * 111: External clock, rising edge.
     * 
     * Let's divide the pre-scaled frequency with the desired frequency and we'll get the counter value
     * that we need to count up to, then go through all the pre-scaler frequencies and see which of the 
     * divisions is the cleanest (the least value after the decimal point).
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
  
      //# Minimising the deviation from the desired frequency accross different pre-scale values
      if (abs(current_candidate_frequency - desired_frequency) < abs(best_candidate_frequency - desired_frequency) && current_candidate_counter <= 65536) {
        best_candidate_frequency = current_candidate_frequency;
        best_candidate_pre_scaler = cnt;
        best_candidate_counter = current_candidate_counter;
      }
    }
      
    Serial.print("Sensor update frequency: desired: ");
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
}