#ifndef _LINE_SENSOR_
#define _LINE_SENSOR_

#include "pin_names_and_constants.h"
#include "encoder.h"
#include "threshold_callback.h"

/**
 * This class will be responsible for reading one of the IR sensors,
 * calibrating it and providing usable values of the reading. The value
 * read will be an analogue value coming from a 10-bit ADC.
 */
class LineSensor {
  public:

    /**
     * Returns the current sensor value compensated for the bias.
     */
    unsigned int getCurrentSensorValue() {
      if (currentReading > bias) {
        return currentReading - bias;
      } else {
        return 0;
      }
    }

    /**
     * A public static function to initialise the timer. Turns out that if we do this inside 
     * the constructor of LineSensor, then our configuration gets overwritten later, because
     * our instance is constructed before Arduino has initialised its stuff. So we have no
     * choice but to call this function in the setup section of the main code.
     */
    static void reInitTimer(long freq) {
      initialiseTimer3(freq);
    }

    /**
     * Returns the current sensor value as is, with no bias compensation.
     */
    unsigned int getCurrentSensorValueRaw() {
      return currentReading;
    }

    /**
     * Returns the current sensor value as is, with no bias compensation.
     */
    unsigned long getBias() {
      return bias;
    }

    /**
     * Returns a flag of whether the sensor is above a line or not.
     */
    bool overLine() {
      return getCurrentSensorValue() > 300;
    }

    /**
     * Resets bias and outstanding calibration values so that we enter calibration routine again.
     */
    void resetCalibration() {
      outstanding_calibration_values = LINE_SENSOR_CALIBRATION_VALUE_COUNT; //# we'll need to calibrate it for the full amount of cailbration values
      bias = 0;
    }

    /**
     * Take in a fresh reading for each initialised sensor.
     */
    static void updateAllInitialisedSensors() {
      for(int i = 0; i < initialisedSensors; i++) {
        /**
         * Notice how we use an "->" instead of "." to reference a function in the class - that's because
         * we're accessing the class via a pointer reference.
         */
        allLineSensors[i]->readCurrentValue();
      }
      
      toggle_led();
      
      /**
       * Apart from updating all initialised sensors, we'll also want to
       * calculate speed of both wheels or at least have the variables ready.
       */
      left_enc_pulse_cnt_prev = left_enc_pulse_cnt_cur;
      right_enc_pulse_cnt_prev = right_enc_pulse_cnt_cur;

      left_enc_pulse_cnt_cur = Encoder::getLeftEncoder()->getPulseCount();
      right_enc_pulse_cnt_cur = Encoder::getRightEncoder()->getPulseCount();

      /*
       * If we have any threshold running, then let's update their counters and act on them if it's
       * the time.
       */
      for (int cnt = 0; cnt < MAX_CALLBACKS_FOR_TIMER; cnt++) {
        if (timer_thresholds[cnt] != NULL && timer_thresholds[cnt]->isThresholdActive()) {
          timer_thresholds[cnt]->decreaseCounter();
          /*
           * If the threshold has done its job and is no longer active, then let's remove it.
           */
          if (!timer_thresholds[cnt]->isThresholdActive()) {
            timer_thresholds[cnt] = NULL;
          }
        }
      }
    }

    /**
     * Accessors for all lines sensors.
     */
    static LineSensor* getRightSensor() {
      return rightSensor;
    }

    static LineSensor* getCentreSensor() {
      return centreSensor;
    }

    static LineSensor* getLeftSensor() {
      return leftSensor;
    }

    /**
     * Accessors for right and left wheel speeds.
     */
    static long getRightWheelSpeed() {
      return (right_enc_pulse_cnt_cur - right_enc_pulse_cnt_prev) * current_measurement_frequency;
    }

    static long getLeftWheelSpeed() {
      return (left_enc_pulse_cnt_cur - left_enc_pulse_cnt_prev) * current_measurement_frequency;
    }

    /**
     * Accessors for right and left wheel speeds converted to m/s.
     */
    static double getRightWheelSpeed_ms() {
      return double((right_enc_pulse_cnt_cur - right_enc_pulse_cnt_prev) * current_measurement_frequency) / PULSES_PER_METER;
    }

    static double getLeftWheelSpeed_ms() {
      return double((left_enc_pulse_cnt_cur - left_enc_pulse_cnt_prev) * current_measurement_frequency) / PULSES_PER_METER;
    }

    /**
     * Sets a threshold of the given time in ms and the function that needs to be run
     * after the time has elapsed.
     */
    static void setThreshold(ThresholdCallback* threshold_triggered_functionality, unsigned int ms) {

      /*
       * If we have a space for thresholds, then let's use that space, otherwise ignore the new threshold.
       * Unless it's the same pointer address. In that case we want to update the threshold.
       */
      int cnt = 0;
      for (cnt = 0; cnt < MAX_CALLBACKS_FOR_TIMER; cnt++) {
        if (timer_thresholds[cnt] == NULL || !timer_thresholds[cnt]->isThresholdActive() || timer_thresholds[cnt] == threshold_triggered_functionality) {
          timer_thresholds[cnt] = threshold_triggered_functionality;

          /**
           * Threshold count will be the number of timer hits that we need to achieve to get the delay of the
           * given amount of ms.
           */
          long thresholdCount = ((double)ms / 1000.0) * current_measurement_frequency;
          threshold_triggered_functionality->setThreshold(thresholdCount);
          
          break;
        }
      }
    }

  private:
    /**
     * Constructor will take in one pin, which is an ADC pin on the Atmel microcontroller.
     * It doesn't have to be public, because we'll only create instances of sensor
     * within ir_line_sensor.h and those instances will be available through a static function.
     */
    LineSensor(byte pin) {
//      Serial.print("Setting up line sensor pin: ");
//      Serial.println(pin);
      if (initialisedSensors >= LINE_SENSOR_COUNT) {
        Serial.println("Attempting to initialise too many line sensors. Please check your code.");
      } else {
        adc_pin = pin;
        pinMode(adc_pin, INPUT);

        allLineSensors[initialisedSensors] = this; //# keeping track of this sensor instance
        initialisedSensors++;
        outstanding_calibration_values = LINE_SENSOR_CALIBRATION_VALUE_COUNT; //# we'll need to calibrate it for the full amount of cailbration values
        bias = 0;
      }
    }

    static volatile bool led_state;
    static void toggle_led() {
      digitalWrite(YELLOW_LED, led_state);
      led_state = !led_state;
    }
    
    /**
     * The ADC pin that this sensor will use.
     */
    byte adc_pin;

    /**
     * This variable will store the actual frequency that the timer3 runs at to take measurements of lines sensors
     * and calculcate the speed based on encoder counts.
     */
    static double current_measurement_frequency;

    /**
     * We'll want to keep track of encoder pulses- the previous (old) value will
     * be compared to the current encoder pulse reading in determining speeds 
     * for both wheels. These will be pulses per second. Another conversion 
     * function will be required to convert it to rad/s or m/s.
     */
    static volatile long left_enc_pulse_cnt_prev;
    static volatile long right_enc_pulse_cnt_prev;
    static volatile long left_enc_pulse_cnt_cur;
    static volatile long right_enc_pulse_cnt_cur;

    /**
     * A reference collection of all the sensors, so that we know what to update
     * in timer3.
     */
    static LineSensor* allLineSensors[LINE_SENSOR_COUNT];

    /**
     * Named references of each sensor. We'll initialise them too within ir_line_sensor.h
     * and they will be available through a public static function.
     */
    static LineSensor* leftSensor;
    static LineSensor* centreSensor;
    static LineSensor* rightSensor;

    /**
     * How many sensors have been initialised.
     */
    static byte initialisedSensors;

    /**
     * Current reading of the sensor.
     */
    volatile unsigned int currentReading;

    /**
     * A number of outstanding calibration values - if this is above 0,
     * then we're still calibrating.
     */
    volatile int outstanding_calibration_values;

    /**
     * Bias calculated in calibration process.
     */
    volatile unsigned long bias;

    /**
     * A function to initialise timer3 to perform the measurements.
     */
    static void initialiseTimer3(long desired_frequency);

    /**
     * A pointer collection to threshold callbacks (typically motor controls) so that we can do 
     * something e.g. with the motor when the threshold has been reached.
     */
    static ThresholdCallback* timer_thresholds[MAX_CALLBACKS_FOR_TIMER];

    /**
     * A function to read the current ADC sensor value.
     */
    void readCurrentValue() {
      currentReading = analogRead(adc_pin);

      /**
       * Making sure that first values will be used to calibrate the sensor.
       */
      if (outstanding_calibration_values > 0) {
        bias += currentReading;
        outstanding_calibration_values--;
//        Serial.print(adc_pin);
//        Serial.print(" : outstanding_calibration_values=");
//        Serial.println(outstanding_calibration_values);
      } else if(outstanding_calibration_values == 0) { 
        /**
         * Enough calibration values have been read, we can now calculate the bias.
         */
        bias = long(bias / double(LINE_SENSOR_CALIBRATION_VALUE_COUNT));
        outstanding_calibration_values--;
//        Serial.print(adc_pin);
//        Serial.print(" : bias=");
//        Serial.print(bias);
      } else {
//        Serial.print(adc_pin);
//        Serial.print(": ");
//        Serial.print(currentReading);
//        Serial.print(",");
//        Serial.print(getCurrentSensorValue());
//        Serial.print(",");
//        Serial.println(bias);
      }
      //Serial.print( outstanding_calibration_values );
      //Serial.print( ", " );
    }
};

/*
 * In the beginning no sensors have been initialised.
 */
byte LineSensor::initialisedSensors = 0;

/**
 * Initialising variables responsible for speed calculations. Making volatile those
 * variables that are used in updateAllInitialisedSensors() as that function is
 * only called from the timer ISR.
 */
double LineSensor::current_measurement_frequency = 0.0;
volatile long LineSensor::left_enc_pulse_cnt_prev = 0;
volatile long LineSensor::right_enc_pulse_cnt_prev = 0;
volatile long LineSensor::left_enc_pulse_cnt_cur = 0;
volatile long LineSensor::right_enc_pulse_cnt_cur = 0;
volatile bool LineSensor::led_state = false;

/**
 * Getting space for the threshold pointers that we might have.
 */
ThresholdCallback * LineSensor::timer_thresholds[MAX_CALLBACKS_FOR_TIMER];

/**
 * We'll need space for the line sensor references.
 */
LineSensor* LineSensor::allLineSensors[LINE_SENSOR_COUNT];

/**
 * Instantiating our sensors.
 */
LineSensor* LineSensor::rightSensor = new LineSensor(LINE_RIGHT_PIN);
LineSensor* LineSensor::centreSensor = new LineSensor(LINE_CENTRE_PIN);
LineSensor* LineSensor::leftSensor = new LineSensor(LINE_LEFT_PIN);

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
  //Serial.print( "\n" );
}

/**
 * Routine to setupt timer3 to run 
 * 
 * @desired_frequency - the desired frequency for the timer to trigger the ISR at.
 */
void LineSensor::initialiseTimer3(long desired_frequency) {
  pinMode(YELLOW_LED, OUTPUT);

  /*
   * If we have any thresholds running, let's cancel them,
   * if not just initialise.
   */
  for (int cnt = 0; cnt < MAX_CALLBACKS_FOR_TIMER; cnt++) {
    if (timer_thresholds[cnt] != NULL) {
      timer_thresholds[cnt]->clearThreshold();
    }
    timer_thresholds[cnt] = NULL;
  }
  
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
    LineSensor::current_measurement_frequency = best_candidate_frequency;
  }
   
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();
}
#endif
