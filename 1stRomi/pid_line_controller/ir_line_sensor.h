#ifndef _LINE_SENSOR_
#define _LINE_SENSOR_

#include <Arduino.h>
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
     * Returns the current rolling average sensor value compensated for the bias.
     */
    unsigned int getCurrentSensorValue();

    /**
     * Returns the current sensor value compensated for the bias.
     */
    unsigned int LineSensor::getUnbiasedSensorValue();

    /**
     * Returns the current sensor value compensated for the if we think
     * that we're over line. Otherwise returns 0.
     */
    unsigned int getCurrentSensorValueWhenReliableSignal();

    /**
     * A public static function to initialise the timer. Turns out that if we do this inside 
     * the constructor of LineSensor, then our configuration gets overwritten later, because
     * our instance is constructed before Arduino has initialised its stuff. So we have no
     * choice but to call this function in the setup section of the main code.
     */
    static void reInitTimer(long freq);

    /**
     * Returns the current sensor value as is, with no bias compensation.
     */
    unsigned int getCurrentSensorValueRaw();

    /**
     * Returns the current sensor value as is, with no bias compensation.
     */
    unsigned long getBias();

    /**
     * Returns a flag of whether the sensor is above a line or not.
     */
    bool overLine();

    /**
     * Resets bias and outstanding calibration values so that we enter calibration routine again.
     */
    void resetCalibration();

    /**
     * Take in a fresh reading for each initialised sensor.
     */
    static void updateAllInitialisedSensors();

    /**
     * Accessors for all lines sensors.
     */
    static LineSensor* getRightSensor();

    static LineSensor* getCentreSensor();

    static LineSensor* getLeftSensor();

    /**
     * Accessors for right and left wheel speeds.
     */
    static long getRightWheelSpeed();

    static long getLeftWheelSpeed();

    /**
     * Accessors for right and left wheel speeds converted to m/s.
     */
    static double getRightWheelSpeed_ms();

    static double getLeftWheelSpeed_ms();

    /**
     * Sets a threshold of the given time in ms and the function that needs to be run
     * after the time has elapsed.
     */
    static void setThreshold(ThresholdCallback* threshold_triggered_functionality, unsigned int ms);

  private:
    /**
     * Constructor will take in one pin, which is an ADC pin on the Atmel microcontroller.
     * It doesn't have to be public, because we'll only create instances of sensor
     * within ir_line_sensor.h and those instances will be available through a static function.
     */
    LineSensor(byte pin);

    static volatile bool led_state;
    static void toggle_led();

    /**
     * A function to read the current ADC sensor value.
     */
    void readCurrentValue();

    /**
     * A function to initialise timer3 to perform the measurements.
     */
    static void initialiseTimer3(long desired_frequency);
    
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
     * This variable will store the scaler value for speed updates to ensure that we get the desired speed update
     * frequency as in WHEEL_SPEED_UPDATE_FREQUENCY.
     */
    static int speed_update_scaler;
    /**
     * In conjunction with speed_update_scaler this will allow for correct scaling.
     */
    static int speed_update_counter;

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
     * Buffer to keep the line sesnor values for rolling averaging.
     */
    volatile unsigned long rollingAvgReadings[LINE_SENSOR_AVG_ROLL_COUNT];
    volatile unsigned int rolling_avg_cnt; //# counter for rolling averaging.

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
     * A pointer collection to threshold callbacks (typically motor controls) so that we can do 
     * something e.g. with the motor when the threshold has been reached.
     */
    static ThresholdCallback* timer_thresholds[MAX_CALLBACKS_FOR_TIMER];
};
#endif
