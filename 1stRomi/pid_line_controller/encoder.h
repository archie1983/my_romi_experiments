#ifndef _ENCODER_
#define _ENCODER_

#include <Arduino.h>

#include "pin_names_and_constants.h"
#include "threshold_callback.h"

class Encoder {
  public:
    /**
     * Static function to access the left encoder
     */
    static Encoder* getLeftEncoder();

    /**
     * Static function to access the right encoder
     */
    static Encoder* getRightEncoder();

    /**
     * Increases the encoder pulse count. I'd really like this to be a private function,
     * but we can't have that if we want to call it from the ISR.
     */
    void incEncPulseCnt();

    /**
     * Decreases the encoder pulse count. I'd really like this to be a private function,
     * but we can't have that if we want to call it from the ISR.
     */
    void decEncPulseCnt();

    /**
     * Returns our current pulse count for this encoder.
     */
    long getPulseCount();

    /**
     * Returns the speed of the wheel.
     * 
     * Gives roughly the following values for given motor powers without PID:
     * 
     * power | clicks per second
     * --------------------------
     *  25   |      77 - 90
     *  100  |     280 - 300   
     *  200  |     590 - 636
     *  255  |     800 - 875
     */
    long getWheelSpeed();

    /**
     * Sets a threshold of the given counts and the function that needs to be run
     * after the threshold has been reached.
     */
    void setThreshold(ThresholdCallback* in_threshold_triggered_functionality);
    
  private:
    /**
     * Constructor is private because we will only invoke it within encoder.h and have
     * the relevant instances returnable from static functions.
     * 
     * We pass a function pointer to set up the encoder, because left and right encoder will have different
     * setup functions.
     */
    Encoder(void (*encoder_setup) (void));
    
    /**
     * Calculates line speed and multiplies it by the passed coefficient before storing.
     * The coefficient is typically either 1 for forward motion or -1 for backward motion,
     * but other values can be used depending on usecase.
     */
    void calculate_wheel_speed(int coefficient);

    /**
     * Pulse count of the encoder that is connected to this motor.
     * volatile because it's operated on in the functions that are only called from ISR.
     */
    volatile long encoder_pulse_cnt = 0;

    /**
     * We'll need this timestamp (coming from micros()) to determine speed
     * when pulse count is changing.
     * volatile because it's operated on in the functions that are only called from ISR.
     */
    volatile unsigned long timestamp_of_last_pulse = 0;

    /**
     * Speed of the wheel at the time of last encoder pulse. This is in terms of pulses 
     * per second. If rad/s or m/s are required, then a separate function needs to be
     * created.
     * volatile because it's operated on in the functions that are only called from ISR.
     */
    volatile long wheel_speed = 0;

    /**
     * References of the left encoder and the right encoder. We'll initialise them too within encoder.h
     * and they will be available through a public static function.
     */
    static Encoder* leftEncoder;
    static Encoder* rightEncoder;

    /**
     * A pointer to the motor control so that we can do something with the motor when the threshold has been reached.
     */
    ThresholdCallback *threshold_triggered_functionality = NULL;
};

#endif
