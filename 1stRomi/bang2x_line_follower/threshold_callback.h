#ifndef _THRESHOLD_CALLBACK_

#define _THRESHOLD_CALLBACK_
/**
 * This will be a base class for functionality that allows passing function pointer
 * to a different class so that it can call it back.
 */
class ThresholdCallback {
  public:
    /**
     * This will need to be overridden with the functionality that we want for when the threshold is reached.
     */
    virtual void callBackFunction() = 0;

    /**
     * Decreases counter until it reaches 0. Then we trigger and stop.
     */
    void decreaseCounter() {
      if (thresholdOn) {
        thresholdCount--;
        if (thresholdCount == 0) {
          thresholdReached();
        }
      }
    }

    /**
     * Increases counter until it reaches 0. Then we trigger and stop.
     */
    void increaseCounter() {
      if (thresholdOn) {
        //Serial.println(thresholdCount);
        thresholdCount++;
        if (thresholdCount == 0) {
          thresholdReached();
        }
      }
    }

    /**
     * Sets the threshold.
     */
    void setThreshold(long count) {
      thresholdCount = count;
      thresholdOn = true;
    }

    /**
     * Clears the current threshold.
     */
    void clearThreshold() {
      thresholdOn = false;
      thresholdCount = 0;
    }

    /**
     * Returns a flag of whether this threshold is active or not
     */
    bool isThresholdActive() {
      return thresholdOn;
    }
  private:
    /**
     * A threshold count. This will be set by setThreshold function. ISR will count this down and when
     * it reaches 0. the threshold will trigger.
     * volatile because it's operated on in the functions that are only called from ISR.
     */
    volatile long thresholdCount = 0;

    /**
     * If threshold is on, then this will be set to TRUE.
     * volatile because it's operated on in the functions that are only called from ISR.
     */
    volatile bool thresholdOn = false;

    /**
     * Function to call (typically by the incEncPulseCnt or decEncPulseCnt functions which in turn should 
     * typically be called by ISR) when the set threshold has been reached.
     */
    void thresholdReached() {
      clearThreshold();
      callBackFunction();
    }
};

#endif
