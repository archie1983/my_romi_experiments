#include "threshold_callback.h"

/**
 * Decreases counter until it reaches 0. Then we trigger and stop.
 */
void ThresholdCallback::decreaseCounter() {
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
void ThresholdCallback::increaseCounter() {
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
void ThresholdCallback::setThreshold(long count) {
  thresholdCount = count;
  thresholdOn = true;
}

/**
 * Clears the current threshold.
 */
void ThresholdCallback::clearThreshold() {
  thresholdOn = false;
  thresholdCount = 0;
}

/**
 * Returns a flag of whether this threshold is active or not
 */
bool ThresholdCallback::isThresholdActive() {
  return thresholdOn;
}

/**
 * Function to call (typically by the incEncPulseCnt or decEncPulseCnt functions which in turn should 
 * typically be called by ISR) when the set threshold has been reached.
 */
void ThresholdCallback::thresholdReached() {
  clearThreshold();
  callBackFunction();
}
