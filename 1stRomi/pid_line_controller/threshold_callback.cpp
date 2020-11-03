#include "threshold_callback.h"

/**
 * Decreases counter until it reaches 0. Then we trigger and stop.
 * Returns TRUE when we've reached threshold.
 */
bool ThresholdCallback::decreaseCounter() {
  if (thresholdOn) {
//    Serial.print("THRC: ");
//    Serial.print((long)this);
//    Serial.print(" : ");
//    Serial.print(thresholdCount);
//    Serial.print(" : ");
//    Serial.println(millis());
    thresholdCount--;
    if (thresholdCount == 0) {
      thresholdReached();
      return true;
    }
  }
  return false;
}

/**
 * Increases counter until it reaches 0. Then we trigger and stop.
 * Returns TRUE when we've reached threshold.
 */
bool ThresholdCallback::increaseCounter() {
  if (thresholdOn) {
//    Serial.print("THRC#####: ");
//    Serial.print((long)this);
//    Serial.print(" : ");
//    Serial.print(thresholdCount);
//    Serial.print(" : ");
//    Serial.println(millis());
    thresholdCount++;
    if (thresholdCount == 0) {
      thresholdReached();
      return true;
    }
  }
  return false;
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
