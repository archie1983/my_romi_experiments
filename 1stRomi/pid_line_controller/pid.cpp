#include "pid.h"

/*
   Class constructor
   This runs whenever we create an instance of the class
*/
PID_c::PID_c(float P, float I, float D)
{
  //Store the gains
  setGains(P, I, D);
  //Set last_millis
  reset();
}

/*
   This function prints the individual contributions to the total contol signal
   You can call this yourself for debugging purposes, or set the debug flag to true to have it called
   whenever the update function is called.
*/
void PID_c::printComponents() {

  Serial.print(Kp_output);
  Serial.print(", ");
  Serial.print(Kd_output);
  Serial.print(", ");
  Serial.print(Ki_output);
  Serial.print(", ");
  Serial.println(output_signal);
  
}


void PID_c::reset() {
  last_error      = 0;
  integral_error  = 0;
  Kp_output       = 0;
  Ki_output       = 0;
  Kd_output       = 0;
  last_millis     = millis();
}

/**
 * This function sets the gains of the PID controller.
 * 
 * Make sure that P < 1.0 otherwise we get to oscillations very quick.
 */
void PID_c::setGains(float P, float I, float D) {
  Kp = P;
  Kd = D;
  Ki = I;
}

/**
 * Returns a flag of whether this PID controller needs to be updated or not.
 */
bool PID_c::isUpdatesWanted() {
  return updates_wanted;
}

/**
 * Sets a flag of whether this PID controller needs to be updated or not.
 */
void PID_c::setUpdatesWanted(bool yes_or_no) {
  updates_wanted = yes_or_no;
}

/*
   This is the update function.
   This function should be called repeatedly.
   It takes a measurement of a particular quantity and a desired value for that quantity as input
   It returns an output; this can be sent directly to the motors,
   or perhaps combined with other control outputs
*/
float PID_c::update(float demand, float measurement) {
  if (updates_wanted) {
    //Calculate how much time (in milliseconds) has 
    // bassed since the last update call
    // Note, we do this in type "long", and then
    // typecast the final result to "float".
    long time_now = millis();
    long diff_time = time_now - last_millis;
    last_millis = time_now;
    
    float time_delta = (float)diff_time;
  
    /**
     * Calculate error between demand and measurement.
     * Positive error means we've overshot (it delivers more than requested).
     * Negative error means we've undershot (it delivers less than requested).
     * 
     * This is the Proportional component of required adjustment.
     */
    float error = measurement - demand;
    //float error = demand - measurement;
  
    /**
     * This represents the error derivative- the Derivative component of required adjustment.
     * 
     * In other words: By how much has the error changed since last time.
     * If error_delta is positive, then our error has increased since last time and if it's negative,
     * then our error has decreased since last time.
     */
    float error_delta = 0.0;
    /**
     * If time_delta is 0, then it means we've just reset and are updating a bit early.
     * Ignore derivative term then.
     */
    if (time_delta != 0) {
      error_delta = (error - last_error) / time_delta;
    }
    
    last_error = error;
  
    /**
     * Integral term- the Integral component of required adjustment.
     * 
     * In other words: Just an accumulation of the total error over time. 
     * If we overshoot, this will get bigger, if we undershoot, this will get smaller.
     * The problem is that this is like a punishment for past errors and it will
     * get bigger very quickly if we have any error in the system, which is why the
     * coefficient must be tiny for this.
     * 
     * Not using time_delta here, because I've overlooked it, however since my PID
     * algorightm seems to work, I'm leaving this as is for now.
     */
    //integral_error += error;// * time_delta;
    integral_error += error * time_delta;
  
    //Calculate P,I,D Term contributions.
    Kp_output = Kp * error;
    Kd_output = Kd * error_delta; 
    Ki_output = Ki * integral_error; 
  
    //Add the three components to get the total output
    /**
     * P component goes with a negative sign because remember our error is positive if we've overshot, 
     * so in that case we want to reduce the output signal and if error is negative, then we've undershot
     * and want to increase output signal.
     * 
     * D component goes with a negative sign, because it needs to dampen the effect of P component. Remember,
     * if we have a positive error_delta, then our error has increased and P term will try to add more, which
     * we want to lessen with D term. It's getting difficult to understand it, but I think that's what it should
     * be.
     */
    output_signal = -Kp_output - Kd_output - Ki_output;
  
  //  Serial.print("Err: ");
  //  Serial.print(error);
  //  Serial.print(" Err_d: ");
  //  Serial.print(error_delta);
  //  Serial.print(" time_d: ");
  //  Serial.println(time_delta);
  //  Serial.print(" Kp_output: ");
  //  Serial.println(Kp_output);
  //  Serial.print(" Kd_output: ");
  //  Serial.println(Kd_output);
  //  Serial.print(" output_signal: ");
  //  Serial.println(output_signal);
  
  //  Serial.print(-Kp_output);
  //  Serial.print(", ");
  //  Serial.println(-Kd_output);
  
    // Pass the result back.
    return output_signal;
  }
}
