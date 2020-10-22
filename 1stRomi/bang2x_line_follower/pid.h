#ifndef _PID_h
#define _PID_h

class PID_c {
  public:

    PID_c(float P, float I, float D);                 // Constructor, not order of P I & D arguments when calling.
    void setGains(float P, float I, float D);       // This function updates the values of the gains
    void reset();                                   // Useful to remove any intergral wind-up
    float update(float demand, float measurement);  // This update takes a demand and measurement.

    void printComponents(); //This function prints the individual components of the control signal and can be used for debugging

    float Kp_output = 0;
    float Ki_output = 0;
    float Kd_output = 0;

    /* Private functions and variables are defined here. These functions / variables cannot be accessed from outside the class.
       For example, if we try to set the value of Kp in the file "Romi.h", we will get an error (Try it out!)
    */
  private:

    //Control gains
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative

    //Values to store
    float output_signal       = 0;
    float last_error          = 0; //For calculating the derivative term
    float integral_error      = 0; //For storing the integral of the error
    unsigned long last_millis = 0;
};

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

/*
   This function sets the gains of the PID controller
*/
void PID_c::setGains(float P, float I, float D) {
  Kp = P;
  Kd = D;
  Ki = I;
}

/*
   This is the update function.
   This function should be called repeatedly.
   It takes a measurement of a particular quantity and a desired value for that quantity as input
   It returns an output; this can be sent directly to the motors,
   or perhaps combined with other control outputs
*/
float PID_c::update(float demand, float measurement) {
  
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

  /**
   * This represents the error derivative- the Derivative component of required adjustment.
   * 
   * In other words: By how much has the error changed since last time.
   */
  float error_delta = last_error - error;
  last_error = error;

  /**
   * Integral term- the Integral component of required adjustment.
   * 
   * In other words: Just an accumulation of the total error over time. 
   * If we overshoot, this will get bigger, if we undershoot, this will get smaller.
   * The problem is that this is like a punishment for past errors and it will
   * get bigger very quickly if we have any error in the system, which is why the
   * coefficient must be tiny for this.
   */
  integral_error += error;

  //Calculate P,I,D Term contributions.
  Kp_output = Kp * error;
  Kd_output = Kd * error_delta; 
  Ki_output = Ki * integral_error; 

  //Add the three components to get the total output
  output_signal = Kp_output + Kd_output + Ki_output;

  // Pass the result back.
  return output_signal;
}


#endif
