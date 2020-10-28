#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "pin_names_and_constants.h"
#include "encoder.h"

class kinematics_c {
  public:

    // What variables do you need?
    // What is the appropriate type?
    // ...

    // Function Prototypes
    kinematics_c();   // constructor 
    void update();    // update kinematics

    long getCurrentX_raw() {
      return current_x;
    }

    long getCurrentY_raw() {
      return current_y;
    }

    long getCurrentX_mm() {
      return encoderCountsToMM(current_x);
    }

    long getCurrentY_mm() {
      return encoderCountsToMM(current_y);
    }

  private:
    /**
     * Current co-ordinates.
     */
    long current_x = 0;
    long current_y = 0;

    /**
     * Previous pulse counts of the encoders.
     */
    long prev_pulses_right = 0;
    long prev_pulses_left = 0;

    /**
     * Current pulse counts of the encoders.
     */
    long cur_pulses_right = 0;
    long cur_pulses_left = 0;
    long cur_distance_travelled = 0;
}; // End of class definition.


kinematics_c::kinematics_c() {
  // ...  
} // end of constructor.

// Routine to execute the update to
// kinematics 
void kinematics_c::update() {
  
  cur_pulses_right = Encoder::getRightEncoder()->getPulseCount();
  cur_pulses_left = Encoder::getLeftEncoder()->getPulseCount();

  /**
   * Distance travelled since last update. This may change, because we are
   * not taking the turning into account.
   */
  cur_distance_travelled = ((cur_pulses_right + cur_pulses_left) / 2) - ((prev_pulses_right + prev_pulses_left) / 2);
  
  /**
   * According to kinematics theory y_new = y_old + d * sin(Theta)
   * 
   * So that's how we'll be updating our current coordinates then.
   * For now we don't yet have Theta.
   */
  current_y = current_y + cur_distance_travelled;
  current_x = current_x + cur_distance_travelled;

  prev_pulses_right = cur_pulses_right;
  prev_pulses_left = cur_pulses_left;
}

#endif
