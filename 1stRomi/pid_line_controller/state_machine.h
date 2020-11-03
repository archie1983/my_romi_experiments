#ifndef _STATE_MACHINE_
#define _STATE_MACHINE_

#include <Arduino.h>

/**
 * This is a class that provides a state machine and a state transfer function to move from
 * one state to another.
 */
class StateMachine {
  public:
    
    enum LineFollowingStates {
      IDLING,
      MOVING_WITH_LINE,
      LINE_LOST,
      LOOKING_FOR_LINE_TURNING_LEFT, //# when we don't have the line in sight and we're turning left to find it
      LOOKING_FOR_LINE_TURNING_RIGHT, //# when we don't have the line in sight and we're turning right to find it
      LOOKING_FOR_LINE_MOVING_FORWARD, //# when we don't have the line in sight and we're moving forward to find it
      TURNING_TO_GO_HOME, //# when we turn to face home
      TURNING_TO_GO_HOME_MOTOR1_DONE, //# Each motor has a callback handle and that handle will be used by each motor
                                      //# to notify us that it has completed turning. They will finish at different times,
                                      //# so after 1st motor finishes, we'll transition to this state. After the 2nd one
                                      //# finishes, we'll continue to MOVING_TO_GO_HOME.
      MOVING_TO_GO_HOME,
      HOME_REACHED
    };

    /**
     * Update function for the states with simple transition without parameters. Unconditional transition.
     */
    void update();

    /**
     * Sets the current state to the given one and executes a function
     * related to that state.
     */
    void setState(LineFollowingStates state);

    static StateMachine* getStateMachine();

  private:
    LineFollowingStates currentState = LineFollowingStates::IDLING;

    static StateMachine* stateMachine;

    /**
     * Private constructore, because we only want one sate machine - a singleton.
     */
    StateMachine(LineFollowingStates initialState);
};

#endif
