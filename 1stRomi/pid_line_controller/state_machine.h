#ifndef _STATE_MACHINE_
#define _STATE_MACHINE_

#include <Arduino.h>
#include "pid.h" //# we'll want to have control of the heading PID in state machine.

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
      LOOKING_FOR_LINE_TURNING_RIGHT, //# when we don't have the line in sight and we're turning right to find it
      LOOKING_FOR_LINE_TURNING_LEFT, //# when we don't have the line in sight and we're turning left to find it
      LOOKING_FOR_LINE_TURNING_BACK, //# when we don't have the line in sight and we've turned right and left and now are turning back to where we were when we started looking for the line.
      LOOKING_FOR_LINE_MOVING_FORWARD, //# when we don't have the line in sight and we're moving forward to find it
      TURNING_TO_GO_HOME, //# when we turn to face home
      TURNING_TO_GO_HOME_MOTOR1_DONE, //# Each motor has a callback handle and that handle will be used by each motor
                                      //# to notify us that it has completed turning. They will finish at different times,
                                      //# so after 1st motor finishes, we'll transition to this state. After the 2nd one
                                      //# finishes, we'll continue to MOVING_TO_GO_HOME.
      TURNING_RIGHT_TO_FIND_LINE_MOTOR1_DONE, //# For identical reasons as above we need this state for when we look right for the line.
      TURNING_LEFT_TO_FIND_LINE_MOTOR1_DONE, //# For identical reasons as above we need this state for when we look left for the line.
      TURNING_BACK_TO_FIND_LINE_MOTOR1_DONE, //# For identical reasons as above we need this state for when we turn back to where we were while looking for the line.
      LOOKING_FOR_LINE_BEHIND_ME, //# On our back run we'll need to look for line behind the robot.
      TURNING_FOR_LINE_BEHIND_ME_MOTOR1_DONE, //# For identical reasons as above we need this state for when we look for the line behind us.
      MOVING_TO_GO_HOME,
      HOME_REACHED
    };

    /**
     * Update function for the states with simple transition without parameters. Unconditional transition.
     */
    void update();

    /**
     * Update function for the states that require line sensor values to transition.
     */
    void update(bool left_line_visible, bool centre_line_visible, bool right_line_visible);

    /**
     * Sets the current state to the given one and executes a function
     * related to that state.
     */
    void setState(LineFollowingStates state);

    static StateMachine* getStateMachine();

    /**
     * Provides a reference of the heading PID to the state machine.
     */
    void setHeadingPID(PID_c * in_heading_pid);

  private:
    LineFollowingStates currentState = LineFollowingStates::IDLING;

    static StateMachine* stateMachine;

    /**
     * Private constructore, because we only want one sate machine - a singleton.
     */
    StateMachine(LineFollowingStates initialState);

    /**
     * A reference to the heading PID.
     */
    PID_c * heading_pid;
};

#endif
