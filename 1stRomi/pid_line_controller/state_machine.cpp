#include "state_machine.h"
#include "kinematics.h"

/**
 * Update function for the states with simple transition without parameters. Unconditional transition.
 */
void StateMachine::update() {
  switch (currentState) {
    case IDLING:
      setState(LOOKING_FOR_LINE_MOVING_FORWARD);
      break;
    case LINE_LOST:
      setState(LOOKING_FOR_LINE_TURNING_LEFT);
      break;
    case LOOKING_FOR_LINE_TURNING_LEFT:
      setState(LOOKING_FOR_LINE_TURNING_RIGHT);
      break;
    case LOOKING_FOR_LINE_TURNING_RIGHT:
      setState(LOOKING_FOR_LINE_MOVING_FORWARD);
      break;
    case TURNING_TO_GO_HOME:
      setState(TURNING_TO_GO_HOME_MOTOR1_DONE);
      break;
    case TURNING_TO_GO_HOME_MOTOR1_DONE:
      setState(MOVING_TO_GO_HOME);
      break;
    case MOVING_TO_GO_HOME:
      setState(HOME_REACHED);
      break;
  }
}

/**
 * Sets the current state to the given one and executes a function
 * related to that state.
 */
void StateMachine::setState(LineFollowingStates state) {
  switch(state) {
    case IDLING:
      Serial.println("IDLING");
      break;
    case MOVING_WITH_LINE:
      Serial.println("MOVING_WITH_LINE");
      break;
    case LINE_LOST:
      Serial.println("LINE_LOST");
      break;
    case LOOKING_FOR_LINE_TURNING_LEFT:
      Serial.println("LOOKING_FOR_LINE_TURNING_LEFT");
      break;
    case LOOKING_FOR_LINE_TURNING_RIGHT:
      Serial.println("LOOKING_FOR_LINE_TURNING_RIGHT");
      break;
    case LOOKING_FOR_LINE_MOVING_FORWARD:
      Serial.println("LOOKING_FOR_LINE_MOVING_FORWARD");
      break;
    case TURNING_TO_GO_HOME:
      Serial.println("TURNING_TO_GO_HOME");
      break;
    case TURNING_TO_GO_HOME_MOTOR1_DONE:
      Serial.println("TURNING_TO_GO_HOME_MOTOR1_DONE");
      break;
    case MOVING_TO_GO_HOME:
      Serial.println("MOVING_TO_GO_HOME");
      break;
    case HOME_REACHED:
      Serial.println("HOME_REACHED");
      break;
  }
  
  currentState = state;
  switch (state) {
    case TURNING_TO_GO_HOME:
      Kinematics::getKinematics()->turnToHomeHeading(true);
      break;
    case MOVING_TO_GO_HOME:
      Kinematics::getKinematics()->walkDistanceToHome();
      break;
  }
}

StateMachine* StateMachine::getStateMachine() {
  return stateMachine;
}

/**
 * Private constructore, because we only want one sate machine - a singleton.
 */
StateMachine::StateMachine(LineFollowingStates initialState) {
  currentState = initialState;
}

StateMachine* StateMachine::stateMachine = new StateMachine(StateMachine::LineFollowingStates::IDLING);
