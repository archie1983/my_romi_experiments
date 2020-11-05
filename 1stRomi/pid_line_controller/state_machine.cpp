#include "state_machine.h"
#include "kinematics.h"
#include "pin_names_and_constants.h"

/**
 * Update function for the states with simple transition without parameters. Unconditional transition.
 * This is typically called as a result of motor completing its task or from similar events.
 * Only some of the state machine states will transition from and to using this function.
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
 * Update function for the states that require line sensor values to transition. This is typically
 * called periodically from the main loop with each line sensor result. Only some of the state machine
 * states will transition from and to using this function.
 */
void StateMachine::update(bool left_line_visible, bool centre_line_visible, bool right_line_visible) {
  switch (currentState) {
    case LOOKING_FOR_LINE_MOVING_FORWARD:
      /**
       * If we were looking for a line and now we find it, then now we need to switch to the 
       * behaviour that allows us to move along the line (with heading PID and everything).
       */
      if (left_line_visible || centre_line_visible || right_line_visible) {
        setState(MOVING_WITH_LINE);
      }
      break;
    case LOOKING_FOR_LINE_TURNING_LEFT:
      /**
       * If we were looking for a line and now we find it, then now we need to switch to the 
       * behaviour that allows us to move along the line (with heading PID and everything).
       */
      if (left_line_visible || centre_line_visible || right_line_visible) {
        setState(MOVING_WITH_LINE);
      }
      break;
    case LOOKING_FOR_LINE_TURNING_RIGHT:
      /**
       * If we were looking for a line and now we find it, then now we need to switch to the 
       * behaviour that allows us to move along the line (with heading PID and everything).
       */
      if (left_line_visible || centre_line_visible || right_line_visible) {
        setState(MOVING_WITH_LINE);
      }
      break;
    case MOVING_WITH_LINE:
      /**
       * If we were moving happily along the line and now the line is lost completely,
       * then we switch to behaviour that will be looking for line.
       */
      if (!left_line_visible && !centre_line_visible && !right_line_visible) {
        setState(LINE_LOST);
      }
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
    case LOOKING_FOR_LINE_MOVING_FORWARD:
      /*
       * Here we just want to set our motors to run at steady pace (with the heading PID disabled) to go straight.
       */
      //heading_pid->reset();
      Kinematics::getKinematics()->fullStop(); //# not sure if we need this. Maybe individual motor PIDs can deal with it.
      heading_pid->setUpdatesWanted(false);
      Kinematics::getKinematics()->walkStraightLookingForLine();
      break;
    case MOVING_WITH_LINE:
      /*
       * Here we want to give the full control to the heading PID.
       */
      Kinematics::getKinematics()->fullStop(); //# not sure if we need this. Maybe individual motor PIDs can deal with it.
      heading_pid->reset();
      heading_pid->setUpdatesWanted(true);
      //Kinematics::getKinematics()->walkStraightLookingForLine();
      break;
    case LINE_LOST:
      /*
       * Here we want to stop the motors, disable all PID controllers and start looking
       * for the line.
       */
      heading_pid->setUpdatesWanted(false);
      Kinematics::getKinematics()->fullStop();
      /*
       * Once we've stopped, we should now wait a little bit so that wheel inertia dies down
       * and then we carry on with the next state transition.
       */
      delay(100);
      update(); //# this will cause a recursive call of this setState function, but it should be able to handle it.
      break;
    case LOOKING_FOR_LINE_TURNING_LEFT:
      /**
       * Here we want to turn left by 90 degrees to hopefully find the line. Actually,
       * why not make that 100 degrees just in case?
       */
      Kinematics::getKinematics()->turnByAngle(-A_100_DEGREES_IN_RADIANS, true);
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


/**
 * Provides a reference of the heading PID to the state machine.
 */
void StateMachine::setHeadingPID(PID_c * in_heading_pid) {
  heading_pid = in_heading_pid;
}

StateMachine* StateMachine::stateMachine = new StateMachine(StateMachine::LineFollowingStates::IDLING);
