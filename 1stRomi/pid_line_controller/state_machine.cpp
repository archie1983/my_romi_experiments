#include "state_machine.h"
#include "kinematics.h"

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
    case LINE_LOST: //# line was lost, so we'll turn right first
      setState(LOOKING_FOR_LINE_TURNING_RIGHT);
      break;
    case LOOKING_FOR_LINE_TURNING_RIGHT: //# we've been turning right looking for line, but now one of the motors has achieved its target
      setState(TURNING_RIGHT_TO_FIND_LINE_MOTOR1_DONE); //# we'll wait for the other one to achieve its target.
      break;
    case TURNING_RIGHT_TO_FIND_LINE_MOTOR1_DONE: //# We've been turning right to look for line, one of the motors has achieved the target a while back, and now the other one achieves it.
      setState(LOOKING_FOR_LINE_TURNING_LEFT);   //# So we'll carry on looking left.
      break;
    case LOOKING_FOR_LINE_TURNING_LEFT: //# We've been turning left looking for line, but now one of the motors has achieved its target.
      setState(TURNING_LEFT_TO_FIND_LINE_MOTOR1_DONE); //# we'll wait for the other one to achieve its target.
      break;
    case TURNING_LEFT_TO_FIND_LINE_MOTOR1_DONE: //# We've been turning left to look for line, one of the motors has achieved the target a while back, and now the other one achieves it.
      setState(LOOKING_FOR_LINE_TURNING_BACK);  //# So now we'll want to go back to where we were before we started looking right and left.
      break;
    case LOOKING_FOR_LINE_TURNING_BACK: //# So we're turning back and one of the motors has already achieved its target.
      setState(TURNING_BACK_TO_FIND_LINE_MOTOR1_DONE); //# Let's wait for the other motor.
      break;
    case TURNING_BACK_TO_FIND_LINE_MOTOR1_DONE: //# Both motors have achieved their target to put us back to where we were before starting to look right and left.
      setState(LOOKING_FOR_LINE_MOVING_FORWARD); //# Now we'll move forwards and continue looking for line.
      break;
    case TURNING_TO_GO_HOME:  //# We're turning to get to the heading that will lead us back home and one of the motors has already achieved its target.
      setState(TURNING_TO_GO_HOME_MOTOR1_DONE); //# Let's wait for the other motor.
      break;
    case TURNING_TO_GO_HOME_MOTOR1_DONE: //# Both motors have achieved their target to put us in a heading that will lead us back home.
      setState(MOVING_TO_GO_HOME); //# Now just start driving home.
      break;
    case MOVING_TO_GO_HOME: //# We've arrived home.
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
    case LOOKING_FOR_LINE_TURNING_RIGHT:
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
    case LOOKING_FOR_LINE_TURNING_BACK:
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
       * 
       * Maybe we want to set a fixed distance here and if it doesn't find it within that distance, then go home.
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
    case LOOKING_FOR_LINE_TURNING_RIGHT:
      /**
       * Here we want to turn right by 90 degrees to hopefully find the line. Actually,
       * why not make that 100 degrees just in case?
       */
      Kinematics::getKinematics()->searchForLineByTurningRight();
      break;
    case LOOKING_FOR_LINE_TURNING_LEFT:
      /**
       * Here we want to turn left by 90 degrees to hopefully find the line. Actually,
       * why not make that 100 degrees just in case? We'll of course need to make a 200
       * degree turn to first turn back the 100 that we went to the right.
       */
      Kinematics::getKinematics()->searchForLineByTurningLeft();
      break;
    case LOOKING_FOR_LINE_TURNING_BACK:
      /**
       * Here we want to turn left by 90 degrees to hopefully find the line. Actually,
       * why not make that 100 degrees just in case?
       */
      Kinematics::getKinematics()->searchForLineByTurningBack();
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


/**
 * Provides a reference of the heading PID to the state machine.
 */
void StateMachine::setHeadingPID(PID_c * in_heading_pid) {
  heading_pid = in_heading_pid;
}

StateMachine* StateMachine::stateMachine = new StateMachine(StateMachine::LineFollowingStates::IDLING);
