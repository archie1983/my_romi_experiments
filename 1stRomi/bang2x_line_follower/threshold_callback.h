#ifndef _THRESHOLD_CALLBACK_

#define _THRESHOLD_CALLBACK_
/**
 * This will be a base class for functionality that allows passing function pointer
 * to a different class so that it can call it back.
 */
class ThresholdCallback {
  public:
    virtual void callBackFunction() = 0;
};

#endif
