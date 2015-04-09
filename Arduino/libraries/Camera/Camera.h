#ifndef Camera_h
#define Camera_h

#include "Arduino.h"

#define CAMERA_HOLD -1
#define CAMERA_IDLE 0
#define CAMERA_FOCUS 1
#define CAMERA_CAPTURE 2


class Camera
{
  public:
    Camera(int shutterPin, int focusPin);
    void focus();
    void capture();
    void release();
    boolean checkTimer();
    void reset();
    void hold();
    void setExposureTime(int millis);
    int counter();
    int state();
    void stateStr(char *buff);
  private:
    int _shutterPin;
    int _focusPin;
    unsigned long _timer;
    int _exposureTime;
    int _state;
    int _counter;
};
#endif