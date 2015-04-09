#include <Camera.h>

Camera::Camera(int shutterPin, int focusPin)
{
  _shutterPin = shutterPin;
  _focusPin = focusPin;
  _exposureTime = 20L; // default 20 milliseconds = 1/50 s
  reset();
}

void Camera::focus()
{
  if (_state > CAMERA_HOLD){
    digitalWrite(_focusPin, HIGH);
    _state = CAMERA_FOCUS;
  }
}

void Camera::capture()
{
  if (_state > CAMERA_HOLD){
    if (_state < CAMERA_FOCUS){
      focus();
      delay(5);
    }
 
    digitalWrite(_shutterPin, HIGH);
    _state = CAMERA_CAPTURE;
    _timer = millis() + _exposureTime;
    _counter++;
  }
}

void Camera::release()
{
  digitalWrite(_shutterPin, LOW);
  digitalWrite(_focusPin, LOW);
  _state = CAMERA_IDLE;
}

void Camera::hold()
{
  digitalWrite(_shutterPin, LOW);
  digitalWrite(_focusPin, LOW);
  _state = CAMERA_HOLD;
}


boolean Camera::checkTimer()
{
  if (_state > CAMERA_FOCUS){
    if (millis() > _timer){
      release();
      return true;
    }
  }
  return false;
}

void Camera::reset()
{
  pinMode(_shutterPin, OUTPUT);
  digitalWrite(_shutterPin, LOW);
  pinMode(_focusPin, OUTPUT);
  digitalWrite(_focusPin, LOW);
  _state = CAMERA_IDLE;
  _timer = millis();
  _counter = 0;
}

int Camera::counter()
{
  return _counter;
}

void Camera::setExposureTime(int millis)
{
  _exposureTime = millis;
}

int Camera::state(){
	return _state;
}

void Camera::stateStr(char *buff){
	sprintf(buff, "_%0004d", _counter);
	switch (_state){
		case CAMERA_HOLD:
			buff[0] = 'H';
			break;

		case CAMERA_CAPTURE:
			buff[0] = 'C';
			break;

		case CAMERA_FOCUS:
			buff[0] = 'F';
			break;
		
		case CAMERA_IDLE:
			buff[0] = 'I';
			break;
			
	}
}