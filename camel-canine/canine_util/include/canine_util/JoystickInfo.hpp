//
// Created by camel on 22. 9. 21.
//

#ifndef QTTCPCLIENT_JOYSTICKINFO_H
#define QTTCPCLIENT_JOYSTICKINFO_H

typedef struct _AXIS_
{
    int LeftStickX;
    int LeftStickY;
    int LeftTrigger;
    int RightStickX;
    int RightStickY;
    int RightTrigger;
    int DpadX;
    int DpadY;
} AXIS, * pAXIS;

typedef struct _BUTTON_
{
    int FaceButtonA;
    int FaceButtonB;
    int FaceButtonX;
    int FaceButtonY;
    int LeftBumper;
    int RightBumper;
    int Back;
    int Start;
    int Guide;
    int LeftStick;
    int RightStick;
} BUTTON, * pBUTTON;


#endif //QTTCPCLIENT_JOYSTICKINFO_H
