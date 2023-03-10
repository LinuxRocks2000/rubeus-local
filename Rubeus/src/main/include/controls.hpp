#pragma once

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>

enum Buttons {
    ELBOW_CONTROL = 1,
    ARM_BARF  = 5,
    ARM_INTAKE = 3,
    INTAKE_MACRO = 4,
    ARM_PICKUP = 5,
    ZERO_NAVX = 6,
    SHOULDER_CONTROL = 7, 
    TOGGLE_OPTION_1 = 8,
    TOGGLE_OPTION_3 = 9,
    STOP_MACROS = 10,
    KEY = 11,
    ZERO = 12
};


enum Axis {
    TRIM = 1,
    SPEED_LIMIT = 2
};


enum Coords {
    LEFT_X = 1,
    LEFT_Y = 2,
    RIGHT_X = 3,
    RIGHT_Y = 4
};


template <int ButtonboardID, int XboxID, int JoystickID>
class Controls {
    frc::GenericHID buttonboard {ButtonboardID};
    frc::GenericHID xbox {XboxID};
    frc::GenericHID joy {JoystickID};

    std::map<Buttons, bool> usedButtonStates;
    
    std::map<Axis, float> usedAxis;
    std::map<Coords, float> usedCoords;

    bool buttonPressedStates[13];
    bool buttonReleasedStates[13];
    bool buttonToggledStates[13];

public:
    Controls() {
        for (int x = 0; x < 13; x ++) {
            buttonPressedStates[x] =  false;
            buttonReleasedStates[x] = false;
            buttonToggledStates[x] =  false;
        }
    }

    void update() {
        if (xbox.IsConnected()) {            
            usedCoords[LEFT_X] = xbox.GetRawAxis(0);
            usedCoords[LEFT_Y] = xbox.GetRawAxis(1);
            usedCoords[RIGHT_X] = xbox.GetRawAxis(3);
            usedCoords[RIGHT_Y] = xbox.GetRawAxis(4);
            usedButtonStates[ARM_BARF] = xbox.GetRawButton(2);
            usedButtonStates[ARM_INTAKE] = xbox.GetRawButton(1);
            usedButtonStates[ZERO_NAVX] = xbox.GetRawButton(3);
            usedButtonStates[ELBOW_CONTROL] = xbox.GetRawButton(5);
            usedButtonStates[SHOULDER_CONTROL] = xbox.GetRawButton(6);
        }
        if (joy.IsConnected()) {
            usedCoords[LEFT_X] = joy.GetRawAxis(0);
            usedCoords[LEFT_Y] = joy.GetRawAxis(1);
            usedCoords[RIGHT_X] = joy.GetRawAxis(2);
            usedCoords[RIGHT_Y] = joy.GetRawAxis(2);
            usedAxis[SPEED_LIMIT] = (joy.GetRawAxis(3) + 1)/2;
            usedButtonStates[ELBOW_CONTROL] = joy.GetRawButton(2);
            usedButtonStates[SHOULDER_CONTROL] = joy.GetRawButton(4);
            usedButtonStates[ARM_BARF] = joy.GetRawButton(3);
            usedButtonStates[ARM_INTAKE] = joy.GetRawButton(5);
            usedButtonStates[ZERO_NAVX] = joy.GetRawButton(6);
        }
        if (buttonboard.IsConnected()) {
            usedAxis[SPEED_LIMIT] = (buttonboard.GetRawAxis(0) + 1) / 2;                // This one is flipped
            usedButtonStates[KEY] = buttonboard.GetRawButton(7);
            usedButtonStates[TOGGLE_OPTION_1] = buttonboard.GetRawButton(10);
            usedButtonStates[TOGGLE_OPTION_3] = buttonboard.GetRawButton(6);
            usedButtonStates[ARM_PICKUP] = buttonboard.GetRawButton(3);
            usedButtonStates[ZERO] = buttonboard.GetRawButton(13);
        }
    }

    bool GetButton(Buttons button) {
        return usedButtonStates[button];
    }

    bool GetButtonPressed(Buttons button) {
        if (!GetButton(button)) {
            buttonPressedStates[(int)button - 1] = true;
        }
        else if (buttonPressedStates[(int)button - 1]) {
            buttonPressedStates[(int)button - 1] = false;
            return true;
        }
        return false;
    }

    bool GetButtonReleased(Buttons button) {
        if (GetButton(button)) {
            buttonReleasedStates[(int)button - 1] = true;
        }
        else if (buttonReleasedStates[(int)button - 1]) {
            buttonReleasedStates[(int)button - 1] = false;
            return true;
        }
        return false;
    }

    bool GetButtonToggled(Buttons button) {
        if (GetButtonReleased(button)) {
            buttonToggledStates[(int)button] =! buttonToggledStates[(int)button];
        }
        return buttonToggledStates[(int)button];
    }

    void ResetToggle(Buttons button) {
        buttonToggledStates[(int)button] = false;
    }

    float LeftX() {
        return usedCoords[LEFT_X];
    }
    float LeftY() {
        return usedCoords[LEFT_Y];
    }
    float RightX() {
        return usedCoords[RIGHT_X];
    }
    float RightY() {
        return usedCoords[RIGHT_Y];
    }
    float GetSpeedLimit() {
        return usedAxis[SPEED_LIMIT];
    }
    short GetOption() {
        if (GetButton(TOGGLE_OPTION_1)) {
            return 1;
        }
        if (GetButton(TOGGLE_OPTION_3)) {
            return 3;
        }
        return 2;
    }
    bool GetKey() {
        return GetButton(KEY);
    }
};