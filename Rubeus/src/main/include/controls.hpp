#pragma once

#include <frc/XboxController.h>

enum Buttons {
    ELBOW_CONTROL,
    ARM_BARF,
    ARM_INTAKE_POS,
    SQUARE_UP,
    ARM_PICKUP_POS,
    ARM_INTAKE,
    ZERO_NAVX,
    SHOULDER_CONTROL, 
    TOGGLE_OPTION_1,
    TOGGLE_OPTION_3,
    STOP_ARM,
    KEY,
    ZERO,
    ZOOM_ZOOM,
    ARM_SHOOT,
    PICKUP_MACRO,
    HIGH_POLE,
    MOOZ_MOOZ
};


enum Axis {
    ARM_TRIM = 1,
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

    bool optionPressedStates[3];
    bool optionReleasedStates[3];

    int secret = 0;
    int oldPOV = 0;
public:
    bool isSecret = false;

    Controls() {
        for (int x = 0; x < 13; x ++) {
            buttonPressedStates[x] =  false;
            buttonReleasedStates[x] = false;
            buttonToggledStates[x] =  false;
        }
        for (int x = 0; x < 3; x ++) {
            optionPressedStates[x] = false;
            optionReleasedStates[x] = false;
        }
    }

    void update() {
        usedAxis[SPEED_LIMIT] = 0.3;
        if (xbox.IsConnected()) {            
            if (xbox.GetPOV() == -1){
                if (oldPOV == 90){ // up
                    if (secret < 2){
                        secret ++;
                    }
                    else{
                        secret = 0;
                    }
                }
                else if (oldPOV == 270){ // down
                    if ((secret >= 2) && (secret < 4)){
                        secret ++;
                    }
                    else{
                        secret = 0;
                    }
                }
                else if (oldPOV == 0){ // right
                    if ((secret == 4) || (secret == 6)){
                        secret ++;
                    }
                    else{
                        secret = 0;
                    }
                }
                else if (oldPOV == 180){ // left
                    if ((secret == 5) || (secret == 7)){
                        secret ++;
                    }
                    else{
                        secret = 0;
                    }
                }
            }
            usedCoords[LEFT_X] = xbox.GetRawAxis(0);
            usedCoords[LEFT_Y] = xbox.GetRawAxis(1);
            usedCoords[RIGHT_X] = xbox.GetRawAxis(4);
            usedCoords[RIGHT_Y] = xbox.GetRawAxis(5);
            usedButtonStates[ZERO_NAVX] = xbox.GetRawButton(3);
            //usedButtonStates[ELBOW_CONTROL] = xbox.GetRawButton(5);
            //usedButtonStates[SHOULDER_CONTROL] = xbox.GetRawButton(6);
            usedButtonStates[ARM_INTAKE] = xbox.GetRawButton(2);
            usedAxis[ARM_TRIM] = xbox.GetRawAxis(2) - xbox.GetRawAxis(3);
            usedButtonStates[SQUARE_UP] = xbox.GetRawButton(4);
            usedButtonStates[ZOOM_ZOOM] = xbox.GetRawButton(6);
            usedButtonStates[ARM_SHOOT] = xbox.GetRawButton(8);
            usedButtonStates[MOOZ_MOOZ] = xbox.GetRawButton(5);
            if (usedButtonStates[ARM_SHOOT]){
                if (secret == 8){
                    secret ++;
                }
                else{
                    secret = 0;
                }
            }
            else {
                if (secret == 9){
                    isSecret = !isSecret;
                }
            }
            oldPOV = xbox.GetPOV();
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
            usedButtonStates[ARM_PICKUP_POS] = buttonboard.GetRawButton(3);
            usedButtonStates[ZERO] = buttonboard.GetRawButton(13);
            //usedAxis[ARM_TRIM] = buttonboard.GetRawAxis(2);
            usedButtonStates[STOP_ARM] = buttonboard.GetRawButton(8);
            usedButtonStates[SQUARE_UP] |= buttonboard.GetRawButton(9);
            usedButtonStates[ARM_BARF] = buttonboard.GetRawButton(4);
            usedButtonStates[ARM_INTAKE_POS] = buttonboard.GetRawButton(5);
            usedButtonStates[PICKUP_MACRO] = buttonboard.GetRawButton(12);
            usedButtonStates[HIGH_POLE] = buttonboard.GetRawButton(11);
        }
    }

    double GetTrim(){
        return usedAxis[ARM_TRIM];
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
    bool GetOptionPressed(short num) {
        if (GetOption() != num) {
            optionPressedStates[num-1] = true;
        }
        else if (optionPressedStates[num-1]) {
            optionPressedStates[num-1] = false;
            return true;
        }
        return false;
    }
    bool GetOptionReleased(short num) {
        if (GetOption() == num) {
            optionReleasedStates[num-1] = true;
        }
        else if (optionReleasedStates[num-1]) {
            optionReleasedStates[num-1] = false;
            return true;
        }
        return false;
    }
    bool GetKey() {
        return GetButton(KEY);
    }
};