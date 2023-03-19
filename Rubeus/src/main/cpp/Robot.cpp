/* By Tyler Clarke
	This is an experiment with c++20 features and new paradigms to make FRC robot code cleaner.
	The idea: structuring an FRC robot like a real C++ program instead of like Java gone even wronger. Craszy.
*/

#define PI 3.141592

#include <iostream>
#include <fstream>
#include <FRL/bases/AwesomeRobotBase.hpp>
#include <FRL/motor/SparkMotor.hpp>
#include <FRL/motor/TalonSRXMotor.hpp>
#include <FRL/swerve/SwerveModule.hpp>
#include <constants.h>
#include <frc/XboxController.h>
#include <frc/GenericHID.h>
#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <arm.hpp>
#include <FRL/util/vector.hpp>
#include <cameraserver/CameraServer.h>
#include <FRL/util/functions.hpp>

#include "controls.hpp"
#include "Positionizer.hpp"
//#include "apriltags.h"

#include <objects.hpp>

/*const vector blue_left_ramp {12.8, -1.5};
const vector blue_mid_ramp {12.8, -1.9};
const vector blue_right_ramp {12.8, -2.3};

const vector red_left_ramp {1.3, -.55};
const vector red_mid_ramp {1.2, -1.28};
const vector red_right_ramp {1.22, -2};*/

double navxOffset = 0;
bool squared = true;

long navxHeading(){
	return smartLoop(navx.GetFusedHeading() - navxOffset, 360);
}

long navxHeadingToEncoderTicks(){
	return navxHeading() * 4096/360;
}

long totalResets = 0;

void zeroNavx(double offset = 0){
	navxOffset = smartLoop(navx.GetFusedHeading() + offset, 360);
    totalResets ++;
    std::cout << "Orientation reset #" << totalResets << std::endl;
}

bool onRamp = false;
double x = .3;

void autoRamp() {
    frc::SmartDashboard::PutNumber("Navx roll", navx.GetRoll());
    mainSwerve.SetDirection(90 * (4096/360));
    if (!onRamp) {
        mainSwerve.SetPercent(.3);
        if (navx.GetRoll() * -1 > 10) {
            onRamp = true;
        }
    }
    else {
        mainSwerve.SetDirection(90 * (4096/360));
        float speed = navx.GetRoll() * -1 * .008;
        mainSwerve.SetPercent(speed);
    }
}

bool driveTo(vector goal, bool goToZero = true, bool invertX = false, bool invertY = false){
    vector pos = odometry.Update(navxHeading());
    frc::SmartDashboard::PutNumber("Odometric X", pos.x);
    frc::SmartDashboard::PutNumber("Odometric Y", pos.y);
    double goX;
    double goY;
    if (invertX) {
        goX = pos.x - goal.x;
    }
    else {
        goX = goal.x - pos.x;
    }
    if (invertY) {
        goY = pos.y - goal.y;
    }
    else {
        goY = goal.y - pos.y;
    }
    vector translation = { goX, goY };
    vector rotation;
    PIDController<vector> rotController {&rotation};
    rotController.SetCircumference(360);
    rotController.constants.MinOutput = -.23;
    rotController.constants.MaxOutput = .23; 
    rotController.constants.P = .012;
    rotController.SetPosition(0);
    //translation.setMagnitude(translation.magnitude() * 2); // Make the falloff steeper, because this is p term only
    translation.speedLimit(0.2);
    //translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));
    rotController.Update(navxHeading());
    if (goToZero) {
        mainSwerve.SetToVector(translation, rotation);
    }
    else {
        mainSwerve.SetToVector(translation, {});
    }
    return translation.magnitude() < 0.04;
}

enum MacroMode {
    TERMINATOR,
    ARM_TYPE,
    DRIVE_TYPE,
    RAMP_TYPE,
    BARF_TYPE,
    SHIM_TYPE,
    SHOOT_TYPE,
    ORIENT_TYPE,
    PICKUP_TYPE
};

double stopBarf;

void resetBarf() {
    stopBarf = (long)frc::Timer::GetFPGATimestamp() + 2;
}

vector rot;
PIDController<vector> control {&rot};

vector squareUp(double offset = 0) {
    rot.setAngle(PI/4);
    control.SetCircumference(360);
    control.constants.MinOutput = -.23;
    control.constants.MaxOutput = .23; 
    control.constants.P = -.012;
    
    if (!squared) {
        control.SetPosition(offset);
        control.Update(navxHeading());
        if (withinDeadband(navxHeading(), 1, offset)) {
            squared = true;
        }
        return rot;
    }
    return rot;
}

struct MacroOp {
    MacroMode type;
    vector pos = {};
    double shim = 0;
};

class MacroController {
    MacroOp* mnm = 0;
    size_t sP;
    double shootStartTime = -1;
	vector rotation;
    PIDController <vector> autoRotationController { &rotation };
    double target = 0;
public:
    MacroController() {

    } 
    void operator=(MacroOp m[]){
        mnm = m;
        sP = 0;
    }

    void Update(){
        if (mnm == 0){
            return;
        }
        MacroOp thing = mnm[sP];
        switch (thing.type) {
            case TERMINATOR:
                mnm = 0;
                sP = 0;
                std::cout << "Macro unloaded" << std::endl;
                arm.AuxSetPercent(0, 0);
                break;
            case ARM_TYPE:
                //std::cout << "called" << std::endl;
                onRamp = false;
                arm.armGoToPos(thing.pos);
                if (arm.atGoal(thing.pos)) {
                    
                    std::cout << "arm reached" << std::endl;
                    sP ++;
                }
                //arm.Update();
                break;
            case DRIVE_TYPE:
                onRamp = false;
                if (driveTo(thing.pos, false)) {
                    sP ++;
                    mainSwerve.SetToVector({0, 0}, {0, 0});
                }
                break;
            case RAMP_TYPE:
                autoRamp();
                break;
            case SHIM_TYPE:
                arm.Shim(thing.shim);
                arm.checkSwitches();
                if (withinDeadband(-arm.ShimGet(), 40, thing.shim)){//arm.ShimGet() < -810) {
                    std::cout << "Shim done" << std::endl;
                    sP ++;
                }
                break;
            case SHOOT_TYPE:
                arm.SetGrab(SHOOT);
                if (shootStartTime == -1){
                    shootStartTime = (double)frc::Timer::GetFPGATimestamp();
                }
                if ((double)frc::Timer::GetFPGATimestamp() > shootStartTime + 0.25){
                    shootStartTime = -1;
                    sP ++;
                }
                break;
            case BARF_TYPE:
                arm.SetGrab(BARF);
                if (shootStartTime == -1){
                    shootStartTime = (double)frc::Timer::GetFPGATimestamp();
                }
                if ((double)frc::Timer::GetFPGATimestamp() > shootStartTime + 1){
                    shootStartTime = -1;
                    sP ++;
                }
                break;
            case ORIENT_TYPE:
                squared = false;
                squareUp(thing.shim);
                if (squared) {
                    sP ++;
                } 
                //std::cout << "Rotation: " << rotation.magnitude() << std::endl;
                break;
            case PICKUP_TYPE:
                arm.armGoToPos({70, -10});
                if (arm.atY(-10)){
                    arm.armGoToPos({100, -10});
                }
                arm.SetGrab(INTAKE);
                if (arm.atGoal({100, -10})) {
                    sP ++;
                }
                break;
        }
    }
};

frc::GenericHID buttonboard {5};
class TeleopMode : public RobotMode {
public:
	ArmPosition p { 120, 120 }; //no

	vector goal { 2, 0 };

	void Start(){
		zeroNavx();
        compressor.EnableDigital();
        arm.checkSwitches();
        mainSwerve.SetLockTime(1); // Time before the swerve drive locks, in seconds
	}

    vector g = { 45, -5 };

    void armAux(){ // Arm auxiliary mode
        if (controls.GetButton(ELBOW_CONTROL)){
            arm.AuxSetPercent(0, controls.LeftY());//g.x += controls.LeftY() * 5;
        }
        else if (controls.GetButton(SHOULDER_CONTROL)) {
            arm.AuxSetPercent(controls.LeftY(), 0);//g.y += controls.LeftY() * 5;
        }
        else{
            arm.AuxSetPercent(0, 0);
        }
        arm.test();
        arm.checkSwitches(); // Always check switches. Friggin' always.
        controls.update();
    }

	void Synchronous(){
        vector pos = odometry.Update(navxHeading());
        frc::SmartDashboard::PutNumber("Odometric X", pos.x);
        frc::SmartDashboard::PutNumber("Odometric Y", pos.y);
		frc::SmartDashboard::PutNumber("Odometry Quality", odometry.Quality());
		vector translation {controls.LeftX(), controls.LeftY()};
		vector rotation;
		rotation.setMandA(controls.RightX(), PI/4);

		float limit = controls.GetSpeedLimit();
        if (controls.GetButton(ZOOM_ZOOM)){
            limit = 0.8;
        }
		frc::SmartDashboard::PutNumber("Speed limit", limit);

        squared = !controls.GetButton(SQUARE_UP);
		rotation.dead(0.15);
		translation.dead(0.12);

		rotation.speedLimit(limit);
		translation.speedLimit(limit);

		translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));

		if (controls.GetButtonReleased(ZERO_NAVX)){
			zeroNavx();
		}

        if (!controls.GetButtonToggled(STOP_ARM)){
            if (controls.GetButton(ARM_INTAKE)){
                arm.armGoToPos({ 113, 85 });
                arm.SetGrab(INTAKE);
            }
            else if (controls.GetButton(ARM_BARF)){
                arm.SetGrab(BARF);
            }
            else if (controls.GetButton(ARM_SHOOT)){
                arm.SetGrab(SHOOT);
            }
        }
        
        if (!controls.GetButtonToggled(STOP_ARM)){
            //arm.ShimHand();
        }

        if (controls.GetKey()){
            armSol.Set(frc::DoubleSolenoid::Value::kForward);
        }
        else {
            armSol.Set(frc::DoubleSolenoid::Value::kReverse);
        }

        if (controls.GetButtonReleased(ARM_PICKUP)){
            //arm.setRetract();
        }

        if (controls.GetButtonReleased(ZERO)){
            arm.Zero();
            //arm.ShimZero();
        }
        arm.checkSwitches(); // call this as many times as you want. you wont get hurt and it makes it harder to break the arm
        arm.SetShimTrim(controls.GetTrim() * 100);
		if (controls.GetButton(ELBOW_CONTROL)){
            arm.AuxSetPercent(0, controls.LeftY());
        }
        else if (controls.GetButton(SHOULDER_CONTROL)) {
            arm.AuxSetPercent(controls.LeftY(), 0);
        }
        else{
            if (!squared) {
                mainSwerve.SetToVector(translation, squareUp());
            }
            else {
                mainSwerve.SetToVector(translation, rotation.flip());
            }
            //arm.AuxSetPercent(0, 0);
            arm.Update();
            //arm.checkSwitches();
            if (controls.GetButtonPressed(TOGGLE_OPTION_3)) {
                //arm.Zero();
            }
            if (controls.GetButton(ARM_PICKUP)) {
                //arm.ShimPickup();
                //arm.goToPickup();
                //arm.armGoToPos({60, -12});
                //arm.armPickup();
                arm.armGoToPos({60, -16});
                arm.SetGrab(INTAKE);
                arm.SetShimTrim(0);
            }
            else if (controls.GetButton(HIGH_POLE)) {
                arm.goToHighCone();
            }
            else if (controls.GetOption() == 1) {
                arm.goToHighPole();
                //arm.ShimPlaceHigh();
            }
            else if (controls.GetOption() == 2) {
                arm.goToLowPole();
                //arm.ShimPlaceLow();
            }
            else {
                //arm.ShimHome();
                arm.goToHome();
                arm.SetShimTrim(0);
                //arm.armGoToPos({35 + controls.LeftY() * -100, });
            }
        }
        arm.SetDisabled(controls.GetButtonToggled(STOP_ARM));
        arm.test();
		// Should run periodically no matter what - it cleans up after itself
        mainSwerve.ApplySpeed();
        controls.update();
	}
};

/* 27-point auto (if it works) */

MacroOp test[] {
    {
        ARM_TYPE,
        highPole
    },
    {
        SHOOT_TYPE
    },
    {
        ARM_TYPE,
        home
    },
    {
        DRIVE_TYPE,
        {5, 5}
    },
    {
        TERMINATOR
    }
};

MacroOp test1[] {
    {
        ARM_TYPE,
        highPole
    },
    {
        SHOOT_TYPE
    },
    {
        ARM_TYPE,           //place high,
        home
    },
    {
        DRIVE_TYPE,
        {0, 0}
    },
    {
        ORIENT_TYPE,
        {},
        90
    },
    {
        PICKUP_TYPE            // pickup cube,
    },
    {
        ORIENT_TYPE,
        {},
        0
    },
    {
        DRIVE_TYPE,
        {0, 0}
    },
    {
        DRIVE_TYPE,
        {0, 0}
    },
    {
        ARM_TYPE,
        highPole
    },
    {
        SHOOT_TYPE
    },
    {
        ARM_TYPE,
        home
    },
    {
        RAMP_TYPE
    },
    {
        TERMINATOR
    }
};

MacroOp autoMacro[] {
    /*{
        SHIM_TYPE,
        {},
        -800
    },
    {
        BARF_TYPE
    },
    {
        SHIM_TYPE,
        {},
        0
    },
    {
        RAMP_TYPE
    },*/
    {
        ORIENT_TYPE,
        {},
        90
    },
    {
        ORIENT_TYPE,
        {},
        0
    },
    {
        DRIVE_TYPE,
        {
            3.2,
            2.1
        }
    },
    {
        ARM_TYPE,
        { 110, 100 }
    },
    {
        BARF_TYPE
    },
    {
        ARM_TYPE,
        home
    },
    {
        DRIVE_TYPE,
        {
            3.2,
            3.5
        }
    },
    {
        ORIENT_TYPE,
        {},
        180
    },
    {
        TERMINATOR
    }
};

class AutonomousMode : public RobotMode {
	vector translation;
    MacroController macros;
public:
	AutonomousMode(){

	}

	void Start() {
        macros = test;
        zeroNavx(180);
        onRamp = false;
        //resetBarf();
        //arm.SetDisabled(false);
        mainSwerve.SetLockTime(30);
        arm.zeroed = false;
        //compressor.EnableDigital();
        //arm.checkSwitches();
        //mainSwerve.SetLockTime(1); // Time before the swerve drive locks, in seconds
	}

    //vector goal;

	void Synchronous(){
        frc::SmartDashboard::PutNumber("Navx heading", navxHeading());
        //std::cout << "not" << std::endl;
        //arm.zeroed = true;
        //mainSwerve.SetDirection(90 * (4096/360));
        macros.Update();
        //arm.armGoToPos(highPole);

        /*goal = {1, 0};
        auto pos = odometry.Update();
        translation = { pos.y - goal.y, pos.x - goal.x };
        translation.speedLimit(0.2);
        translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));
        translation = translation.flip();
        autoRotationController.SetPosition(0);//odometry.NearestAngle() * 180/PI); // Always rotate to the nearest apriltag
        autoRotationController.Update(navxHeading());
        rotation.setAngle(PI/4); // Standard rotation.
		mainSwerve.SetToVector(translation, rotation);*/
		mainSwerve.ApplySpeed();
        arm.Update();
        arm.test();
        //arm.ShimHand();
        arm.SetShimTrim(0);
	}

    void End() {
        zeroNavx(180);
    }
};


class TestMode : public RobotMode {

};


class DisabledMode : public RobotMode {

};

#ifndef RUNNING_FRC_TESTS // I'm afraid to remove this.
int main() {
    //frc::CameraServer::StartAutomaticCapture();
    compressor.Disable();
	mainSwerve.Link(&backRightSwerve); // Weird, right? This can in fact be used here.
	backRightSwerve.Link(&frontRightSwerve);
	frontRightSwerve.Link(&frontLeftSwerve);
	// As it turns out, int main actually still exists and even works here in FRC. I'm tempted to boil it down further and get rid of that stupid StartRobot function (replace it with something custom inside AwesomeRobot).
	return frc::StartRobot<AwesomeRobot<TeleopMode, AutonomousMode, TestMode, DisabledMode>>(); // Look, the standard library does these nested templates more than I do.
}
#endif