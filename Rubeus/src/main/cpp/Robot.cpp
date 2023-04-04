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
#include "robot-http/robothttp.hpp" // what error? you might need to update your build.gradle against master or something
//#include "apriltags.h"

#include <frc/DigitalOutput.h>
#include <objects.hpp>

//#define SHIM_MODE



/*const vector blue_left_ramp {12.8, -1.5};
const vector blue_mid_ramp {12.8, -1.9};
const vector blue_right_ramp {12.8, -2.3};

const vector red_left_ramp {1.3, -.55};
const vector red_mid_ramp {1.2, -1.28};
const vector red_right_ramp {1.22, -2};*/

double navxOffset = 0;
bool squared = true;

std::vector <std::string> autoCommand;

const std::string staticDir = "/home/lvuser/robot-site/";

void onRequest(Request* req){
	std::cout << req -> url << std::endl;
    if (req -> url == "/api/postRobotCommands"){
        std::cout << "Got a new set of robot commands: " << req -> body << std::endl;
        autoCommand = splitString(req -> body, ',');
        req -> response -> status = "418 I'm A Teapot";
        req -> response -> body = "... and thus, I cannot make coffee";
    }
    else{ // doesn't match an api, do static file checks
        std::string staticPath = staticDir + req -> url;
        struct stat buffer;
        if (stat((staticPath + "/index.html").c_str(), &buffer) == 0){
            staticPath += "/index.html";
        }
        if (stat(staticPath.c_str(), &buffer) == 0){ // There is a potential exploit here relating to how POSIX directories are handled. Can you see it?
            std::ifstream stream(staticPath);
            std::stringstream buffer;
            buffer << stream.rdbuf();
            req -> response -> body = buffer.str();
            if (ends_with(staticPath, ".html")){
                req -> response -> contentType = "text/html";
            }
            else if (ends_with(staticPath, ".js")){
                req -> response -> contentType = "text/javascript";
            }
            else if (ends_with(staticPath, ".css")){
                req -> response -> contentType = "text/css";
            }
        }
        else{
            req -> response -> status = "404 Not Found";
            req -> response -> contentType = "text/html";
            req -> response -> body = "Whoopsie-diddle! You clearly don't know what page to visit! Get better n00b!<img src='https://gifsec.com/wp-content/uploads/2022/09/among-us-gif-3.gif' />";
        }
    }
	req -> response -> send();
}

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
double rollOffset = 0;

void zeroRoll() {
    rollOffset = navx.GetRoll();
}

double getRoll() {
    return navx.GetRoll() - rollOffset;
}

vector rot;
PIDController<vector> control {&rot};

vector squareUp(double offset = 0) {
    rot.setAngle(PI/4);
    control.SetCircumference(360);
    control.constants.MinOutput = -.23;
    control.constants.MaxOutput = .23; 
    control.constants.P = -.01;
    
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

int initialDriveToAngle = -1;

bool driveTo(vector goal, bool goToZero = true, int zeroOffset = 0, bool invertX = true, bool invertY = false){
    if (initialDriveToAngle == -1){
        initialDriveToAngle = navxHeading();
    }
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed){ // If it's on the red side, all the directions we move in are flipped for some reason.
        invertX = !invertX;
        //invertY = !invertY;
    }
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
    vector go = { goX, goY };
    vector translation = go;
    translation.setMagnitude(translation.magnitude() * 3.5); // Make the falloff steeper, because this is p term only
    translation.speedLimit(0.25);
    translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));
    if (goY > 0){
        //translation = translation.flip();
    }
    squared = false;
    mainSwerve.SetToVector(translation, squareUp(initialDriveToAngle));
    if (translation.magnitude() < 0.02) {
        initialDriveToAngle = -1;
        return true;
    }
    return false;
}

short state = 1;
long current;

int approachAngleOverRamp = -1;

bool goOverRamp() {
    if (approachAngleOverRamp == -1) {
        approachAngleOverRamp = navxHeading();
    }
    frc::SmartDashboard::PutNumber("Ramp state", state);
    frc::SmartDashboard::PutNumber("Navx roll", getRoll());
    vector translation;//mainSwerve.SetDirection(90 * (4096/360));
    if (state == 1) {
        if (getRoll() < -5) {
            state = 2;
        }
        translation.setMagnitude(0.35);//mainSwerve.SetPercent(.35);
    }
    else if (state == 2) {
        if (getRoll() > 5) {
            state = 3;
        }
        translation.setMagnitude(0.35);//mainSwerve.SetPercent(.35);
    }
    else if (state == 3) {
        if (withinDeadband(getRoll(), 1, 0)) {
            state = 4;
            current = (double)frc::Timer::GetFPGATimestamp();
        }
        translation.setMagnitude(0.35);//mainSwerve.SetPercent(.35);
    }
    else {
        if (!(current + 1 <= (double)frc::Timer::GetFPGATimestamp())) {
            translation.setMagnitude(0.2);//mainSwerve.SetPercent(.2);
        }
        else {
            return true;
        }
    }
    translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));
    mainSwerve.SetToVector(translation, squareUp(approachAngleOverRamp));
    return false;
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
    PICKUP_TYPE,
    GO_OVER_RAMP_TYPE,
    FORWARD_TYPE,
    FLIP_ORIENTATION_TYPE,
    SQUARE_UP_TYPE
};

double stopBarf;

void resetBarf() {
    stopBarf = (long)frc::Timer::GetFPGATimestamp() + 2;
}


/*void autoRamp() {
    frc::SmartDashboard::PutNumber("Navx roll", getRoll());
    if (!onRamp) {
        mainSwerve.SetDirection(90 * (4096/360));
        mainSwerve.SetPercent(.4);
        if (getRoll() * -1 > 8) {
            onRamp = true;
        }
    }
    else {
        //vector tran;
        //tran.setMandA(getRoll() * -1 * .008, 90 * (4096/360));
        //tran.setAngle(smartLoop(PI - tran.angle() + (navxHeading() * PI/180), PI * 2));

        if (withinDeadband(getRoll(), 3, 0)) {
            mainSwerve.SetPercent(0);
            mainSwerve.SetLockTime(0);
        }
        else {
            mainSwerve.SetDirection(90 * (4096/360));
            mainSwerve.SetPercent(getRoll() * -1 * .008);
            //mainSwerve.SetToVector(tran, {}squareUp());
        }
    }
}*/

int autoRampStartingOrientation = -1;

bool autoRamp() {
    if (autoRampStartingOrientation == -1){
        autoRampStartingOrientation = navxHeading();
    }
    frc::SmartDashboard::PutNumber("Navx roll", getRoll());
    vector tran;
    if (!onRamp) {
        //mainSwerve.SetDirection(90 * (4096/360));
        tran.SetPercent(.4);
        if (std::abs(getRoll()) > 11) {
            onRamp = true;
        }
    }
    else {
        if (withinDeadband(getRoll(), 3, 0)) {
            mainSwerve.SetPercent(0);
            mainSwerve.Lock();
            autoRampStartingOrientation = -1;
            return true;
        }
        else {
            tran.setMagnitude(std::abs(getRoll()) * .008);
        }
    }
    tran.setAngle(3 * PI/2);
    tran.setAngle(smartLoop(PI - tran.angle() + (navxHeading() * PI/180), PI * 2));
    squared = false;
    mainSwerve.SetToVector(tran, squareUp(autoRampStartingOrientation));
    return false;
}

short rampState = 1;
/*
bool autoRampFaster() {
    vector apply;
    if (rampState == 1) {
        apply.SetPercent(.4);
        if (std::abs(getRoll()) > 10) {
            rampState = 2;
        }
    }
    else if (rampState == 2) {
        apply.SetPercent(.2);
        if (withinDeadband(getRoll(), 8)) {
            rampState = 3;
            std::cout << "level" << std::endl;
            mainSwerve.Lock();
            mainSwerve.SetPercent(0);
            return true;
        }
    }

    apply.setAngle(3 * PI/2);
    apply.setAngle(smartLoop(PI - apply.angle() + (navxHeading() * PI/180), PI * 2));
    squared = false;
    mainSwerve.SetToVector(apply, squareUp(180));
    return false;
}*/

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
    double timeStarted = -1;
public:
    MacroController() {
        
    } 
    void operator=(MacroOp m[]){
        mnm = m;
        sP = 0;
        timeStarted = -1;
        target = 0;
        shootStartTime = -1;
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
                if (driveTo(thing.pos)) {
                    sP ++;
                    std::cout << "drived there" << std::endl;
                    mainSwerve.SetToVector({0, 0}, {0, 0});
                }
                break;
            case RAMP_TYPE:
                if (autoRamp()){
                    sP ++;
                }
                break;
            case SHIM_TYPE:
                arm.Shim(thing.shim);
                arm.checkSwitches();
                if (withinDeadband(-arm.ShimGet(), 70, thing.shim)){//arm.ShimGet() < -810) {
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
                mainSwerve.SetToVector({}, squareUp(thing.shim));
                if (squared) {
                    std::cout << "oriented" << std::endl;
                    sP ++;
                } 
                //std::cout << "Rotation: " << rotation.magnitude() << std::endl;
                break;
            case PICKUP_TYPE:
                /*arm.armGoToPos({65, -15});
                arm.SetGrab(INTAKE);
                if (arm.atY(-15)){
                    arm.armGoToPos({100, -15});
                }
                if (arm.atGoal({100, -15})) {
                    sP ++;
                }*/
                //;
                if (arm.gigaPickup()){
                    sP ++;
                }
                break;
            case GO_OVER_RAMP_TYPE:
                if (goOverRamp()) {
                    sP ++;
                    //state = 1;
                    onRamp = false;
                    std::cout << "RAMPED" << std::endl;
                }
                break;
            case FLIP_ORIENTATION_TYPE:
                navxOffset = smartLoop(navxOffset + 180, 360);
                sP ++;
                break;
            case SQUARE_UP_TYPE:
                squared = false;
                squareUp();
                if (squared){
                    sP ++;
                }
                break;
            case FORWARD_TYPE:
                //mainSwerve.SetDirection(90 * (4096/360));
                //mainSwerve.SetPercent(thing.pos.x * -1);          // don't ask
                vector tran = thing.pos;
                tran.setAngle(smartLoop(PI - tran.angle() + (navxHeading() * PI/180), PI * 2));
                mainSwerve.SetToVector(tran, {});
                if (timeStarted == -1) {
                    timeStarted = (double)frc::Timer::GetFPGATimestamp();
                }
                if ((double)frc::Timer::GetFPGATimestamp() > timeStarted + thing.shim) {
                    sP ++;
                    timeStarted = -1;
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
		zeroNavx(180);
        compressor.EnableDigital();
        arm.checkSwitches();
        mainSwerve.SetLockTime(1); // Time before the swerve drive locks, in seconds
        circuitplayground.DisablePWM();
        //circuitplayground.Pulse((units::time::second_t)1);
        //circuitplayground.Set(true);
	}

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

    frc::DigitalOutput circuitplayground { 6 };

    void DoShim() {
        vector pos = odometry.Update(navxHeading());
        frc::SmartDashboard::PutNumber("Odometric X", pos.x);
        frc::SmartDashboard::PutNumber("Odometric Y", pos.y);
		frc::SmartDashboard::PutNumber("Odometry Quality", odometry.Quality());
		vector translation {controls.LeftX(), controls.LeftY()};
		vector rotation;
		rotation.setMandA(controls.RightX(), PI/4);

		float limit = controls.GetSpeedLimit();
        if (controls.GetButton(ZOOM_ZOOM)){
            limit = 1;
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
        
        if (controls.GetButton(ARM_BARF)){
            arm.SetGrab(BARF);
            //circuitplayground.UpdateDutyCycle(0);
        }
        else if (controls.GetButton(ARM_SHOOT)){
            arm.SetGrab(SHOOT);
            //circuitplayground.UpdateDutyCycle(0.5);
        }
        
        if (!controls.GetButtonToggled(STOP_ARM)){
            //arm.ShimHand();
        }

        if (controls.GetKey()){
            armSol.Set(frc::DoubleSolenoid::Value::kForward);
            circuitplayground.Set(false);
        }
        else {
            armSol.Set(frc::DoubleSolenoid::Value::kReverse);
            circuitplayground.Set(true);
        }

        if (controls.GetButtonReleased(ARM_PICKUP_POS)){
            //arm.setRetract();
        }

        if (controls.GetButton(ARM_INTAKE)) {
            arm.SetGrab(INTAKE);
        }

        if (controls.GetButtonReleased(ZERO)){
            //arm.Zero();
            arm.ShimZero();
        }
        //arm.checkSwitches(); // call this as many times as you want. you wont get hurt and it makes it harder to break the arm
        arm.SetShimTrim(controls.GetTrim() * 200);
		if (controls.GetButton(ELBOW_CONTROL)){
            arm.AuxSetPercent(0, controls.LeftY());
        }
        else if (controls.GetButton(SHOULDER_CONTROL)) {
            arm.AuxSetPercent(controls.LeftY(), 0);
        }
        else{
            if (!squared) {
                std::cout << "called" << std::endl;
                if (smartLoop(navxHeading()  - 180) > 270 || smartLoop(navxHeading() - 180) < 90) {    // square up to the nearest "180"
                    mainSwerve.SetToVector(translation, squareUp(180));
                }
                else {
                    mainSwerve.SetToVector(translation, squareUp());
                }
            }
            else {
                mainSwerve.SetToVector(translation, rotation.flip());
            }
            //arm.AuxSetPercent(0, 0);
            arm.checkSwitches();
            if (controls.GetButtonPressed(TOGGLE_OPTION_3)) {
                //arm.Zero();
            }
            if (controls.GetButton(ARM_PICKUP_POS)) {
                arm.ShimPickup();
                //arm.armPickup();
                //arm.SetShimTrim(0);
            }
            else if (controls.GetButton(ARM_INTAKE_POS)){
                //arm.armGoToPos({ 113, 85 });
                arm.ShimPickup();
                //arm.SetGrab(INTAKE);
            }
            else if (controls.GetButton(HIGH_POLE)) {
                //arm.goToHighCone();
            }
            else if (controls.GetOption() == 1) {
                //arm.goToHighPole();
                arm.ShimPlaceHigh();
            }
            else if (controls.GetOption() == 2) {
                //arm.goToLowPole();
                arm.ShimPlaceLow();
            }
            else {
                arm.ShimHome();
                arm.SetShimTrim(0);
                //arm.goToHome();
                //arm.armGoToPos({35 + controls.LeftY() * -100, });
            }
            /*if (controls.GetOptionPressed(3)) {
                arm.Zero();
            }
            if (controls.GetOptionReleased(3)) {

            }*/
        }
        arm.SetDisabled(controls.GetButtonToggled(STOP_ARM));
        //arm.Update();
        arm.ShimHand();
        arm.test();
		// Should run periodically no matter what - it cleans up after itself
        mainSwerve.ApplySpeed();
        controls.update();
    }

    void SynchronousTesting(){
		vector translation {controls.LeftX(), controls.LeftY()};
		vector rotation;
		rotation.setMandA(controls.RightX(), PI/4);

		float limit = controls.GetSpeedLimit();
        if (controls.GetButton(ZOOM_ZOOM)){
            limit = 1;
        }
		frc::SmartDashboard::PutNumber("Speed limit", limit);

		rotation.dead(0.15);
		translation.dead(0.12);

		rotation.speedLimit(limit);
		translation.speedLimit(limit);

        translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));

        mainSwerve.SetToVector(translation, rotation.flip());

        mainSwerve.ApplySpeed();
        controls.update();
    }

	void SynchronousFull(){
        vector pos = odometry.Update(navxHeading());
        frc::SmartDashboard::PutNumber("Odometric X", pos.x);
        frc::SmartDashboard::PutNumber("Odometric Y", pos.y);
		frc::SmartDashboard::PutNumber("Odometry Quality", odometry.Quality());
		vector translation {controls.LeftX(), controls.LeftY()};
		vector rotation;
		rotation.setMandA(controls.RightX(), PI/4);

		float limit = controls.GetSpeedLimit();
        if (controls.GetButton(ZOOM_ZOOM)){
            limit = 1;
        }
        if (controls.GetButton(MOOZ_MOOZ)) {       // slow-down button, as requested
            limit = .3;
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
        
        if (controls.GetButton(ARM_BARF)){
            arm.SetGrab(BARF);
        }
        else if (controls.GetButton(ARM_SHOOT)){
            arm.SetGrab(SHOOT);
        }
        
        if (controls.GetKey()){
            armSol.Set(frc::DoubleSolenoid::Value::kForward);
            circuitplayground.Set(false);
        }
        else {
            armSol.Set(frc::DoubleSolenoid::Value::kReverse);
            circuitplayground.Set(true);
        }

        if (controls.GetButtonReleased(ARM_PICKUP_POS)){
            //arm.setRetract();
        }

        if (controls.GetButton(ARM_INTAKE)) {
            arm.SetGrab(INTAKE);
        }

        if (controls.GetButtonReleased(ZERO)){
            #ifdef SHIM_MODE
            arm.ShimZero();
            #else
            arm.Zero();
            #endif
        }
        //arm.checkSwitches(); // call this as many times as you want. you wont get hurt and it makes it harder to break the arm
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
            arm.checkSwitches();
            if (controls.GetButtonPressed(TOGGLE_OPTION_3)) {
                //arm.Zero();
            }
            if (controls.GetButton(ARM_PICKUP_POS)) {
                #ifdef SHIM_MODE
                arm.ShimPickup();
                #else
                arm.gigaPickup();
                #endif
                //arm.SetShimTrim(0);
            }
            else if (controls.GetButton(ARM_INTAKE_POS)){
                #ifdef SHIM_MODE
                arm.ShimPickup();
                #else
                arm.armGoToPos({ 113, 95 });
                #endif
                //arm.SetGrab(INTAKE);
            }
            else if (controls.GetButton(HIGH_POLE)) {
                #ifndef SHIM_MODE
                arm.goToHighCone();
                #endif
            }
            else if (controls.GetOption() == 1) {
                #ifdef SHIM_MODE
                arm.ShimPlaceHigh();
                #else
                arm.goToHighPole();
                #endif
            }
            else if (controls.GetOption() == 2) {
                #ifdef SHIM_MODE
                arm.ShimPlaceLow();
                #else
                arm.goToLowPole();
                #endif
            }
            else {
                #ifdef SHIM_MODE
                arm.ShimHome();
                arm.SetShimTrim(0);
                #else
                arm.goToHome();
                #endif
                //arm.armGoToPos({35 + controls.LeftY() * -100, });
            }
        }
        arm.SetDisabled(controls.GetButtonToggled(STOP_ARM));
        #ifdef SHIM_MODE
        arm.ShimHand();
        #else
        arm.Update();
        #endif
        arm.test();
		// Should run periodically no matter what - it cleans up after itself
        mainSwerve.ApplySpeed();
        controls.update();
	}

    void Synchronous(){
        SynchronousFull();
    }
};

/* 27-point auto (if it works) */

MacroOp test[] {
    /*{
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
        {5, 6}
    },*/
    {
        PICKUP_TYPE
    },
    {
        ARM_TYPE,
        home
    },
    {
        TERMINATOR
    }
};

MacroOp shoot_taxi[] {
    {
        DRIVE_TYPE,
        {4.7, 2.3}
    },
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
        {5.5, 4},
    },
    {
        TERMINATOR
    }
};

MacroOp shimMacro[] {
    {
        SHIM_TYPE,
        {},
        -830
    },
    {
        SHOOT_TYPE
    },
    {
        SHIM_TYPE,
        {},
        0
    },
    {
        RAMP_TYPE
    },
    {
        TERMINATOR
    }
};

MacroOp over_ramp[] {
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
        GO_OVER_RAMP_TYPE
    },
    {
        ORIENT_TYPE,
        {},
        0
    },
    {
        DRIVE_TYPE,
        {0, 0},
        1
    },
    {
        RAMP_TYPE
    },
    {
        TERMINATOR
    }
};

MacroOp ihatethis[] {
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
        FORWARD_TYPE,
        {-.3, 0},
        3.21579215315
    },
    {
        TERMINATOR
    }
};

MacroOp twenty_one[] {
    {
        DRIVE_TYPE,
        {4.7, 2.3},
    },
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
        {5.5, 4},
    },
    {
        DRIVE_TYPE,
        {4.7, 2.3},
    },
    {
        DRIVE_TYPE,
        {3.7, 2.4}
    },
    {
        ORIENT_TYPE,
        {},
        180
    },
    {
        RAMP_TYPE
    },
    {
        TERMINATOR
    }
};

MacroOp twenty_seven[] {
    {
        DRIVE_TYPE,
        {4.7, 2.3},
    },
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
        {5.1, 6.2},
    },
    {
        ORIENT_TYPE,
        {},
        0
    },
    {
        PICKUP_TYPE            // pickup cube,
    },
    {
        ARM_TYPE,
        home
    },
    {
        ORIENT_TYPE,
        {},
        180
    },
    {
        DRIVE_TYPE,
        {4.8, 2.37},
    },
    {
        DRIVE_TYPE,
        {3, 2.18},
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

MacroOp climb[] = {
    {
        RAMP_TYPE
    },
    {
        TERMINATOR
    }
};

MacroOp oldBoi[] = {
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
        RAMP_TYPE
    },
    {
        TERMINATOR
    }
};

class AutonomousMode : public RobotMode {
	vector translation;
    MacroController macros;
    std::vector <MacroOp> dynamicMacro;
public:
	AutonomousMode(){
        arm.goToHome();
	}

	void Start() {
        dynamicMacro.clear();
        for (std::string s : autoCommand){
            if (s == "place-high-cube"){
                dynamicMacro.push_back({
                    ARM_TYPE,
                    highPole
                });
                dynamicMacro.push_back({
                    SHOOT_TYPE
                });
                dynamicMacro.push_back({
                    ARM_TYPE,
                    home
                });
            }
            else if (s == "auto-ramp"){
                dynamicMacro.push_back({
                    RAMP_TYPE
                });
            }
            else if (s == "taxi-out-short"){
                dynamicMacro.push_back({
                    FORWARD_TYPE,
                    {
                        0, -.3
                    },
                    2
                });
            }
            else if (s == "taxi-out-long"){
                dynamicMacro.push_back({
                    FORWARD_TYPE,
                    {
                        0, -.3
                    },
                    3
                });
            }
            else if (s == "go-to-short-cube"){
                dynamicMacro.push_back({
                    DRIVE_TYPE,
                    {
                        0, 6
                    }
                });
            }
            else if (s == "go-to-middle-cube"){
                dynamicMacro.push_back({
                    DRIVE_TYPE,
                    {
                        -1.7, 6
                    }
                });
            }
            else if (s == "go-to-long-cube"){
                dynamicMacro.push_back({
                    DRIVE_TYPE,
                    {
                        -3.6, 6
                    }
                });
            }
            else if (s == "place-mid-cube"){
                dynamicMacro.push_back({
                    ARM_TYPE,
                    lowPole
                });
                dynamicMacro.push_back({
                    SHOOT_TYPE
                });
                dynamicMacro.push_back({
                    ARM_TYPE,
                    home
                });
            }
            else if (s == "barf"){
                dynamicMacro.push_back({
                    BARF_TYPE
                });
            }
            else if (s == "go-over-ramp"){
                dynamicMacro.push_back({
                    GO_OVER_RAMP_TYPE
                });
            }
            else if (s == "flip-one-eighty"){
                dynamicMacro.push_back({
                    FLIP_ORIENTATION_TYPE
                });
                /*dynamicMacro.push_back({
                    ORIENT_TYPE,
                    {},
                    0
                });*/
            }
            else if (s == "square-up"){
                dynamicMacro.push_back({
                    SQUARE_UP_TYPE
                });
            }
            else if (s == "go-to-second-cube-pickup"){
                dynamicMacro.push_back({
                    DRIVE_TYPE,
                    { -3.6, 2.5 } 
                });
                dynamicMacro.push_back({
                    ORIENT_TYPE,
                    {},
                    350
                });
            }
            else if (s == "go-to-middle-pickup"){
                dynamicMacro.push_back({
                    DRIVE_TYPE,
                    { -2.3, 2.5 } 
                });
                dynamicMacro.push_back({
                    ORIENT_TYPE,
                    {},
                    0
                });
            }
            else if (s == "pickup"){
                dynamicMacro.push_back({
                    PICKUP_TYPE
                });
            }
        }
        dynamicMacro.push_back({
            TERMINATOR
        });
        //macros = ihatethis;
        macros = &dynamicMacro[0]; // Because vectors are C arrays under the hood, this is perfectly valid
        zeroNavx(180);
        zeroRoll();
        onRamp = false;
        //resetBarf();
        //arm.SetDisabled(false);
        mainSwerve.SetLockTime(1);
        arm.zeroed = false;
        state = 1;
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
        translation = { pos.y - goal.y.kl, pos.x - goal.x };
        translation.speedLimit(0.2);
        translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));
        translation = translation.flip();
        autoRotationController.SetPosition(0);//odometry.NearestAngle() * 180/PI); // Always rotate to the nearest apriltag
        autoRotationController.Update(navxHeading());
        rotation.setAngle(PI/4); // Standard rotation.
		mainSwerve.SetToVector(translation, rotation);*/
		mainSwerve.ApplySpeed();
        #ifndef SHIM_MODE
        arm.Update();
        #else
        arm.ShimHand();
        #endif
        arm.test();
        
        arm.SetShimTrim(0);
	}

    void End() {
        zeroNavx(180);
    }
};


class TestMode : public RobotMode {
    void Synchronous(){
        arm.AuxSetPercent(controls.LeftY(), controls.RightY());
        controls.update();
    }
};


class DisabledMode : public RobotMode {

};


void servThread(){
    Server httpserver (5805, onRequest);
    while (true){
        httpserver.Iterate();
        usleep(1000); // Millisecond pause - this will make the server slower, but will also surrender a significant amount of time to the kernel for processing other more important threads.
    }
}

#ifndef RUNNING_FRC_TESTS // I'm afraid to remove this.
int main() {
    //frc::CameraServer::StartAutomaticCapture();
    std::thread server(servThread);
    server.detach();
    compressor.Disable();
	mainSwerve.Link(&backRightSwerve); // Weird, right? This can in fact be used here.
	backRightSwerve.Link(&frontLeftSwerve);
	frontLeftSwerve.Link(&frontRightSwerve);
	// As it turns out, int main actually still exists and even works here in FRC. I'm tempted to boil it down further and get rid of that stupid StartRobot function (replace it with something custom inside AwesomeRobot).
	return frc::StartRobot<AwesomeRobot<TeleopMode, AutonomousMode, TestMode, DisabledMode>>(); // Look, the standard library does these nested templates more than I do.
}
#endif