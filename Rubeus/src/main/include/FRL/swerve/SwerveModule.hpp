/* By Luke White and Tyler Clarke
    Custom swerve module library using PIDController.
*/
// Note: This and PIDController.hpp need some more cleanup work.

#pragma once

#include <FRL/motor/BaseMotor.hpp>
#include <ctre/Phoenix.h>
#include <iostream>
#include <FRL/motor/PIDController.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <FRL/util/vector.hpp>
#include <frc/Timer.h>

/**
 @author Luke White and Tyler Clarke
 @version 1.0
 
 * Swerve module for FRC. Manages 2 BaseMotor pointers (which because of the polymorphism can be any motor type in FRC)
 */
class SwerveModule {
    /**
     * Motor that controls the rotation of the wheel
     */
    BaseMotor* speed;
    /**
     * Motor that controls the direction of the wheel
     */
    BaseMotor* direction;
    /**
     * PIDController that manages the direction motor
     */
    PIDController<BaseMotor>* directionController;
    /**
     * PIDController that manages the speed motor
     */
    PIDController<BaseMotor>* speedController;
    /**
     * CANCoder to use for PID; heap allocated by an ID provided on construction.
     */
    CANCoder* cancoder;

    /**
     * Current percentage that will be applied to the wheel
     */
    double curPercent; // So multiple commands can alter speed
       
    /**
     * SwerveModules are a linked list! This means you can have any number of 'em configured with separate offsets and command them all at once with a single call.
     */
    SwerveModule* linkSwerve;

    /**
     * Whether or not the SwerveModule is linked to another one.
     */
    bool isLinked = false;  

    /**
     * Configured offset.
     */
    double encoderOffset;

    float lockTime = -1; // Don't ever lock
    double lockStart = -1; // Time that it decided locking was necessary
    
    bool locked = false;
public:
    short swerveRole;
    bool readyToOrient = false;

    /**
     * Constructor
     @param speedMotor The motor to use for wheel speed control
     @param directionMotor The motor to use for wheel direction control
     @param CanCoderID The CAN id of the CANCoder
     @param offset The offset of the wheel, in encoder ticks
     @param speedInverted Whether or not to invert the wheel speed motor
     @param direcInverted Whether or not to invert the wheel direction motor
     */
    SwerveModule(BaseMotor* speedMotor, BaseMotor* directionMotor, int CanCoderID, short role, double offset, bool speedInverted=false, bool direcInverted=false) {
        encoderOffset = offset;
        speed = speedMotor;
        direction = directionMotor;
        cancoder = new CANCoder {CanCoderID};
        
        swerveRole = role;
        directionController = new PIDController<BaseMotor> (direction);
        speedController = new PIDController<BaseMotor> (speed);
        directionController -> constants.P = 0.0005;
        //directionController -> constants.I = 0.0001;
        directionController -> constants.MaxOutput = 0.2;
        directionController -> constants.MinOutput = -0.2;
        directionController -> SetCircumference(4096);

        speedController -> constants.P = 0.005;
        speedController -> constants.D = 0.0015;

        speed -> SetInverted(speedInverted);
        direction -> SetInverted(direcInverted);

        speed -> ConfigIdleToBrake();
        direction -> ConfigIdleToBrake();
    }

    void SetLockTime(float lT, bool followLink = true){
        lockTime = lT;
        if (followLink && isLinked){
            linkSwerve -> SetLockTime(lT);
        }
    }
    
    /**
     * Link to another swerve module
     @param LinkSwerve The swerve module to link to
     */
    void Link(SwerveModule* LinkSwerve) {
        isLinked = true;           
        linkSwerve = LinkSwerve; 
    }
    
    /**
     * Get the current (physical) direction of the module
     */
    long GetDirection() {
        if (speed -> inversionState){
            return smartLoop(2048 + (cancoder -> GetAbsolutePosition() - encoderOffset));
        }
        else{
            return smartLoop(cancoder -> GetAbsolutePosition() - encoderOffset);
        }
    }

     /**
      * Return true if within a certain deadband.
      @param num The current number
      @param dead The current deadband
      @param reference The reference point (defaulted to zero)
     */

    bool withinDeadband(double num, double dead, double reference = 0) {
        num -= reference; // So reference becomes 0
        
        return (num < dead) &&
               (num > -dead);
    }

    bool allAtLocation(double loc) {
        if (isLinked) {
            return linkSwerve -> withinDeadband(linkSwerve -> GetDirection(), 30, loc) && withinDeadband(GetDirection(), 30, loc);
        }
        return withinDeadband(GetDirection(), 30, loc);
    }

    /**
     * Set the direction of the motor.
     @param targetPos The encoder tick to aim for
     @param followLink Whether or not to follow its link
     */
    
    void SetDirection(double targetPos, bool followLink = true, bool ignoreLock = false) {
        if (locked && !ignoreLock){ // Can't set direction if it's locked
            return;
        }
        if (std::abs(loopize(targetPos, GetDirection(), 4096)) > 1524){ // All turns >90 degrees can be optimized with 180 reversion logic.
            speed -> SetInverted();
        }

        directionController -> SetPosition(targetPos);
        directionController -> Update(GetDirection());

        if (isLinked && followLink){
            linkSwerve -> SetDirection(targetPos);
        }
    }

    void SetSpeed(double targetSpeed, bool followLink = true){
        speedController -> SetSpeed(targetSpeed);
        speedController -> Update(GetSpeed());

        if (isLinked && followLink){
            linkSwerve -> SetSpeed(targetSpeed);
        }
    }

    /**
     Increase the speed of the wheel motor
     @param spd Percentage to add
     @param followLink Whether or not to command the linked swerve module, if it exists
     */
    void SetPercent(double spd, bool followLink = true){
        curPercent += spd;
        if (isLinked && followLink){
            linkSwerve -> SetPercent(spd);
        }
    }

    /**
     Apply a percentage to the wheel motor
     */
    void ApplySpeed(){
        locked = false;
        speed -> SetPercent(curPercent);

        if (lockTime != -1){
            if (curPercent == 0) { // If nothin' done been did
                if (lockStart == -1){
                    lockStart = (double)frc::Timer::GetFPGATimestamp();
                }
                if ((double)frc::Timer::GetFPGATimestamp() - lockStart > lockTime){
                    Lock(false); // locking is done on a per-module basis
                    locked = true;
                }
            }
            else{
                lockStart = -1;
            }
        }

        curPercent = 0; // Velocity ain't "sticky", this is a safety thing
        if (isLinked){
            linkSwerve -> ApplySpeed();
        }
    } 
    
    /**
     * Make sure all wheels are ready for orientation change
     */

    bool allReadyToOrient() {
        if (isLinked){
            return linkSwerve -> allReadyToOrient() && readyToOrient;
        }
        return readyToOrient;
    }

    /**
     * Orient the swerve drive 
     @param angle The desired angle (in encoder ticks)
     @param currentAngle The current navX angle of the robot (also in encoder ticks)
     */

    bool Orient(int angle, int currentAngle) {
        if (angle == -1) {        // If the POV is not being currently pressed
            return true;
        }

        else {
            if ((swerveRole == 1 || swerveRole == 3)) {          // If top-left or botton-right
                SetDirection((4096/360) * 45, false);          // Go at 45 degrees
                if (withinDeadband(GetDirection(), 3, (4096/360) * 45)) {         // If there
                    readyToOrient = true;                    // Ready to orient; when all of them are ready, the speed will set
                }
            }   

            else {
                SetDirection((4096/360) * 315, false);
                if (withinDeadband(GetDirection(), 3, (4096/360) * 315)) {
                    readyToOrient = true;
                }
            }

            if (allReadyToOrient()) {
                
            }

            if (isLinked) {
                linkSwerve -> Orient(angle, currentAngle);    // Makes the linkSwerve act like a void, because it kinda is
                // The bool value is ignored if you don't use it by default ;)
            }
            return withinDeadband(angle, 5, currentAngle);
        }
        return false;
    }

    /**
     * Return whether or not it's reached a position
     */

    bool IsAtPosition(double position, float dead = 30){
        return withinDeadband(GetDirection(), dead, position);
    }

    /**
     * Orient this and all linked swerve modules to form the Orb pattern (x,y movement locked, rotational movement enabled)
    
     * Returns whether or not the swerve drive is fully in Orb mode.
     */

    bool Orb(){
        double angle = smartLoop(4096/360 * (360 - swerveRole * 90 + 45));
        SetDirection(angle, false);
        bool ret = IsAtPosition(angle);
        if (isLinked){
            return linkSwerve -> Orb() && ret;
        } // Everything after this if is basically inside an else{ because the function exits inside this if.
        return ret;
    }


    /**
     * Set the swerve module to a few FRL vectors
     */
    void SetToVector(vector translation, vector rotation, bool followLink = true) {
        if (isLinked && followLink){
            linkSwerve -> SetToVector(translation, rotation);
        }
        if (translation.isZero() && rotation.isZero()){
            if (!locked){
                direction -> SetPercent(0); // Just stop movin' direction
            }
            return; // Don't do nothin' - it'll all sort itself out
        }
        vector mein = translation + rotation.rotate(PI/2 * swerveRole);
        SetDirection(mein.angle() * 2048/PI, false);
        SetPercent(mein.magnitude(), false);
    }


    /**
     * Lock this and all linked swerve modules
     */
    bool Lock(bool followLink = true){
        double angle = smartLoop(4096/360 * (360 - swerveRole * 90 + 135));
        SetDirection(angle, false, true);
        bool ret = IsAtPosition(angle);
        if (followLink && isLinked){
            return linkSwerve -> Lock() && ret;
        } // Everything after this if is basically inside an else{ because the function exits inside this if.
        return ret;
    }

    double loopize(double set, double cur, double rotationLength = 360){
        if (std::abs(set - cur) >= rotationLength/2){
            if (set > cur){
                return -(rotationLength - set + cur);
            }
            else{
                return rotationLength - cur + set;
            }
        }
        else{
            return set - cur;
        }
    }

    /**
     * Temporary overload.
     */
    double iState = 0;
    double lastTime = -1;
    bool Orient(double current, double angle, bool tuba){
        if (angle == -1){
            iState = 0;
            lastTime = (double)frc::Timer::GetFPGATimestamp();
            return false;
        }
        if (lastTime == -1){
            lastTime = (double)frc::Timer::GetFPGATimestamp();
        }
        double secsElapsed = (double)frc::Timer::GetFPGATimestamp() - lastTime;
        current = smartLoop(current, 360);
        angle = smartLoop(angle, 360);
        frc::SmartDashboard::PutNumber("current", current);
        frc::SmartDashboard::PutNumber("angle", angle);
        double error = loopize(angle, current);
        double p = error * -0.002;
        SetPercent(p + iState);
        if (error < -3){
            if (iState > 0){
                iState = 0;
            }
            iState += 0.01 * secsElapsed;
        }
        else if (error > 3){
            if (iState < 0){
                iState = 0;
            }
            iState -= 0.01 * secsElapsed;
        }
        else{
            iState = 0;
        }
        return false;
    }

    void NoOrient(){
        iState = 0;
        lastTime = -1;
        if (isLinked){
            linkSwerve -> NoOrient();
        }
    }

    double GetSpeed(){
        return speed -> GetVelocity();
    }

    double GetAverageLinkSpeed(){
        double totes = 0;
        int cnt = 1;
        SwerveModule* link = this;
        while (link -> isLinked){
            totes += link -> GetSpeed();
            cnt ++;
            link = link -> linkSwerve;
        }
        totes += link -> GetSpeed();
        return totes/cnt;
    }

    double lastLinkPosition = 0;

    vector GetDenormalizedVector(){
        vector ret;
        double pos = speed -> GetPosition();
        double speed = pos - lastLinkPosition;
        lastLinkPosition = pos;
        ret.setMandA(speed, GetDirection() * PI/2048);
        return ret;
    }

    vector GetEstimatedDisplacement(){
        vector ret;
        int cnt = 1;
        SwerveModule* link = this;
        while (link -> isLinked){
            ret += link -> GetDenormalizedVector();
            cnt ++;
            link = link -> linkSwerve;
        }
        ret += link -> GetDenormalizedVector();
        ret.x /= cnt;
        ret.y /= cnt;
        return ret;
    }
};