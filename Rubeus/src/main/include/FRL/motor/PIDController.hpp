/* By Tyler Clarke
    PID controls for any conforming BaseMotor
*/

// This is entirely based off the code at https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control, squeezed into a C++ format
#pragma once
#include "BaseMotor.hpp"
#include <frc/Timer.h>

/**
 @author Tyler Clarke
 @version 1.0
 * Coterminality function. I would use modulus, but it doesn't work too well for some reason.

 * This supports doubles anyways which means radians don't fail spectacularly.
 */
double smartLoop(double pos, double round = 4096){
  while (pos > round){
    pos -= round;
  }
  while (pos < 0){
    pos += round;
  }
  return pos;
}


enum PIDSetpointType {
    SPEED,
    POSITION,
    DISABLED
};


/**
 @author Tyler Clarke
 @version 1.0

 * Structure containing PID constants.
 */
struct PIDConstants { // Defaults: configure it yourself if you want it to run
    double P = 0; // Proportional - speed += error * P
    double I = 0; // Integral - sums over time
    double D = 0; // Derivative - damps motion to increase accuracy while decreasing speed (dangerous)
    double F = 0; // Constant feed - speed += F
    double T = 0; // Constant revulsion - speed -= 1 - T * (error^2), where T * error^2 is clamped to 0,1. The goal is to shift the curve forwards, so the fastest moments are in the middle instead of at the start.
    double TMin = 0.25;
    double TMax = 1;
    double iZone = 0;
    double MinOutput = -1;
    double MaxOutput = 1;
};

template <typename T>
concept PIDControllableType = requires(T t, double d){
    t.SetPercent(d);
};


/**
 @author Tyler Clarke
 @version 1.0
 * Class that controls any BaseMotor for PID.

 * Tune coefficients by altering the constants property directly.

 * Only for controlling positions at the moment - speed coming soon.
 */
template <PIDControllableType PIDControllable>
class PIDController {
    /**
     * Setpoint type
     */
    PIDSetpointType mode = POSITION;
    /**
     * Thing to control
     */
    PIDControllable* motor;
    /**
     * Timestamp at the last update
     */
    double lastTime;
    /**
     * Frequency to update at.
     */
    float hz;

    /**
     * Target position
     */
    double setPoint = 0;

    /**
     * Last recorded error (needed for D term calculation)
     */
    double previousError = 0;
    /**
     * Integral state. Added to motor output speed. This accumulates over time, so the motor accelerates.
     */
    double iState = 0;

    /**
     * Current position. Assigned in Update and used mostly for semantic purposes.
     */
    double curPos;

    /**
     * Length of one rotation, if you're looping around a circle.
     */
    long rotationLength = -1; // -1 = no looping

    /**
     * Returns a classically calculated error if the PIDController is not configured to loop-around, otherwise returns the output of loopize.
    
     * Only used internally.
     @param set The setpoint
     @param cur The current position
     */
    double getError(double set, double cur){
        if (rotationLength == -1){
            return set - cur;
        }
        else{
            return loopize(set, cur);
        }
    }

    /**
     * Calculate error between a setpoint and current position *based on the fact that there are always 2 ways to reach any given point on a circle*.
     
     * Used internally only.
     @param set The setpoint
     @param cur The current position
     */
    double loopize(double set, double cur){
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
     * Do PID math given a number of ticks elapsed since last update.
     
     * This way the end result is always the same no matter what the frequency of the processor.
     @param FE Number of ticks elapsed since last update. This is a reference to the similar code in <a href="https://linuxrocks2000.github.io/platformer/platformer-game">Platformer</a>.
     */
    double DoMath(double FE){
        double error = getError(setPoint, curPos);

        double p = error * constants.P; // This does not need to be adjusted for FE

        if (fabs(error) <= constants.iZone || constants.iZone == 0){ // no clue, I'm basically copy pasting. looks like IZone is a "zone" in which the I coefficient applies.
            //iState += std::abs(error)/error * (constants.I) * FE; // *FE means that, if error * constants.I is 2, it will only actually gain 2 after 1 second/hz is passed. (20 ms by default). This keeps it smooth.
            // This kind of thing is used all throughout platformer; very tested and stable
        }
        else{
            iState = 0;
        }

        double d = (error - previousError);
        previousError = error;
        d *= constants.D;

        double t = constants.T * error * error;
        if (t < constants.TMin){
            t = 0;
        }
        if (t > constants.TMax){
            t = 1;
        }
        t = 1 - t;

        //float f = setPoint * constants.F;

        return (p + iState + d + constants.F) * t;
    }

public:
    bool lowSwitch = false;
    bool highSwitch = false;
    /**
     * Constants for PID.
     */
    PIDConstants constants;

    /**
     * Turn on looping-mode and set the circumference of one "circle"
     @param circumference The circumference to loop around
     */
    void SetCircumference(long circumference){
        rotationLength = circumference;
    }
    
    /**
     * Construct the PIDController
     @param m The controllable object to control
     @param frequency The frequency to update PID at. Larger frequencies will result in faster PID code. Default is 50 hz. This is really just here to set a good speed baseline for PID control, but it can also be a quick way to increase ramp speed.
     */
    PIDController (PIDControllable* m, float frequency = 50){ // Update at 50 hz by default
        motor = m;
        hz = frequency;
    }

    /**
     * Update the motor. This is actually just an overload that calls Update(double) with a value from the motor's encoder.
     */
    void Update(){ // Call synchronously at any frequency, this uses Math (tm) to adjust for it
        // Pass in a current position argument to Update if you like that, or just let it figure it out from the motor encoder (see: very cool piece of code)
        if (mode == POSITION){
            Update(motor -> GetPosition());
        }
        else if (mode == SPEED){
            Update(motor -> GetSpeed());
        }
    }

    double speedAccumulated;

    /**
     * Update the motor with a current position specified. Call periodically. The hz-smoothing algorithm means the frequency doesn't matter too much, but try to call it at least as many times per second as the frequency, and preferably not too many more. The algorithm breaks down at the extremes.
     * Call without parameters to use the motor's encoder; pass in a value to use an external encoder. Good for controlling a Neo with a CANCoder (which is literally exactly what we're doing).
     @param cPos Current position to base PID calculations on
     */
    double Update(double cPos){
        if (mode == DISABLED){
            return 0;
        }
        curPos = cPos;
        double secsElapsed = (double)frc::Timer::GetFPGATimestamp() - lastTime;
        double FE = secsElapsed / hz; // This is a trick from my online game. Measures elapsed time and converts it to number of ticks it needs to "draw"!
        // The roborio has at least a few mhz so this will almost never be >1, and will probably hover <0.1 most of the time.
        // We can set up SmartDashboard to track it for performance metrics, if it becomes necessary
        double ret = DoMath(FE);
        if (mode == SPEED){
            speedAccumulated += ret * FE;
            ret = speedAccumulated;
        }
        if (ret > constants.MaxOutput){
            ret = constants.MaxOutput;
        }
        else if (ret < constants.MinOutput){
            ret = constants.MinOutput;
        }
        if ((ret < 0) && lowSwitch){
            ret = 0;
        }
        else if ((ret > 0) && highSwitch){
            ret = 0;
        }
        motor -> SetPercent(ret);
        lastTime = (double)frc::Timer::GetFPGATimestamp();
        return ret;
    }

    /**
     * Return true if it (the motor) has reached a previously assigned target
     @param margin Acceptable error margin
     */
    bool IsAtTarget(double margin){
        if ((curPos > setPoint - margin) && (curPos < setPoint + margin)){
            return true;
        }
        return false;
    }

    /**
     * Set the position setpoint
     @param pos The position to ramp up towards
     */
    void SetPosition(double pos){
        setPoint = pos;
        mode = POSITION;
    }

    /**
     * Set the speed setpoint.
     @param speed The speed to ramp up towards
     */
    void SetSpeed(double speed){
        setPoint = speed;
        mode = SPEED;
    }

    /**
     * Clear all PID
     */
    void Stop(){
        mode = DISABLED;
        speedAccumulated = 0;
        iState = 0;
    }
};