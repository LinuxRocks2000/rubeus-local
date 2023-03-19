#include <FRL/motor/BaseMotor.hpp>
#include <frc/AnalogInput.h>
#include <FRL/motor/PIDController.hpp>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <FRL/motor/CurrentWatcher.hpp>
#include "controls.hpp"
#include <FRL/util/functions.hpp>

const double shoulderBarLengthCM = 91.44; // Length of the forearm in centimeters
const double elbowBarLengthCM = 91.44; // What can I call it? Anti-forearm? Arm? Elbow-y bit? Yeah. Elbow-y bit. This is the elbow-y bit length in centimeters.

const double shoulderDefaultAngle = 80; // I calculated. At displacement x 5, displacement y is 30. So it's atan(30/5). Which is about 80.5 degrees.
const double elbowDefaultAngle = 280; // Reflect the angle of the shoulder about the x axis
// TODO: use sane units (radians).

enum GrabMode {
    OFF,
    INTAKE,
    BARF,
    SHOOT
};


struct ArmPosition {
    float x;
    float y;
};

const vector lowPole { 110, 76.36 };
const vector highPole { 110, 106.84 };
const vector shootHigh { 110, 115 };
const vector home { 35, 0 };

struct ArmInfo {
/*
    Arm math structure, to clean up.
    Generates values for the encoders, and stores all the intermediate calculations.
    Units: Degrees (needs to be radians).

    This is virtual math only; the hardware translation layer (class Arm) should convert to encoder ticks.
    Usage:
    armInfo.goal = ...; // goal position
    armInfo.curHeadX = ...; // current head x position
    armInfo.curHeadY = ...; // current head y position
    armInfo.Update(); // convert goal pos to angles
    // RestrictOutputs goes here, because RestrictOutputs is a post-processing thing
*/
    double omega; // Elbow composites - omega = true goal angle, theta = goal dist from shoulder
    double theta;
    double y; // Angle between the appropriate vertical line and the shoulder
    double a; // Angle between the appropriate vertical line and the elbow
    double n; // Real (goal) angle of the shoulder
    double f; // Angle between goal-vector and shoulder
    double x; // Angle of the goal-vector
    double s;


    vector goal; // The program sets these and uses gamma and n.
    double curHeadX;
    double curHeadY;
    // Current head position is needed for Restrict...() functions.

    double GetTheta(){
        double base = goal.magnitude() / 2;
        double thetaOverTwo = asin(base / shoulderBarLengthCM) * 180/PI;
        return smartLoop(thetaOverTwo * 2, 360);
        // It's an isoscelese triangle. That means we can divide it up into two right triangles about the base and theta; each one has an angle of F
        // and an angle of theta/2, and the base is the original base/2. Also, each one has a hypotenuse of shoulderBarLengthCM (or elbowBarLengthCM; they're the same),
        // Sin is defined as opposite/hypotenuse (SOHCAHTOA!), so to get that top angle (which is, remember, theta/2), we use asin(base/shoulderBarLengthCM). The
        // resulting angle is exactly half of theta, so we multiply by two and convert to degrees.
    }

    void Update (){ // Do the calculations to set *every* variable
        x = smartLoop(goal.angle() * 180/PI, 360);
        theta = GetTheta();
        f = smartLoop((180 - theta) / 2, 360); // F is an angle in an isoscelese triangle. It is equal to d. The equation for any triangle is,
        // f + d + theta = 180; since this is an isoscelese triangle, we know that f == d, and f * 2 + theta = 180. Solve for f,
        // we get f = (180 - theta) / 2.
        n = smartLoop(f + x, 360); // n = f + x. Check the chart.
        y = smartLoop(90 - n, 360); // Yep.
        a = smartLoop(theta - y, 360); // Check the chart.
        omega = smartLoop(270 + a - 10, 360); // Check das chart.
        s = smartLoop(180 - a, 360); // Note: s and gamma are related, but 360 - s != gamma.
    }

    void RestrictOutputs(double shoulderMax, double shoulderMin, double elbowMax, double elbowMin){
        assert(shoulderMax > shoulderMin); // C++ feature: assert evaluates an expression, and if it's false throws an error.
        assert(elbowMax > elbowMin); // Insane values for shoulder and elbow mins/maxs could throw off the entire program.
        if (omega > elbowMax){
            omega = elbowMax;
        }
        else if (omega < elbowMin){
            omega = elbowMin;
        }
        if (n > shoulderMax){
            n = shoulderMax;
        }
        else if (n < shoulderMin){
            n = shoulderMin;
        }
    }
};


template <int elbowID, int shoulderID, int boopID, int elbowLimitswitchID, int shoulderLimitswitchID>
class Arm {
public:
    long elbowDefaultEncoderTicks = 0;
    long shoulderDefaultEncoderTicks = 0;

    double trim = 0;
    bool disabled = false;
    bool doneBarfing = false;
    bool handState = false;
    BaseMotor* shoulder;
    BaseMotor* elbow;
    BaseMotor* hand;
    PIDController<BaseMotor>* elbowController;
    PIDController<BaseMotor>* shoulderController;
    CurrentWatcher* shoulderWatcher;
    CurrentWatcher* elbowWatcher;
    std::vector<ArmPosition> stack;
    vector goalPos;
    ArmInfo info;
    bool retract = false;
    bool sweeping = false;
    frc::DigitalInput elbowLimitSwitch { elbowLimitswitchID };
    frc::DigitalInput shoulderLimitSwitch { shoulderLimitswitchID };
    frc::AnalogInput elbowEncoder { elbowID };
    frc::AnalogInput shoulderEncoder { shoulderID };
    frc::DigitalInput boop { boopID };
    vector curPos;

    Arm(BaseMotor* s, BaseMotor* e, BaseMotor* h){
        shoulder = s;
        elbow = e;
        hand = h;
        elbowController = new PIDController(e);
        elbowController -> constants.P = 0.002;
        elbowController -> constants.I = 0;
        //elbowController -> constants.D = 0.05;
        //elbowController -> constants.F = -0.05;
        elbowController -> constants.MinOutput = -0.2;
        elbowController -> constants.MaxOutput = 0.3;
        shoulderController = new PIDController(s);
        shoulderController -> constants.P = 0.007;
        shoulderController -> constants.I = 0;
        shoulderController -> constants.D = 0.04;
        //shoulderController -> constants.F = 0.15;
        shoulderController -> constants.MinOutput = -0.3;
        shoulderController -> constants.MaxOutput = 0.3;
        elbowController -> SetCircumference(4096);
        shoulderController -> SetCircumference(4096);
        shoulder -> ConfigIdleToBrake();
        elbow -> ConfigIdleToBrake();
        hand -> ConfigIdleToBrake();
        shoulderWatcher = new CurrentWatcher { shoulder, 60, 0.25 };
        elbowWatcher = new CurrentWatcher { elbow, 35, 0.25 };
    }

    void test(){
        //goalX += 0.002;
        //vector goal = { 60, 5 };
        //frc::SmartDashboard::PutNumber("Goal X", goal.x);
        //shoulderController -> SetPosition(halfPos);
        frc::SmartDashboard::PutNumber("Shoulder real", shoulderEncoder.GetValue());
        frc::SmartDashboard::PutNumber("Shoulder nice", GetShoulderPos());
        frc::SmartDashboard::PutNumber("Elbow real", elbowEncoder.GetValue());
        frc::SmartDashboard::PutNumber("Elbow normal", GetNormalizedElbow());
        frc::SmartDashboard::PutNumber("Elbow nice", GetElbowPos());
        vector p = GetArmPosition();
        frc::SmartDashboard::PutNumber("Head X", p.x);
        frc::SmartDashboard::PutNumber("Head Y", p.y);
        frc::SmartDashboard::PutNumber("Shoulder Current", shoulder -> GetCurrent());
        frc::SmartDashboard::PutNumber("Elbow Current", elbow -> GetCurrent());
        frc::SmartDashboard::PutBoolean("Elbow Danger", !elbowWatcher -> isEndangered);
        frc::SmartDashboard::PutBoolean("Shoulder Danger", !shoulderWatcher -> isEndangered);
        frc::SmartDashboard::PutBoolean("Hand switch", boop.Get());
        //armGoToPos(lowPole);
        //frc::SmartDashboard::PutNumber("Shoulder goal", GetShoulderGoalFrom(goal));
        //frc::SmartDashboard::PutNumber("Elbow goal", GetElbowGoalFrom(goal));
        //frc::SmartDashboard::PutNumber("Shoulder goal ticks", ShoulderAngleToEncoderTicks(GetShoulderGoalFrom(goal)));
        //frc::SmartDashboard::PutNumber("Elbow goal ticks", ElbowAngleToEncoderTicks(GetElbowGoalFrom(goal), GetShoulderGoalFrom(goal)));
    }

    GrabMode grabMode;
    
    void goToHome(bool triggerSol = false) {
        armGoToPos({30, 0});
    }

    void goToPickup() {
        armGoToPos({65, -15});
    }

    void goToLowPole(bool high = false) {
        armGoToPos({lowPole});
    }

    void goToHighPole() {
        armGoToPos({highPole});
    }

    void goToHighCone() {
        armGoToPos({125, 120});
    }

    bool checkSwitches() {
        frc::SmartDashboard::PutBoolean("elbow switch", elbowLimitSwitch.Get()); // elbow is Normally Open because c'est messed up
        frc::SmartDashboard::PutBoolean("shoulder switch", shoulderLimitSwitch.Get()); // shoulder is, as proper, Normally Closed
        if (shoulderLimitSwitch.Get() && elbowLimitSwitch.Get()){
            shoulderDefaultEncoderTicks = shoulderEncoder.GetValue();
            elbowDefaultEncoderTicks = elbowEncoder.GetValue();
            return true;
        }
        return false;
    }

    void armGoToPos(vector pos) {
        goalPos = pos;
        if ((curPos.x < 60) && (curPos.x > 30)){
            if (curPos.y < 0){
                if (std::abs(curPos.x - 35) < std::abs(curPos.x - 60)){ // if it's closer to 35 than 60
                    goalPos.x = 35;
                }
                else{
                    goalPos.x = 55;
                }
            }
            goalPos.y = 5;
        }
    }

    int GetNormalizedShoulder(){
        return smartLoop(shoulderDefaultEncoderTicks - shoulderEncoder.GetValue());
    }

    int GetNormalizedElbow(){
        return smartLoop(elbowDefaultEncoderTicks - elbowEncoder.GetValue());
    }

    double GetShoulderPos(){ // Get the shoulder angle in degrees relative to the ground
        return smartLoop(shoulderDefaultAngle - (GetNormalizedShoulder() * 360/4096), 360); // Todo: USE DANG RADIANS
    }

    double GetElbowPos(){ // Ditto
        return smartLoop(elbowDefaultAngle + 10 + (GetNormalizedElbow() * 360/4096) - (90 - GetShoulderPos()), 360);
    }

    // TODO: use radians
    // Actually todo: use sane code instead of this garbage

    double GetShoulderRadians(){
        return GetShoulderPos() * PI/180;
    }

    double GetElbowRadians(){
        return GetElbowPos() * PI/180;
    }

    vector GetArmPosition(){
        vector ret;
        double shoulderRads = GetShoulderRadians();
        double elbowRads = GetElbowRadians();
        double elbowX = cos(elbowRads) * elbowBarLengthCM;
        double elbowY = sin(elbowRads) * elbowBarLengthCM;
        double shoulderX = cos(shoulderRads) * shoulderBarLengthCM;
        double shoulderY = sin(shoulderRads) * shoulderBarLengthCM;
        ret.x = shoulderX + elbowX;
        ret.y = shoulderY + elbowY;
        return ret;
    }

    double ElbowAngleToEncoderTicks(double ang, double shoulder){
        return smartLoop(elbowDefaultEncoderTicks - (ang - elbowDefaultAngle + (90 - shoulder)) * 4096/360);
    }

    double ShoulderAngleToEncoderTicks(double ang) {
        return smartLoop((ang - shoulderDefaultAngle) * 4096/360 + shoulderDefaultEncoderTicks); 
    }

    double setX = 35;
    void setRetract(bool s = true) {
        retract = s;
    }

    void armPickup(bool triggerSol = true) {
        goToPickup();
        /*if (!retract) {
            if (!sweeping) {
                armGoToPos({setX, -10});
                if (atGoal()) {
                    sweeping = true;
                }
            }
            else {
                armGoToPos({setX, -10});
                hand -> SetPercent(.35);
                setX += .005;
                if (Has() || setX > 150) {
                    retract = true;
                }
            }
        }*/
    }

    double sAng, eAng;

    bool atGoal(vector goal){
        vector current = GetArmPosition();
        return withinDeadband(current.x, 5, goal.x) && withinDeadband(current.y, 7, goal.y);
        //return (std::abs(shoulderEncoder.GetValue() - sAng) < 40) && (std::abs(elbowEncoder.GetValue() - eAng) < 40);
    }

    bool atY(double val) {
        double curr = GetArmPosition().y;
        return withinDeadband(curr, 7, val);
    }

    bool zeroed = false;

    vector lastPos;

    void Update(){
        shoulderController -> highSwitch = shoulderLimitSwitch.Get();
        elbowController -> highSwitch = elbowLimitSwitch.Get();
        if (!disabled) {
            if (grabMode == INTAKE){
                if (!boop.Get()){
                    hand -> SetPercent(0);
                }
                else {
                    hand -> SetPercent(-0.3);
                }
            }
            else if (grabMode == BARF){
                hand -> SetPercent(0.2);
            }
            else if (grabMode == SHOOT){
                hand -> SetPercent(1);
            }
            else {
                hand -> SetPercent(0);
            }
        }
        shoulderWatcher -> Update();
        elbowWatcher -> Update();
        if (!zeroed){
            //std::cout << "is zero" << std::endl;
            AuxSetPercent(0.4, 0.1);
            zeroed = checkSwitches();
            return;
        }
        
        /*if (retract){
            goToHome();
            if (atGoal()){
                retract = false;
            }
        }*/
        //std::cout << std::endl;
        checkSwitches();
        if (elbowWatcher -> isEndangered || shoulderWatcher -> isEndangered){
            //std::cout << "is endangered" << std::endl;
            AuxSetPercent(0, 0);
            return;
        }

        vector pos = GetArmPosition();
        curPos = pos;
        info.goal = goalPos;
        lastPos = pos;
        info.curHeadX = pos.x;
        info.curHeadY = pos.y;
        info.Update();
        double useN = info.n;
        double useOmega = info.omega;
        //info.RestrictOutputs(shoulderDefaultAngle, 0, 360, elbowDefaultAngle);
        sAng = ShoulderAngleToEncoderTicks(useN);
        eAng = ElbowAngleToEncoderTicks(useOmega, info.n) + trim;
        shoulderController -> SetPosition(sAng);
        elbowController -> SetPosition(eAng);
        frc::SmartDashboard::PutNumber("Shoulder goal", sAng);
        frc::SmartDashboard::PutNumber("Elbow goal", eAng);
        frc::SmartDashboard::PutNumber("Head Goal X", info.goal.x);
        frc::SmartDashboard::PutNumber("Head Goal Y", info.goal.y);
        frc::SmartDashboard::PutNumber("Current x", GetArmPosition().x);
        frc::SmartDashboard::PutNumber("Current y", GetArmPosition().y);

        frc::SmartDashboard::PutNumber("Shouldah", shoulderController -> Update(shoulderEncoder.GetValue()));
        if (!disabled) {
            elbowController -> Update(elbowEncoder.GetValue());
            shoulderController -> Update(shoulderEncoder.GetValue());
        }
        else {
            AuxSetPercent(0, 0);
        }
        grabMode = OFF; // ain't sticky - don't want breakies
    }

    void ShimZero(){
        zeroed = false;
    }

    void Shim(double s){
        if (!zeroed){
            AuxSetPercent(0, 0.1);
            zeroed = checkSwitches();
            return;
        }
        shoulderWatcher -> Update();
        elbowWatcher -> Update();
        if (elbowWatcher -> isEndangered || shoulderWatcher -> isEndangered){
            AuxSetPercent(0, 0);
            return;
        }
        double goal = smartLoop(elbowDefaultEncoderTicks + s);
        frc::SmartDashboard::PutNumber("Goal", goal);
        frc::SmartDashboard::PutNumber("Default", elbowDefaultEncoderTicks);
        elbowController -> SetPosition(goal);
        if (!disabled) {
            elbowController -> Update(elbowEncoder.GetValue());
        }
        else {
            AuxSetPercent(0, 0);
        }
    }

    void SetShimTrim(double t) {
        trim = t;
    }

    void SetDisabled(bool state) {
        disabled = state;
    }

    void ShimPickup(){
        Shim(-900 - trim);
    }

    void ShimPlaceLow(){
        Shim(-315 - trim);
    }

    void ShimPlaceHigh(){
        Shim(-830 - trim);
    }

    double ShimGet() {
        return GetNormalizedElbow();
    }

    void ShimHand(){
        /*if (!disabled) {
            if (grabMode == INTAKE){
                hand -> SetPercent(0.2);
            }
            else if (grabMode == BARF){
                hand -> SetPercent(-0.1);
            }
            else {
                hand -> SetPercent(0);
            }
        }
        grabMode = OFF; // ain't sticky - don't want breakies*/
    }

    void ShimHome() {
        //ShimZero();//Shim(0); // unlike the other shim(0) calls this one should stay as shim(0)
        if (!elbowAtLimit()){
            AuxSetPercent(0, 0.1);
        }
    }

    bool Has(){
        return !boop.Get();
    }

    void SetGrab(GrabMode mode){
        grabMode = mode;
    }

    bool shoulderAtLimit(){ // These do *not* return the state of the limit switch; they return whether or not the respective motor is at its limit. Thus they also include watchers in their math.
        return shoulderLimitSwitch.Get() || shoulderWatcher -> isEndangered;
    }

    bool elbowAtLimit(){
        return elbowLimitSwitch.Get() || elbowWatcher -> isEndangered;
    }

    void Zero(){
        //std::cout << "zero" << std::endl;
        zeroed = false;
    }

    void AuxSetPercent(double s, double e){
        shoulderWatcher -> Update();
        elbowWatcher -> Update();
        if ((shoulderLimitSwitch.Get() && (s > 0)) || shoulderWatcher -> isEndangered){
            s = 0;
        }
        if ((elbowLimitSwitch.Get() && (e > 0)) || elbowWatcher -> isEndangered){
            e = 0;
        }
        shoulder -> SetPercent(s);
        elbow -> SetPercent(e);
    }
};