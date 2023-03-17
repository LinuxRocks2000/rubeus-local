/* Talon FX motor; can be used as a BaseMotor. */
/* TalonSRXMotor but it controls falcons. */


#include <ctre/Phoenix.h> /* Requires Phoenix along with this vendordep */
#include "BaseMotor.hpp"

/**
 @author Tyler Clarke and Luke White
 @version 1.0
 * Talon FX Motor wrapper.
 */

class TalonFXMotor : public BaseMotor{
    TalonFX* talon;
    bool invert = false;
public:
    /**
     * Construct a Talon FX
     @param canID The CAN id of the Talon it's wrapping
     */
    TalonFXMotor (int canID){
        talon = new TalonFX(canID);
    }

    void SetPercent(double speed){
        if (speed < 0) {
            talon -> SetInverted(!invert);
            speed *= -1;
        }
        else {
            talon -> SetInverted(invert);
        }   
        
        talon -> Set(ControlMode::PercentOutput, speed);
    }

    void _setInverted(bool doInv) {
        invert = doInv;
    }
    
    void SetP(double kP){
        talon -> Config_kP(0, kP);
    }

    void SetI(double kI){
        talon -> Config_kI(0, kI);
    }

    void SetD(double kD){
        talon -> Config_kD(0, kD);
    }

    void SetF(double kF){
        talon -> Config_kF(0, kF);
    }

    void SetOutputRange(double kPeakOF, double kPeakOR, double kNominalOF = 0, double kNominalOR = 0){
        talon -> ConfigPeakOutputForward(kPeakOF);
        talon -> ConfigPeakOutputReverse(kPeakOR);
        talon -> ConfigNominalOutputForward(kNominalOF);
        talon -> ConfigNominalOutputReverse(kNominalOR);
    }

    double GetPosition() {
        return talon -> GetSelectedSensorPosition();
    }
    
    double GetVelocity() {
        return talon -> GetSelectedSensorVelocity();
    }
    
    void SetPositionPID(double position){
        if (position < 0) {
            talon -> SetInverted(!invert);
            position *= -1;
        }
        else {
            talon -> SetInverted(invert);
        }   
            
        talon -> Set(ControlMode::Position, position);
    }

    void SetSpeedPID(double speed){
        if (speed < 0) {
            talon -> SetInverted(!invert);
            speed *= -1;
        }
        else {
            talon -> SetInverted(invert);
        }   
        
        talon -> Set(ControlMode::Velocity, speed);
    }
    
    void SetZeroEncoder() {
        talon -> SetSelectedSensorPosition(0);
    }

    double GetCurrent() {
        return talon -> GetOutputCurrent();
    }

    void ConfigIdleToBrake() {
        
    }
};
