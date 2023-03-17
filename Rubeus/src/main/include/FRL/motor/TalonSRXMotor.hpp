/* Talon SRX motor; can be used as a BaseMotor. */


#include <ctre/Phoenix.h> /* Requires Phoenix along with this vendordep */
#include "BaseMotor.hpp"

/**
 @author Tyler Clarke and Luke White
 @version 1.0

 * Talon SRX motor wrapper.
 */
class TalonSRXMotor : public BaseMotor{
    /**
     * Internal pointer to a heap-allocated Talon SRX.
     */
    TalonSRX* talon;
public:
    /**
     * Construct a Talon SRX wrapper.
     @param canID The CAN id of the Talon it's wrapping.
     */
    TalonSRXMotor (int canID){
        talon = new TalonSRX(canID);
    }

    ~TalonSRXMotor (){
        delete talon;
    }
    
    void _setInverted(bool invert) {
        talon -> SetInverted(invert);
    }
    
    void SetPercent(double speed){
        /*if (speed < 0) {
            talon -> SetInverted(!invert);
        }
        else {
            talon -> SetInverted(invert);
        }*/
        
        talon -> Set(ControlMode::PercentOutput, speed);
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

    void SetPositionPID(double position){
        talon -> Set(ControlMode::Position, position);
    }

    void SetSpeedPID(double speed){        
        talon -> Set(ControlMode::Velocity, speed);
    }
    
    double GetPosition() {
        return talon -> GetSelectedSensorPosition();
    }
    
    double GetVelocity() {
        return talon -> GetSelectedSensorVelocity();
    }

    double GetCurrent() {
        return 0;
    }

    void ConfigIdleToBrake() {
        talon -> SetNeutralMode(NeutralMode::Brake);
    }
};
