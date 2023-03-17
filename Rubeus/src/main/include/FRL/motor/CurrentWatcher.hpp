/* Keep safe with a current watcher that emits a warning when a dangerous thing happens to a motor */

class CurrentWatcher {
    BaseMotor* watchee;
    double dangerousCurrent;
    double dangerousTime;
    double spikeStartTime = -1;
    double safeTime = -1;
    double cool;
    double coolTime; // After a spike ends, this is set to the timestamp we have to reach to leave cooling mode
    bool wasEndangered; // If, at the last cycle, it was spiking

public:
    bool isEndangered = false;

    CurrentWatcher(BaseMotor* bm, double dangerCurrent, double dangerCurrentSecs, double cooldown = 1){
        watchee = bm;
        dangerousCurrent = dangerCurrent;
        dangerousTime = dangerCurrentSecs;
        cool = cooldown;
    }

    bool Update(){ // Returns whether it's safe or not
        double cTime = (double)frc::Timer::GetFPGATimestamp(); /* Todo: replace with clock code that doesn't depend on wpilib */
        if (watchee -> GetCurrent() > dangerousCurrent){
            if (spikeStartTime == -1){
                spikeStartTime = cTime;
            }
        }
        else {
            if (wasEndangered){
                coolTime = cTime + cool;
            }
            spikeStartTime = -1;
            wasEndangered = false;
        }
        isEndangered = (spikeStartTime != -1) && ((cTime - spikeStartTime) > dangerousTime); // If it's actively in danger
        wasEndangered = isEndangered;
        if (cTime < coolTime){ // If it's not actively spiking, but it's been safe for less than the cooldown time
            isEndangered = true;
        }
        return isEndangered;
    }
};