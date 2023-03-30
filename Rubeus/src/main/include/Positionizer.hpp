/*
    Use apriltags and and swerve displacement readings to know where you are
    Odometryyyyyyyyyyyyyyyyyyyyyy
*/
#include <AHRS.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DriverStation.h>


enum OdometryQuality {
    AOK, // an AprilTag is being actively tracked
    STALE, // Bearings were established by AprilTag, but there is no longer an apriltag in view (using alternative relative odometry)
    BAD // No apriltags present and bearings have not been established - values should be ignored.
};


template <SwerveModule* swerve> // I'm just doing this for the fun of it, really :D
class Odometry {
    vector lastGoodResult { 0, 0 };
    vector lastResult { 0, 0 };
    std::vector<double> limePos;
    bool isValid = false;
    bool isStale = true;
    std::shared_ptr<nt::NetworkTable> camera;
    double orientation;
    double lastNavxHeading;

public:
    Odometry(const char* camName){
        camera = nt::NetworkTableInstance::GetDefault().GetTable(camName);
    }

    const vector Update(double heading) { // Trophy collected by Tyler: *****  *****
        heading *= PI/180;
        vector ret;
        double tid = camera -> GetNumber("tid", -1);
        if (tid >= 0){
            limePos = camera -> GetEntry("botpose").GetDoubleArray(limePos);
            ret.x = limePos[1];
            ret.y = limePos[0];
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
                ret.y *= -1;
                ret.x -= .8;
            }
            lastGoodResult = ret;
        }
        else{
            //lastGoodResult += swerve -> GetEstimatedDisplacement().rotate(-heading);
            ret = lastGoodResult;
            isStale = true; // If it doesn't have an AprilTag, it's relying on less accurate relative odometry, and is thus stale
        }
        lastResult = ret;
        return ret;
    }

    bool Valid() { // If the values it reports are not garbage
        return isValid;
    }

    bool Good() { // If the values it reports are good
        return isValid && !isStale;
    }

    OdometryQuality Quality(){
        if (isStale){
            if (isValid){
                return OdometryQuality::STALE;
            }
            else{
                return OdometryQuality::BAD;
            }
        }
        return OdometryQuality::AOK;
    }
};
