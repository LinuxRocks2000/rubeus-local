/*
    Use apriltags and navx displacement readings to know where you are
    Odometryyyyyyyyyyyyyyyyyyyyyy
*/
#include <AHRS.h>
#include <networktables/NetworkTableInstance.h>

struct ApriltagPosition {
    uint8_t id; // ID of this apriltag
    double dX; // x offset in meters of this apriltag
    double dY; // y offset in meters of this apriltag
    float angle = 0; // Angle of the apriltag
};


struct Position2D {
    double x; // In meters
    double y;
};


enum OdometryQuality {
    AOK, // an AprilTag is being actively tracked
    STALE, // Bearings were established by AprilTag, but there is no longer an apriltag in view (using NavX odometry)
    BAD // No apriltags present and bearings have not been established - values are purely NavX.
};


template <uint8_t TagCount, const ApriltagPosition Tags[TagCount], AHRS* Navx> // I'm just doing this for the fun of it, really :D
class Odometry {
    Position2D lastGoodResult { 0, 0 };
    Position2D lastResult { 0, 0 };
    double limePos[6];
    bool isValid = false;
    bool isStale = true;
    std::shared_ptr<nt::NetworkTableInstance> camera;
    double orientation;
    double lastNavxHeading;

public:
    Odometry(char* camName){
        camera = nt::NetworkTableInstance::GetDefault().GetTable(camName);
    }

    const Position2D Update() {
        Position2D ret;
        if (dat.HasTargets()){
            Navx -> ResetDisplacement(); // If there's a valid target, zero the navx displacement
            photonlib::PhotonTrackedTarget targ;
            double bestAmbi = 2; // ambiguity is 0-1, so this is an impossibly high value
            auto targets = dat.GetTargets();
            for (photonlib::PhotonTrackedTarget t : targets){
                if (t.GetPoseAmbiguity() < bestAmbi) {
                    bestAmbi = t.GetPoseAmbiguity();
                    targ = t;
                }
            }
            frc::SmartDashboard::PutNumber("Target ambiguity", targ.GetPoseAmbiguity());
            auto pos = targ.GetBestCameraToTarget();
            ret.x = (double)pos.X();
            ret.y = (double)pos.Y();
            isValid = false; // Reset validity
            isStale = false; // Upon seeing an AprilTag, it is no longer stale
            frc::SmartDashboard::PutNumber("seek id", targ.GetFiducialId());
            for (uint8_t tag = 0; tag < TagCount; tag ++){
                if (Tags[tag].id == targ.GetFiducialId()){
                    vector v { Tags[tag].dX, Tags[tag].dY }; // Apriltag position relative to field
                    vector r { ret.x, ret.y }; // Robot position relative to apriltag
                    vector d = v + r.rotate(Tags[tag].angle); // Robot relative to field
                    ret.x = d.x;
                    ret.y = d.y;
                    isValid = true; // If it can see an AprilTag, and knows where that AprilTag is on the field, then it's reporting valid values.
                    orientation = targ.GetYaw() + Tags[tag].angle * 180/PI;
                }
            }
            lastGoodResult.x = ret.x;
            lastGoodResult.y = ret.y;
        }
        else{
            ret.x = lastGoodResult.x;
            ret.y = lastGoodResult.y;
            //ret.x += Navx -> GetDisplacementX();
            //ret.y += Navx -> GetDisplacementY();
            isStale = true; // If it doesn't have an AprilTag, it's relying on navX odometry, and is thus stale
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

    ApriltagPosition Nearest() {
        int bestI = 0;
        double bestD = 9999999999; // probably not gonna hit this many meters on the field
        for (int i = 0; i < TagCount; i ++){
            double dx = Tags[i].dX - lastResult.x;
            double dy = Tags[i].dY - lastResult.y;
            double hyp2 = dx * dx + dy * dy;
            if (hyp2 < bestD){
                bestD = hyp2;
                bestI = i;
            }
        }
        return Tags[bestI];
    }

    double NearestAngle(){
        ApriltagPosition n = Nearest();
        return n.angle;
        double dX = n.dX - lastResult.x;
        double dY = n.dY - lastResult.y;
        if (dX == 0) { // If the value we pass into atan will be infinity, basically
            return PI/2;
        }
        return atan(dY/dX);
    }
};
