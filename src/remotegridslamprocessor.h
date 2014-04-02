#ifndef REMOTEGRIDSLAMPROCESSOR_H
#define REMOTEGRIDSLAMPROCESSOR_H

#include <climits>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
#include <utils/macro_params.h>
#include <utils/point.h>
#include <scanmatcher/scanmatcher.h>
#include <gridfastslam/motionmodel.h>
#include <gridfastslam/gridslamprocessor.h>
#include "protobuf/protobufhelper.h"

namespace ProtoBuf{
class ProtobufHelper;
}

namespace GMapping {


class RemoteGridSlamProcessor
{
    friend class ProtoBuf::ProtobufHelper;

public:
    RemoteGridSlamProcessor();
    ~RemoteGridSlamProcessor();
    void init(unsigned int size, double xmin, double ymin,
              double xmax, double ymax, double delta, OrientedPoint initialPose,std::vector<int> & indexes);

    void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt,
                               int iterations, double likelihoodSigma=1, double likelihoodGain=1, unsigned int likelihoodSkip=0);
    //void setMotionModelParameters(double srr, double srt, double str, double stt);

    //void scanMatch(const double* plainReading,GMapping::Particle & particle,gmapping_structs::WorkResponse & reply_);

    void processScan(double * plainreading, zmq::socket_t & remotersesponsequeue,int index);
    void registerScan(double * plainreading,bool nullWeight);
    void resample(zmq::message_t & resamplemessage,zmq::socket_t & remotesamplequeue,zmq::socket_t & syncqueue,int min,int max,double* plainreading);


    /**the scanmatcher algorithm*/
    ScanMatcher m_matcher;

    /**the particles*/
    ParticleVector m_particles;

    /**the particle indexes*/
    std::vector<unsigned int> m_indexes;

    /**the motion model*/
    MotionModel m_motionModel;


    //accessor methods
    /**the maxrange of the laser to consider */
    MEMBER_PARAM_SET_GET(m_matcher, double, laserMaxRange, protected, public, public);

    /**the maximum usable range of the laser. A beam is cropped to this value. [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, usableRange, protected, public, public);

    /**The sigma used by the greedy endpoint matching. [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher,double, gaussianSigma, protected, public, public);

    /**The sigma  of a beam used for likelihood computation [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher,double, likelihoodSigma, protected, public, public);

    /**The kernel in which to look for a correspondence[scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, int,    kernelSize, protected, public, public);

    /**The optimization step in rotation [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, optAngularDelta, protected, public, public);

    /**The optimization step in translation [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, optLinearDelta, protected, public, public);

    /**The number of iterations of the scanmatcher [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, unsigned int, optRecursiveIterations, protected, public, public);

    /**the beams to skip for computing the likelihood (consider a beam every likelihoodSkip) [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, unsigned int, likelihoodSkip, protected, public, public);

    /**translational sampling range for the likelihood [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, llsamplerange, protected, public, public);

    /**angular sampling range for the likelihood [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, lasamplerange, protected, public, public);

    /**translational sampling range for the likelihood [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, llsamplestep, protected, public, public);

    /**angular sampling step for the likelihood [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, lasamplestep, protected, public, public);

    /**generate an accupancy grid map [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, bool, generateMap, protected, public, public);

    /**enlarge the map when the robot goes out of the boundaries [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, bool, enlargeStep, protected, public, public);

    /**pose of the laser wrt the robot [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, OrientedPoint, laserPose, protected, public, public);


    /**odometry error in translation as a function of translation (rho/rho) [motionmodel]*/
    STRUCT_PARAM_SET_GET(m_motionModel, double, srr, protected, public, public);

    /**odometry error in translation as a function of rotation (rho/theta) [motionmodel]*/
    STRUCT_PARAM_SET_GET(m_motionModel, double, srt, protected, public, public);

    /**odometry error in rotation as a function of translation (theta/rho) [motionmodel]*/
    STRUCT_PARAM_SET_GET(m_motionModel, double, str, protected, public, public);

    /**odometry error in  rotation as a function of rotation (theta/theta) [motionmodel]*/
    STRUCT_PARAM_SET_GET(m_motionModel, double, stt, protected, public, public);

    /**minimum score for considering the outcome of the scanmatching good*/
    PARAM_SET_GET(double, minimumScore, protected, public, public);


    //processing parameters (size of the map)
    PARAM_GET(double, xmin, protected, public);
    PARAM_GET(double, ymin, protected, public);
    PARAM_GET(double, xmax, protected, public);
    PARAM_GET(double, ymax, protected, public);
    //processing parameters (resolution of the map)
    PARAM_GET(double, delta, protected, public);


    void getMissingParticles(zmq::socket_t& remotesamplequeue, std::set<int> missingids, gmapping_structs::Particles particles, gmapping_structs::resampleMessage neededParticles);
private:
    ProtoBuf::ProtobufHelper pbufhelper;

};

}

#endif // REMOTEGRIDSLAMPROCESSOR_H
