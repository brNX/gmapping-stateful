#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>
#include <utils/stat.h>
#include "gridslamprocessor.h"
#include <boost/detail/endian.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>

#ifdef BOOST_BIG_ENDIAN
#define LITTLE_ENDIAN32(i) { i = swap32(i); }
#else
#define LITTLE_ENDIAN32(i) {}
#endif

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

//#define MAP_CONSISTENCY_CHECK
//#define GENERATE_TRAJECTORIES

namespace GMapping {

const double m_distanceThresholdCheck = 20;

using namespace std;

GridSlamProcessor::GridSlamProcessor() :
    m_infoStream(cout) {

    period_ = 5.0;
    m_obsSigmaGain = 1;
    m_resampleThreshold = 0.5;
    m_minimumScore = 0.;

    context = new  zmq::context_t(1);

}

GridSlamProcessor::GridSlamProcessor(const GridSlamProcessor& gsp) :
    last_update_time_(0.0), m_particles(gsp.m_particles), m_infoStream(cout) {

    period_ = 5.0;

    //context = gsp.context;

    m_obsSigmaGain = gsp.m_obsSigmaGain;
    m_resampleThreshold = gsp.m_resampleThreshold;
    m_minimumScore = gsp.m_minimumScore;

    m_beams = gsp.m_beams;
    m_indexes = gsp.m_indexes;
    m_motionModel = gsp.m_motionModel;
    m_resampleThreshold = gsp.m_resampleThreshold;
    m_matcher = gsp.m_matcher;

    m_count = gsp.m_count;
    m_readingCount = gsp.m_readingCount;
    m_lastPartPose = gsp.m_lastPartPose;
    m_pose = gsp.m_pose;
    m_odoPose = gsp.m_odoPose;
    m_linearDistance = gsp.m_linearDistance;
    m_angularDistance = gsp.m_angularDistance;
    m_neff = gsp.m_neff;

    cerr << "FILTER COPY CONSTRUCTOR" << endl;
    cerr << "m_odoPose=" << m_odoPose.x << " " << m_odoPose.y << " "
         << m_odoPose.theta << endl;
    cerr << "m_lastPartPose=" << m_lastPartPose.x << " " << m_lastPartPose.y
         << " " << m_lastPartPose.theta << endl;
    cerr << "m_linearDistance=" << m_linearDistance << endl;
    cerr << "m_angularDistance=" << m_linearDistance << endl;

    m_xmin = gsp.m_xmin;
    m_ymin = gsp.m_ymin;
    m_xmax = gsp.m_xmax;
    m_ymax = gsp.m_ymax;
    m_delta = gsp.m_delta;

    m_regScore = gsp.m_regScore;
    m_critScore = gsp.m_critScore;
    m_maxMove = gsp.m_maxMove;

    m_linearThresholdDistance = gsp.m_linearThresholdDistance;
    m_angularThresholdDistance = gsp.m_angularThresholdDistance;
    m_obsSigmaGain = gsp.m_obsSigmaGain;

#ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": trajectories copy.... ";
#endif
    TNodeVector v = gsp.getTrajectories();
    for (unsigned int i = 0; i < v.size(); i++) {
        m_particles[i].node = v[i];
    }
#ifdef MAP_CONSISTENCY_CHECK
    cerr << "end" << endl;
#endif

    cerr
            << "Tree: normalizing, resetting and propagating weights within copy construction/cloneing ...";
    updateTreeWeights(false);
    cerr << ".done!" << endl;
}

GridSlamProcessor::GridSlamProcessor(std::ostream& infoS) :
    m_infoStream(infoS) {
    cout << "GridSlamProcessor ostream" << endl;
    period_ = 5.0;
    m_obsSigmaGain = 1;
    m_resampleThreshold = 0.5;
    m_minimumScore = 0.;
    context = new  zmq::context_t(1);

    cout << "GridSlamProcessor MAINTHREAD: start" << endl;

}

GridSlamProcessor* GridSlamProcessor::clone() const {
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing preclone_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++) {
        const ScanMatcherMap& m1(it->map);
        const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
        for (int x=0; x<h1.getXSize(); x++) {
            for (int y=0; y<h1.getYSize(); y++) {
                const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
                if (a1.m_reference) {
                    PointerMap::iterator f=pmap.find(a1.m_reference);
                    if (f==pmap.end())
                        pmap.insert(make_pair(a1.m_reference, 1));
                    else
                        f->second++;
                }
            }
        }
    }
    cerr << __PRETTY_FUNCTION__ << ": Number of allocated chunks" << pmap.size() << endl;
    for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
        assert(it->first->shares==(unsigned int)it->second);

    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
    GridSlamProcessor* cloned = new GridSlamProcessor(*this);

# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": trajectories end" << endl;
    cerr << __PRETTY_FUNCTION__ << ": performing afterclone_fit_test" << endl;
    ParticleVector::const_iterator jt=cloned->m_particles.begin();
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++) {
        const ScanMatcherMap& m1(it->map);
        const ScanMatcherMap& m2(jt->map);
        const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
        const HierarchicalArray2D<PointAccumulator>& h2(m2.storage());
        jt++;
        for (int x=0; x<h1.getXSize(); x++) {
            for (int y=0; y<h1.getYSize(); y++) {
                const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
                const autoptr< Array2D<PointAccumulator> >& a2(h2.m_cells[x][y]);
                assert(a1.m_reference==a2.m_reference);
                assert((!a1.m_reference) || !(a1.m_reference->shares%2));
            }
        }
    }
    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
    return cloned;
}

GridSlamProcessor::~GridSlamProcessor() {

    cout << "MAINTHREAD: sending stop "<< endl;

    threads.join_all();
    cout << "MAINTHREAD: done" << endl;
    //TODO: delete remaining queue here or in respective threads
    delete syncqueue;
    delete context;
    cerr << __PRETTY_FUNCTION__ << ": Start" << endl;
    cerr << __PRETTY_FUNCTION__ << ": Deleting tree" << endl;
    for (std::vector<Particle>::iterator it = m_particles.begin();
         it != m_particles.end(); it++) {
#ifdef TREE_CONSISTENCY_CHECK		
        TNode* node=it->node;
        while(node)
            node=node->parent;
        cerr << "@" << endl;
#endif
        if (it->node)
            delete it->node;
        //cout << "l=" << it->weight<< endl;
    }

# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing predestruction_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++) {
        const ScanMatcherMap& m1(it->map);
        const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
        for (int x=0; x<h1.getXSize(); x++) {
            for (int y=0; y<h1.getYSize(); y++) {
                const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
                if (a1.m_reference) {
                    PointerMap::iterator f=pmap.find(a1.m_reference);
                    if (f==pmap.end())
                        pmap.insert(make_pair(a1.m_reference, 1));
                    else
                        f->second++;
                }
            }
        }
    }
    cerr << __PRETTY_FUNCTION__ << ": Number of allocated chunks" << pmap.size() << endl;
    for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
        assert(it->first->shares>=(unsigned int)it->second);
    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
}

void GridSlamProcessor::setMatchingParameters(double urange, double range,
                                              double sigma, int kernsize, double lopt, double aopt, int iterations,
                                              double likelihoodSigma, double likelihoodGain,
                                              unsigned int likelihoodSkip) {
    m_obsSigmaGain = likelihoodGain;
    m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt,
                                    iterations, likelihoodSigma, likelihoodSkip);
    if (m_infoStream)
        m_infoStream << " -maxUrange " << urange << " -maxUrange " << range
                     << " -sigma     " << sigma << " -kernelSize " << kernsize
                     << " -lstep " << lopt << " -lobsGain " << m_obsSigmaGain
                     << " -astep " << aopt << endl;

}

void GridSlamProcessor::setMotionModelParameters(double srr, double srt,
                                                 double str, double stt) {
    m_motionModel.srr = srr;
    m_motionModel.srt = srt;
    m_motionModel.str = str;
    m_motionModel.stt = stt;

    if (m_infoStream)
        m_infoStream << " -srr " << srr << " -srt " << srt << " -str " << str
                     << " -stt " << stt << endl;

}

void GridSlamProcessor::setUpdateDistances(double linear, double angular,
                                           double resampleThreshold) {
    m_linearThresholdDistance = linear;
    m_angularThresholdDistance = angular;
    m_resampleThreshold = resampleThreshold;
    if (m_infoStream)
        m_infoStream << " -linearUpdate " << linear << " -angularUpdate "
                     << angular << " -resampleThreshold " << m_resampleThreshold
                     << endl;
}

//HERE STARTS THE BEEF

/*Particle::Particle(const ScanMatcherMap& m) :
    map(m), pose(0, 0, 0), weight(0), weightSum(0), gweight(0), previousIndex(
                                                                    0) {
    node = 0;
}*/

void GridSlamProcessor::setSensorMap(const SensorMap& smap) {

    /*
     Construct the angle table for the sensor

     FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot
     */

    SensorMap::const_iterator laser_it = smap.find(std::string("FLASER"));
    if (laser_it == smap.end()) {
        cerr << "Attempting to load the new carmen log format" << endl;
        laser_it = smap.find(std::string("ROBOTLASER1"));
        assert(laser_it!=smap.end());
    }
    const RangeSensor* rangeSensor =
            dynamic_cast<const RangeSensor*>((laser_it->second));
    assert(rangeSensor && rangeSensor->beams().size());

    m_beams = static_cast<unsigned int>(rangeSensor->beams().size());
    double* angles = new double[rangeSensor->beams().size()];
    for (unsigned int i = 0; i < m_beams; i++) {
        angles[i] = rangeSensor->beams()[i].pose.theta;
    }
    m_matcher.setLaserParameters(m_beams, angles, rangeSensor->getPose());
    delete[] angles;
}

void GridSlamProcessor::init(unsigned int size, double xmin, double ymin,
                             double xmax, double ymax, double delta, OrientedPoint initialPose) {
    m_xmin = xmin;
    m_ymin = ymin;
    m_xmax = xmax;
    m_ymax = ymax;
    m_delta = delta;
    if (m_infoStream)
        m_infoStream << " -xmin " << m_xmin << " -xmax " << m_xmax << " -ymin "
                     << m_ymin << " -ymax " << m_ymax << " -delta " << m_delta
                     << " -particles " << size << endl;


    std::string line;

    ifstream ifile(FILENAME);
    if (!ifile){
        cout << "workers.txt not found" << endl;
        exit(-1);
    }


    //syncqueue
    syncqueue = new zmq::socket_t(*context, ZMQ_PULL);
    syncqueue->bind("inproc://syncqueue");
    syncqueue->bind("tcp://*:20003");

    //pub/sub
    broadcastQueue = new zmq::socket_t(*context, ZMQ_PUB);
    broadcastQueue->bind("tcp://*:20000");
    broadcastQueue->bind("inproc://sensorpubqueue");

    sleep(1);

    threads.create_thread(boost::bind(&GridSlamProcessor::responseThread,this));

    int localcounter=0;
    int remotecounter=0;

    int particleindex=0;

    while(getline(ifile,line)){

        boost::algorithm::trim(line);
        if (line.size()>0){
            if (line.compare("local")==0){
                localcounter++;
            }else{

                std::vector<std::string> tokens;
                boost::split(tokens,line, boost::is_any_of(","));

                int percentage=0;
                try {
                    percentage = boost::lexical_cast<int>(tokens[1]);
                } catch( boost::bad_lexical_cast const& ) {
                    std::cout << "Error: input string was not valid, skipping line" << std::endl;
                    continue;
                }

                //TODO: try catch error

                //workerqueue
                zmq::socket_t * workerqueue = new zmq::socket_t(*context, ZMQ_PUSH);
                workerqueue->connect(tokens[0].c_str());
                workerqueues.push_back(workerqueue);

                //distribute indexes
                gmapping_structs::indexMessage indexmessage;
                int numberofparticles = size  / (100.0 / (float)percentage);
                for(int i = particleindex; i < (particleindex + numberofparticles);i++){
                    particleIndextoWorkerMap[i]=remotecounter;
                    indexmessage.add_resampleindexes(i);

                    std::cout <<  "added index " << i <<  " to worker " <<remotecounter <<std::endl;
                }

                indexmessage.set_id(remotecounter);
                std::cout << "index message size of array: " << indexmessage.resampleindexes_size() << std::endl;

                //send indexmessage to worker
                int datasize = indexmessage.ByteSize();
                char dataarray[datasize];

                std::cout.setf(std::ios::hex, std::ios::basefield);
                for(int i = 0; i<datasize;i++){
                    std::cout << (int)dataarray[i] << std::endl;
                }

                indexmessage.SerializeToArray(dataarray,datasize);

                std::cout.unsetf(std::ios::hex);
                std::cout << "message size " << datasize << std::endl;
                std::cout.setf(std::ios::hex, std::ios::basefield);

                for(int i = 0; i<datasize;i++){
                    std::cout << (int)dataarray[i] <<std::endl;
                }

                std::cout.unsetf(std::ios::hex);

                zmq::message_t message(datasize);
                memcpy(message.data(),dataarray,datasize);
                workerqueue->send(message);

                remotecounter++;
                particleindex+=numberofparticles;
            }
        }

    }

    remoteworkercounter = remotecounter;


    int remainingparticles = size - particleindex;
    int chunk = remainingparticles / localcounter;

    //distribute remaining particles between the localthreads
    for (int i=0;i<localcounter-1;i++){
        threads.create_thread(boost::bind(&GridSlamProcessor::localWorker,this,i,particleindex,chunk));

        for (int j = particleindex ; j < particleindex + chunk; j++){
            particleIndextoWorkerMap[j]=-1;
        }

        particleindex+=chunk;

    }

    threads.create_thread(boost::bind(&GridSlamProcessor::resampleThread,this));

    threads.create_thread(boost::bind(&GridSlamProcessor::localWorker,this,localcounter-1,particleindex,size-particleindex));
    for (unsigned int i=particleindex;i<size;i++){
        particleIndextoWorkerMap[i]=-1;
    }

    m_particles.clear();
    TNode* node = new TNode(initialPose, 0, 0, 0);
    ScanMatcherMap lmap(Point(xmin + xmax, ymin + ymax) * .5, xmax - xmin,
                        ymax - ymin, delta);
    for (unsigned int i = 0; i < size; i++) {
        m_particles.push_back(Particle(lmap));
        m_particles.back().pose = initialPose;
        m_particles.back().previousPose = initialPose;
        m_particles.back().setWeight(0);
        m_particles.back().previousIndex = 0;

        // this is not needed
        //		m_particles.back().node=new TNode(initialPose, 0, node, 0);

        // we use the root directly
        m_particles.back().node = node;
    }
    m_neff = (double) size;
    m_count = 0;
    m_readingCount = 0;
    m_linearDistance = m_angularDistance = 0;
}

void GridSlamProcessor::processTruePos(const OdometryReading& o) {
    const OdometrySensor* os =
            dynamic_cast<const OdometrySensor*>(o.getSensor());
    if (os && os->isIdeal() && m_outputStream) {
        m_outputStream << setiosflags(ios::fixed) << setprecision(3);
        m_outputStream << "SIMULATOR_POS " << o.getPose().x << " "
                       << o.getPose().y << " ";
        m_outputStream << setiosflags(ios::fixed) << setprecision(6)
                       << o.getPose().theta << " " << o.getTime() << endl;
    }
}

bool GridSlamProcessor::processScan(const RangeReading & reading,
                                    int adaptParticles) {

    string sensorid("S");
    gmapping_structs::Sensordata data;

    /**retireve the position from the reading, and compute the odometry*/
    OrientedPoint relPose = reading.getPose();
    if (!m_count) {
        m_lastPartPose = m_odoPose = relPose;
    }

    //write the state of the reading and update all the particles using the motion model
    for (unsigned int i = 0; i < m_particles.size(); i++) {
        m_particles[i].pose =  m_motionModel.drawFromMotion(m_particles[i].pose, relPose,m_odoPose);
    }

    // update the output file
    /*if (m_outputStream.is_open()){
     m_outputStream << setiosflags(ios::fixed) << setprecision(6);
     m_outputStream << "ODOM ";
     m_outputStream << setiosflags(ios::fixed) << setprecision(3) << m_odoPose.x << " " << m_odoPose.y << " ";
     m_outputStream << setiosflags(ios::fixed) << setprecision(6) << m_odoPose.theta << " ";
     m_outputStream << reading.getTime();
     m_outputStream << endl;
     }
     if (m_outputStream.is_open()){
     m_outputStream << setiosflags(ios::fixed) << setprecision(6);
     m_outputStream << "ODO_UPDATE "<< m_particles.size() << " ";
     for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
     OrientedPoint& pose(it->pose);
     m_outputStream << setiosflags(ios::fixed) << setprecision(3) << pose.x << " " << pose.y << " ";
     m_outputStream << setiosflags(ios::fixed) << setprecision(6) << pose.theta << " " << it-> weight << " ";
     }
     m_outputStream << reading.getTime();
     m_outputStream << endl;
     }*/

    //invoke the callback
    onOdometryUpdate();

    // accumulate the robot translation and rotation
    OrientedPoint move = relPose - m_odoPose;
    move.theta = atan2(sin(move.theta), cos(move.theta));
    m_linearDistance += sqrt(move * move);
    m_angularDistance += fabs(move.theta);

    // if the robot jumps throw a warning
    if (m_linearDistance > m_distanceThresholdCheck) {
        cerr
                << "***********************************************************************"
                << endl;
        cerr
                << "********** Error: m_distanceThresholdCheck overridden!!!! *************"
                << endl;
        cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
        cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y
             << " " << m_odoPose.theta << endl;
        cerr << "New Odometry Pose (reported from observation)= " << relPose.x
             << " " << relPose.y << " " << relPose.theta << endl;
        cerr
                << "***********************************************************************"
                << endl;
        cerr
                << "** The Odometry has a big jump here. This is probably a bug in the   **"
                << endl;
        cerr
                << "** odometry/laser input. We continue now, but the result is probably **"
                << endl;
        cerr
                << "** crap or can lead to a core dump since the map doesn't fit.... C&G **"
                << endl;
        cerr
                << "***********************************************************************"
                << endl;
    }

    m_odoPose = relPose;

    bool processed = false;

    // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
    if (!m_count || m_linearDistance >= m_linearThresholdDistance
            || m_angularDistance >= m_angularThresholdDistance
            || (period_ >= 0.0
                && (reading.getTime() - last_update_time_) > period_)) {
        last_update_time_ = reading.getTime();

        /*if (m_outputStream.is_open()) {
            m_outputStream << setiosflags(ios::fixed) << setprecision(6);
            m_outputStream << "FRAME " << m_readingCount;
            m_outputStream << " " << m_linearDistance;
            m_outputStream << " " << m_angularDistance << endl;
        }*/

        if (m_infoStream)
            m_infoStream << "update frame " << m_readingCount << endl
                         << "update ld=" << m_linearDistance << " ad="
                         << m_angularDistance << endl;

        cerr << "Laser Pose= " << reading.getPose().x << " "
             << reading.getPose().y << " " << reading.getPose().theta
             << endl;

        //this is for converting the reading in a scan-matcher feedable form
        assert(reading.size()==m_beams);
        double * plainReading = new double[m_beams];
        for (unsigned int i = 0; i < m_beams; i++) {
            plainReading[i] = reading[i];
        }
        m_infoStream << "m_count " << m_count << endl;

        RangeReading* reading_copy = new RangeReading(reading.size(),
                                                      &(reading[0]),
                static_cast<const RangeSensor*>(reading.getSensor()),
                reading.getTime());


        //other scans
        if (m_count > 0) {

            pbufhelper.serializeSensorDataLaserReading(reading,m_particles,&data);

            //send data to workers
            int datasize = data.ByteSize();
            char dataarray[datasize];
            data.SerializeToArray(dataarray,datasize);


            zmq::message_t sensordatamsg(datasize);
            memcpy(sensordatamsg.data(),dataarray,datasize);

            zmq::message_t idmessage(sensorid.size());
            memcpy(idmessage.data(),sensorid.data(),sensorid.size());

            broadcastQueue->send(idmessage,ZMQ_SNDMORE);
            broadcastQueue->send(sensordatamsg);


            //wait for sync
            for (unsigned int i=0; i<m_particles.size();i++){
                zmq::message_t sync;
                syncqueue->recv(&sync);
                /*std::string synctext(static_cast<char*>(sync.data()),sync.size());
                std::cout << synctext << std::endl;*/
            }

            onScanmatchUpdate();

            updateTreeWeights(false);

            if (m_infoStream) {
                m_infoStream << "neff= " << m_neff << endl;
            }

            resample(plainReading, adaptParticles, reading_copy);

        } else { //first scan
            m_infoStream << "Registering First Scan" << endl;

            for (unsigned int i = 0; i < m_particles.size(); i++) {
                m_matcher.invalidateActiveArea();
                m_matcher.computeActiveArea(m_particles[i].map, m_particles[i].pose, plainReading);
                m_matcher.registerScan(m_particles[i].map, m_particles[i].pose, plainReading);

                // cyr: not needed anymore, particles refer to the root in the beginning!
                TNode* node = new TNode(m_particles[i].pose, 0., m_particles[i].node, 0);
                //node->reading=0;
                node->reading = reading_copy;
                m_particles[i].node = node;
            }
            string initialmessageid("I");

            //distribute particles among workers, send scanmatcher settings
            if (remoteworkercounter>0){

                //create message with inital pose and scanmatchersettings
                gmapping_structs::StartPackage startpackage;
                pbufhelper.serializeStartPackage(*this,m_matcher,m_motionModel,plainReading,&startpackage);


                int datasize = startpackage.ByteSize();
                char dataarray[datasize];
                startpackage.SerializeToArray(dataarray,datasize);

                zmq::message_t initialdatamsg(datasize);
                memcpy(initialdatamsg.data(),dataarray,datasize);

                zmq::message_t idmessage(initialmessageid.size());
                memcpy(idmessage.data(),initialmessageid.data(),initialmessageid.size());

                broadcastQueue->send(idmessage,ZMQ_SNDMORE);
                broadcastQueue->send(initialdatamsg);

                //wait for sync from resample thread number of remote workers
                for (int i=0; i<remoteworkercounter;i++){
                    zmq::message_t sync;
                    syncqueue->recv(&sync);
                    /*std::string synctext(static_cast<char*>(sync.data()),sync.size());
                    std::cout << synctext << std::endl;*/
                }
            }

        }


        //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
        updateTreeWeights(false);
        //		cerr  << ".done!" <<endl;

        delete[] plainReading;
        m_lastPartPose = m_odoPose; //update the past pose for the next iteration
        m_linearDistance = 0;
        m_angularDistance = 0;
        m_count++;
        processed = true;


        //keep ready for the next step
        for (unsigned int i = 0; i < m_particles.size(); i++) {
            m_particles[i].previousPose = m_particles[i].pose;
        }

    }

    if (m_outputStream.is_open())
        m_outputStream << flush;
    m_readingCount++;
    return processed;
}

std::ofstream& GridSlamProcessor::outputStream() {
    return m_outputStream;
}

std::ostream& GridSlamProcessor::infoStream() {
    return m_infoStream;
}

int GridSlamProcessor::getBestParticleIndex() const {
    unsigned int bi = 0;
    double bw = -std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < m_particles.size(); i++)
        if (bw < m_particles[i].weightSum) {
            bw = m_particles[i].weightSum;
            bi = i;
        }
    return (int) bi;
}

void GridSlamProcessor::onScanmatchUpdate() {
}
void GridSlamProcessor::onResampleUpdate() {
}
void GridSlamProcessor::onOdometryUpdate() {
}

void GridSlamProcessor::localWorker(const int id, const int startindex, const int size){

    std::cout << "THREAD localWorker "<< id << " started "<<std::endl;

    int particlesindex[size];

    for (int i=0;i<size;i++){
        particlesindex[i]=startindex+i;
    }

    zmq::socket_t pubsubqueuesubscriber(*context, ZMQ_SUB);
    pubsubqueuesubscriber.connect("inproc://sensorpubqueue");
    pubsubqueuesubscriber.setsockopt(ZMQ_SUBSCRIBE, "S",1);//  Subscribe on Sensordata

    zmq::socket_t syncqueuesubscriber(*context, ZMQ_PUSH);
    syncqueuesubscriber.connect("inproc://syncqueue");

    gmapping_structs::Sensordata sensor_data;
    double reading[m_beams];

    for(;;){

        zmq::message_t envelope;
        zmq::message_t sensorsdata;
        pubsubqueuesubscriber.recv(&envelope);
        pubsubqueuesubscriber.recv(&sensorsdata);

        if (sensorsdata.size()==0){
            printf("THREAD localWorker %d ending\n",id);
            break;
        }

        sensor_data.ParseFromArray(sensorsdata.data(),sensorsdata.size());
        //deserialize laserdata
        for(unsigned int i = 0 ; i< m_beams;i++){
            reading[i]=sensor_data.plainreading(i);
        }


        for(int i=0;i<size;i++){

            int indexp=particlesindex[i];

            //scanmatch if needed
            scanMatchLocal(reading,indexp);

            //sync
            /*std::string syncstring("localworker");
            zmq::message_t smsg(syncstring.size());
            memcpy(smsg.data(),syncstring.data(),syncstring.size());
            syncqueuesubscriber.send(smsg);*/
            syncqueuesubscriber.send(0,0);
        }

    }
}

void GridSlamProcessor::responseThread(){
    printf("THREAD responseThread started\n");

    zmq::socket_t remoteworkerresponsequeue(*context, ZMQ_PULL);
    remoteworkerresponsequeue.bind("tcp://*:20001");
    remoteworkerresponsequeue.bind("inproc://remoteworkerresponsequeue");

    zmq::socket_t syncqueuesubscriber(*context, ZMQ_PUSH);
    syncqueuesubscriber.connect("inproc://syncqueue");

    gmapping_structs::WorkResponse reply;

    for (;;){

        zmq::message_t reply_message;
        remoteworkerresponsequeue.recv(&reply_message);

        if (reply_message.size()==0){
            std::cout << "THREAD responseThread ending" << std::endl;
            break;
        }

        if (unlikely(!reply.ParseFromArray(reply_message.data(),reply_message.size()))) {
            std::cerr << "Failed to parse workresponse." << std::endl;
            exit(-1);
        }

        int id = reply.id();
        //std::cout << "response received with id " << id << std::endl;
        //if (reply.has_weight()){
            //std::cout << "has weight " << reply.weight() << std::endl;
            //pbufhelper.deserializeReply(reply,m_particles[id]);
        //}
        pbufhelper.deserializeReply(reply,m_particles[id]);

        /*std::string syncstring("syncresponse");
        zmq::message_t smsg(syncstring.size());
        memcpy(smsg.data(),syncstring.data(),syncstring.size());
        syncqueuesubscriber.send(smsg);*/
        syncqueuesubscriber.send(0,0);
    }

}

void GridSlamProcessor::resampleThread(){

    //TODO: caching using round/counter number in request

    std::cout << "THREAD resampleThread started" << std::endl;

    zmq::socket_t remotesamplequeue(*context, ZMQ_REP);
    remotesamplequeue.bind("tcp://*:20010");

    zmq::socket_t syncqueuesubscriber(*context, ZMQ_PUSH);
    syncqueuesubscriber.connect("inproc://syncqueue");

    unsigned int roundcounter=0;
    std::map<int,gmapping_structs::Particle> particlecache;

    google::protobuf::io::GzipOutputStream::Options options;
    options.format = google::protobuf::io::GzipOutputStream::ZLIB;
    options.compression_level = 1;

    std::string workstring;


    for (;;){

        zmq::message_t requestmessage;
        remotesamplequeue.recv(&requestmessage);

        gmapping_structs::resampleMessage request;

        //if particles request -> respond with particles;
        if (requestmessage.size()>0){
            request.ParseFromArray(requestmessage.data(),requestmessage.size());

            std::cout << "resamplerequest received" << std::endl;


            //shutdown signal
            /*if (request.has_finish()){
                //ok message
                remotesamplequeue.send(0,0);
                break;
            }*/

            if (request.resamplecounter()>roundcounter){
                particlecache.clear();
                roundcounter=request.resamplecounter();
            }

            gmapping_structs::Particles particles;

            for (int i=0; i<request.resampleindexes_size(); i++){
                gmapping_structs::Particle * particle=particles.add_particles();

                //in cache ?
                std::map<int,gmapping_structs::Particle>::iterator it;
                it = particlecache.find(request.resampleindexes(i));
                if (it!=particlecache.end()){

                    *particle = it->second;

                }else{
                    pbufhelper.serializeParticle(m_particles[request.resampleindexes(i)],particle,request.resampleindexes(i));
                    particlecache[request.resampleindexes(i)]=*particle;
                }
            }

            //send data to workers

            /*int datasize = particles.ByteSize();
            char dataarray[datasize];
            particles.SerializeToArray(dataarray,datasize);*/

            //send data to workers
            workstring.clear();
            {
                google::protobuf::io::StringOutputStream compressedStream(&workstring);
                google::protobuf::io::GzipOutputStream compressingStream(&compressedStream, options);
                particles.SerializeToZeroCopyStream(&compressingStream);
            }

            /*zmq::message_t particlesmessage(datasize);
            memcpy(particlesmessage.data(),dataarray,datasize);
            remotesamplequeue.send(particlesmessage);*/

            zmq::message_t particlesmessage((void*)workstring.c_str(),workstring.length(),NULL,NULL); //zero copy
            remotesamplequeue.send(particlesmessage);


        }else{
            //ok message
            remotesamplequeue.send(0,0);

            /*std::string syncstring("resamplesync");
            zmq::message_t smsg(syncstring.size());

            memcpy(smsg.data(),syncstring.data(),syncstring.size());
            //if done request -> sync
            syncqueuesubscriber.send(smsg);*/
            syncqueuesubscriber.send(0,0);
        }
    }

}



}
// end namespace

