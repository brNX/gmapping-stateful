#include "remotegridslamprocessor.h"
#include <utils/point.h>
#include <scanmatcher/smmap.h>
#include <gridfastslam/motionmodel.h>


//#define DADOS

namespace GMapping {

RemoteGridSlamProcessor::RemoteGridSlamProcessor()
{
    m_minimumScore = 0.;
}

RemoteGridSlamProcessor::~RemoteGridSlamProcessor()
{
}

void RemoteGridSlamProcessor::init(unsigned int size, double xmin, double ymin,
                                   double xmax, double ymax, double delta, OrientedPoint initialPose,std::vector<int> & indexes) {
    m_xmin = xmin;
    m_ymin = ymin;
    m_xmax = xmax;
    m_ymax = ymax;
    m_delta = delta;

    m_particles.clear();
    ScanMatcherMap lmap(Point(xmin + xmax, ymin + ymax) * .5, xmax - xmin,
                        ymax - ymin, delta);

    for (unsigned int i = 0; i < size; i++) {
        m_particles.push_back(Particle(lmap));
        m_particles.back().pose = initialPose;
        m_particles.back().previousPose = initialPose;
        m_particles.back().setWeight(0);
        m_particles.back().previousIndex = 0;

        m_indexes.push_back(indexes[i]);
    }

    std::cout << "indexes received: ";
    for (unsigned int i=0; i<indexes.size();i++){
        std::cout << indexes[i] << " ";
    }
    std::cout << std::endl;
}

/*void RemoteGridSlamProcessor::scanMatch(const double* plainReading , GMapping::Particle & particle, gmapping_structs::WorkResponse & reply_) {

    GMapping::OrientedPoint corrected;
    double score, l, s;
    score = m_matcher.optimize(corrected, particle.map, particle.pose, plainReading);
    //    it->pose=corrected;
    if (score > m_minimumScore) {
        particle.pose = corrected;
    }

    gmapping_structs::OrientedPoint * pbufpose = reply_.mutable_pose();
    pbufpose->set_x(particle.pose.x);
    pbufpose->set_y(particle.pose.y);
    pbufpose->set_theta(particle.pose.theta);


    m_matcher.likelihoodAndScore(s, l, particle.map, particle.pose, plainReading);

    reply_.set_weight(particle.weight + l);
    reply_.set_weightsum(particle.weightSum + l);

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
    m_matcher.invalidateActiveArea();
    GMapping::IntPoint min,max;

    if (m_matcher.computeActiveAreaRemote(particle.map, particle.pose, plainReading,min,max)){
        reply_.set_minx(min.x);
        reply_.set_miny(min.y);
        reply_.set_maxx(max.x);
        reply_.set_maxy(max.y);
    }

    const std::set< GMapping::point<int>, GMapping::pointcomparator<int> > & activearea = particle.map.storage().getActiveArea();

    //activearea
    for (std::set<GMapping::point<int> >::iterator it = activearea.begin(); it != activearea.end(); ++it)
    {
        gmapping_structs::IntPoint * point = reply_.add_m_activearea();
        point->set_x(it->x);
        point->set_y(it->y);
    }
}*/

void RemoteGridSlamProcessor::registerScan(double * plainReading, bool nullWeight){
    for (unsigned int i = 0; i < m_particles.size(); i++){
        m_matcher.invalidateActiveArea();
        m_matcher.registerScan(m_particles[i].map, m_particles[i].pose, plainReading);
        if (nullWeight){
            m_particles[i].setWeight(0);
        }
    }
}

void RemoteGridSlamProcessor::processScan(double * plainReading, zmq::socket_t &remotersesponsequeue, int index){

        gmapping_structs::WorkResponse reply;

        //scanmatch
        OrientedPoint corrected;
        double score, l, s;
        score = m_matcher.optimize(corrected, m_particles[index].map, m_particles[index].pose, plainReading);
        //    it->pose=corrected;
        if (score > m_minimumScore) {
            m_particles[index].pose = corrected;
        }

        m_matcher.likelihoodAndScore(s, l, m_particles[index].map, m_particles[index].pose, plainReading);
        m_particles[index].weight += l;
        m_particles[index].weightSum += l;

        reply.set_weight(m_particles[index].weight);
        reply.set_weightsum(m_particles[index].weightSum);

        //set up the selective copy of the active area
        //by detaching the areas that will be updated
        m_matcher.invalidateActiveArea();

        GMapping::IntPoint min,max;

        if (m_matcher.computeActiveAreaRemote(m_particles[index].map, m_particles[index].pose, plainReading,min,max)){
            reply.set_minx(min.x);
            reply.set_miny(min.y);
            reply.set_maxx(max.x);
            reply.set_maxy(max.y);
        }

        const std::set< GMapping::point<int>, GMapping::pointcomparator<int> > & activearea = m_particles[index].map.storage().getActiveArea();

        //activearea
        for (std::set<GMapping::point<int> >::iterator it = activearea.begin(); it != activearea.end(); ++it)
        {
            gmapping_structs::IntPoint * point = reply.add_m_activearea();
            point->set_x(it->x);
            point->set_y(it->y);
        }


        gmapping_structs::OrientedPoint * pbufpose = reply.mutable_pose();
        pbufpose->set_x(m_particles[index].pose.x);
        pbufpose->set_y(m_particles[index].pose.y);
        pbufpose->set_theta(m_particles[index].pose.theta);


        reply.set_id(m_indexes[index]);

        //std::cout << "sending reply with particle "<< i <<std::endl;

        int replysize = reply.ByteSize();
        char sendarray[replysize];

        reply.SerializeToArray(sendarray,replysize);

        zmq::message_t reply_message(replysize);
        memcpy(reply_message.data(),sendarray,replysize);
        remotersesponsequeue.send(reply_message);

        //send it
        /*zmq::message_t reply_message(&sendarray,replysize,NULL,NULL); //zero copy
        remotersesponsequeue.send(reply_message);*/

}

void RemoteGridSlamProcessor::resample(zmq::message_t & resamplemessage, zmq::socket_t & remotesamplequeue, zmq::socket_t & syncqueue, int min, int max,double * plainReading){
    //resample neeeded
    if (resamplemessage.size()>0){

        gmapping_structs::resampleMessage rmessage;
        rmessage.ParseFromArray(resamplemessage.data(),resamplemessage.size());

        //std::cout << "resamplemessage id " << rmessage.resamplecounter()<< std::endl;

        std::set<int> missingids;
        std::cout << "resample indexes: " ;
        for (unsigned int i =0; i<rmessage.resampleindexes_size() ; i++){
            std::cout << rmessage.resampleindexes(i) <<  " " ;
        }
        std::cout << std::endl;

        //
        for (int i = min; i <= max; i++) {
            unsigned int rindex = rmessage.resampleindexes(i);
            if ( rindex < min || rindex > max){
                missingids.insert(rindex);
                //std::cout << " " <<  rindex << " " ;
            }
        }

        std::cout << "missing ids: ";
        for (std::set<int>::iterator it = missingids.begin(); it != missingids.end(); ++it)
        {
           std::cout << *it << " " ;
        }
        std::cout << std::endl;


        gmapping_structs::resampleMessage neededParticles;
        neededParticles.set_resamplecounter(rmessage.resamplecounter());


        GMapping::ParticleVector temp;


        if (missingids.size()>0){

            gmapping_structs::Particles particles;

            //request particles

            for (std::set<int>::iterator it = missingids.begin(); it != missingids.end(); ++it)
            {
                neededParticles.add_resampleindexes(*it);
            }
            int datasize = neededParticles.ByteSize();
            char dataarray[datasize];
            neededParticles.SerializeToArray(dataarray,datasize);

            zmq::message_t neededParticlesmessage(datasize);
            memcpy(neededParticlesmessage.data(),dataarray,datasize);
            remotesamplequeue.send(neededParticlesmessage);

            //std::cout << "request sent" <<std::endl;

            zmq::message_t particlesmessage;
            remotesamplequeue.recv(&particlesmessage);

            //std::cout << "response received" <<std::endl;

            google::protobuf::io::ArrayInputStream input(particlesmessage.data(), particlesmessage.size());
            google::protobuf::io::GzipInputStream decompressingStream(&input);

            particles.ParseFromZeroCopyStream(&decompressingStream);

            //particles.ParseFromArray(particlesmessage.data(),particlesmessage.size());

            //exchange particles
            for (int i = min,index=0; i <= max; i++,index++) {
                unsigned int rindex = rmessage.resampleindexes(i);

                if ( rindex < min || rindex > max){
                    for(int j = 0 ; j < particles.particles_size();j++){
                        if(rindex==particles.mutable_particles(j)->id()){
                            GMapping::ScanMatcherMap map(1,1,5);
                            GMapping::Particle p(map);
                            pbufhelper.deserializeParticle(*particles.mutable_particles(j),p);
                            p.previousIndex=rindex;
                            temp.push_back(p);
                            break;
                        }
                    }
                }else{
                    for(unsigned int j = 0 ; j < m_indexes.size();j++){
                        if(rindex==m_indexes[j]){
                            m_particles[j].previousIndex=rindex;
                            temp.push_back(m_particles[j]);
                            break;
                        }
                    }
                }
            }



            /*registerScan(plainReading,true);


            //sync
            remotesamplequeue.send(0,0);
            zmq::message_t dummy;
            remotesamplequeue.recv(&dummy);*/

        }else{ //everything needed available locally

            for (int i = min,index=0; i <= max; i++,index++) {
                unsigned int rindex = rmessage.resampleindexes(i);

                for(unsigned int j = 0 ; j < m_indexes.size();j++){
                    if(rindex==m_indexes[j]){
                        m_particles[j].previousIndex=rindex;
                        temp.push_back(m_particles[j]);
                        break;
                    }
                }
            }

        }

        m_particles.clear();
        for (GMapping::ParticleVector::iterator it = temp.begin(); it != temp.end(); it++) {
            m_particles.push_back(*it);
        }

        std::cout << "new configuration: ";
        for(unsigned int i = 0; i < m_particles.size();i++){
            std::cout << m_particles[i].previousIndex << " ";
        }
        std::cout << std::endl;

        registerScan(plainReading,true);


        //sync
        remotesamplequeue.send(0,0);
        zmq::message_t dummy;
        remotesamplequeue.recv(&dummy);

    }else{

        //std::cout << "register scan , and sync " << std::endl;
        registerScan(plainReading,false);
        std::string syncstring("resamplesyncremotenoresample");
        zmq::message_t smsg(syncstring.size());

        memcpy(smsg.data(),syncstring.data(),syncstring.size());
        //if done request -> sync
        syncqueue.send(smsg);

    }
}


}
