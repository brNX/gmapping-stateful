#include <boost/program_options.hpp>
#include <string>
#include <gridfastslam/gridslamprocessor.h>
#include <scanmatcher/scanmatcher.h>
#include "protobuf/protobufhelper.h"
#include <zmq.hpp>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/gzip_stream.h>
#include <boost/algorithm/string.hpp>
#include <boost/detail/endian.hpp>
#include "remotegridslamprocessor.h"


#define PARTICLESCRATCHSPACE (1024*1024*2) //2MB
#define CHUNKSIZE (1024*512) //512k
#define COMPRESSEDESCRATCHSPACE (1024*512) //512k
#define COMPRESSEDCHUNKSIZE (1024*64) //64k


#ifdef BOOST_BIG_ENDIAN
#define LITTLE_ENDIAN32(i) { i = swap32(i); }
#else
#define LITTLE_ENDIAN32(i) {}
#endif

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)



#define REMOTERESPONSE_PORT ":20001"
#define PUBSUB_PORT ":20000"
#define REQREP_PORT ":20010"
#define SYNCQUEUE_PORT ":20003"

#define DADOS

using namespace std;


namespace
{
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

} // namespace


void parsecmdline(int argc, char *argv[],std::string & address, std::string & port){

    /** Define and parse the program options
         */
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
            ("help", "Print help messages")
            ("address,a",po::value<std::string>()->required() ,"Server Address")
            ("port,p",po::value<std::string>()->required(),"Listen port");


    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc),
                  vm); // can throw

        /** --help option*/
        if ( vm.count("help")  )
        {
            std::cout << "SLAM Gemp.str()Mapping Worker App" << std::endl
                      << desc << std::endl;
            exit(SUCCESS);
        }

        po::notify(vm); // throws on error, so do after help in case
        // there are any problems
    }
    catch(po::error& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        exit(ERROR_IN_COMMAND_LINE);
    }

    address = vm["address"].as<std::string>();
    port = vm["port"].as<std::string>();
}


void printDetails(std::string port, std::string address)
{
    std::cout << "listening to: " << address << std::endl;
    std::cout << "broadcastqueue : " << address+ PUBSUB_PORT << std::endl;
    std::cout << "remoteworkerresponsequeue : " << address+ REMOTERESPONSE_PORT << std::endl;
    std::cout << "remotesamplequeue : " << address+ REQREP_PORT << std::endl;
    std::cout << "syncqueue : " << address+ SYNCQUEUE_PORT << std::endl;
    std::cout << "workerqueue : " << "tcp://*:" + port << std::endl;
}

vector<int> getIndexes(zmq::socket_t & workerqueue)
{
    zmq::message_t indexes;
    workerqueue.recv(&indexes);


    //std::cout << "indexes size " << indexes.size() << std::endl;

    gmapping_structs::indexMessage indexmessage;
    indexmessage.ParseFromArray(indexes.data(),indexes.size());

    //std::cout << "index message size of array: " << indexmessage.resampleindexes_size() << std::endl;

    vector<int> vindexes;
    for (int i = 0; i < indexmessage.resampleindexes_size();i++){
        vindexes.push_back(indexmessage.resampleindexes(i));
    }


    //printf("index size %d\n",indexmessage.resampleindexes_size());

    return vindexes;
}

void parseSettings(ProtoBuf::ProtobufHelper & pbufhelper, GMapping::RemoteGridSlamProcessor &rgslamprocessor, gmapping_structs::StartPackage &startpackage, vector<int> &vindexes)
{
    pbufhelper.deserializeScanMatcherSettings(startpackage.settings(),rgslamprocessor.m_matcher);

    /*motion model
    rgslamprocessor.m_motionModel.srr=startpackage.srr();
    rgslamprocessor.m_motionModel.str=startpackage.str();
    rgslamprocessor.m_motionModel.srt=startpackage.srt();
    rgslamprocessor.m_motionModel.stt=startpackage.stt();*/

    //remote gridslam processor
    GMapping::OrientedPoint initialPose;
    initialPose.x=startpackage.startpose().x();
    initialPose.y=startpackage.startpose().y();
    initialPose.theta=startpackage.startpose().theta();
    rgslamprocessor.init(vindexes.size(),startpackage.xmin(),startpackage.ymin(),startpackage.xmax(),startpackage.ymax(),startpackage.delta(),initialPose,vindexes);
}

int main (int argc, char *argv[])
{

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1,time(NULL));

    GMapping::RemoteGridSlamProcessor rgslamprocessor;
    ProtoBuf::ProtobufHelper pbufhelper;

    int laserbeams=0;


    try{

        //unsigned long workcounter = 0;

        std::string address;
        std::string port;
        parsecmdline(argc,argv,address,port);

        zmq::context_t context(1);

        //startup sockets
        zmq::socket_t remoteworkerresponsequeue(context, ZMQ_PUSH);
        zmq::socket_t broadcastqueue(context, ZMQ_SUB);
        zmq::socket_t remotesamplequeue(context, ZMQ_REQ);
        zmq::socket_t syncqueue(context, ZMQ_PUSH);
        zmq::socket_t workerqueue(context, ZMQ_PULL);

        printDetails(port, address);

        std::string broadcastqueueport = address+ PUBSUB_PORT;
        std::string remoteworkerresponseport = address+ REMOTERESPONSE_PORT;
        std::string remotesampleport = address+ REQREP_PORT;
        std::string syncport = address+ SYNCQUEUE_PORT;
        std::string workerport = "tcp://*:"+port ;

        broadcastqueue.connect(broadcastqueueport.c_str());
        broadcastqueue.setsockopt(ZMQ_SUBSCRIBE, "", 0);
        remoteworkerresponsequeue.connect(remoteworkerresponseport.c_str());
        remotesamplequeue.connect(remotesampleport.c_str());
        syncqueue.connect(syncport.c_str());

        workerqueue.bind(workerport.c_str());

        //receive indexes
        vector<int> vindexes = getIndexes(workerqueue);

        unsigned int min = vindexes[0];
        unsigned int max = vindexes[vindexes.size()-1];

        std::cout << "min index: " << min << " max index: "<< max << std::endl;

        zmq::message_t startpackagemessage;
        zmq::message_t id;
        broadcastqueue.recv(&id);
        broadcastqueue.recv(&startpackagemessage);

        std::cout << "received startpackage" << std::endl;
        gmapping_structs::StartPackage startpackage;
        startpackage.ParseFromArray(startpackagemessage.data(),startpackagemessage.size());

        //parse gslamprocessorSettings
        parseSettings(pbufhelper, rgslamprocessor, startpackage, vindexes);

        std::cout << "parsed settings" << std::endl;

        laserbeams = startpackage.plainreading_size();
        std::cout << laserbeams <<  " laserbeams" << std::endl;
        double plainReading [laserbeams];
        for(int i = 0 ; i < laserbeams; i++){
            plainReading[i]=startpackage.plainreading(i);
        }

        //register 1st scan
        for(unsigned int i = 0 ; i < rgslamprocessor.m_particles.size(); i++){
            //register 1st scan
            rgslamprocessor.m_matcher.invalidateActiveArea();
            rgslamprocessor.m_matcher.computeActiveArea(rgslamprocessor.m_particles[i].map,rgslamprocessor.m_particles[i].pose,plainReading);
            rgslamprocessor.m_matcher.registerScan(rgslamprocessor.m_particles[i].map, rgslamprocessor.m_particles[i].pose, plainReading);

        }

        std::cout << "first scan registered" << std::endl;

        //workcounter++;

        //sync workers
        std::string syncstring("sync1rstmessage");
        zmq::message_t smsg(syncstring.size());
        memcpy(smsg.data(),syncstring.data(),syncstring.size());
        syncqueue.send(smsg);

        int indexi=0;

        //main loop
        while(1){
#ifdef DADOS
            std::cout << std::endl;
            std::cout << indexi++ << ";" ;
#endif

            //std::cout << "main loop , waiting for scan" << std::endl;

            //wait for data on broadcastqueue
            zmq::message_t broadcastmessage;
            zmq::message_t id;
            broadcastqueue.recv(&id); //envelope id
            broadcastqueue.recv(&broadcastmessage);

/*#ifdef DADOS
            std::cout << broadcastmessage.size() << ";" ;
#endif*/

            //std::string idstring (static_cast<char*>(id.data()),id.size());
            //std::cout << "received data id:" << idstring << std::endl;

            gmapping_structs::Sensordata sensordata;
            sensordata.ParseFromArray(broadcastmessage.data(),broadcastmessage.size());


            //deserialize laserdata
            for(int i = 0 ; i< laserbeams;i++){
                plainReading[i]=sensordata.plainreading(i);
            }

            #pragma omp parallel for
            for(unsigned int i = 0 ; i < rgslamprocessor.m_particles.size(); i++){
                unsigned int currentindex = vindexes[i]; //get corresponding global index

                //deserialize odom and weights for corresponding particles
                rgslamprocessor.m_particles[i].weight=sensordata.pose_weights(currentindex).weight();
                rgslamprocessor.m_particles[i].weightSum=sensordata.pose_weights(currentindex).weightsum();
                rgslamprocessor.m_particles[i].pose.x= sensordata.pose_weights(currentindex).pose().x();
                rgslamprocessor.m_particles[i].pose.y= sensordata.pose_weights(currentindex).pose().y();
                rgslamprocessor.m_particles[i].pose.theta= sensordata.pose_weights(currentindex).pose().theta();

                //scanmatch
                rgslamprocessor.processScan(plainReading,remoteworkerresponsequeue,i);
            }

            //std::cout << "waiting for resample message" << std::endl;
            //wait for resample message
            broadcastqueue.recv(&id); //envelope id
            broadcastqueue.recv(&broadcastmessage);
/*#ifdef DADOS
            std::cout << broadcastmessage.size() << ";" ;
#endif*/

            //std::cout << "received resamplemessage" << std::endl;

            rgslamprocessor.resample(broadcastmessage,remotesamplequeue,syncqueue,min,max,plainReading);


            //sync workers
            //syncqueue.send(0,0);
            //std::cout << "sync" << std::endl;


        }

#ifdef DADOS
        std::cout << std::endl ;
#endif

    }
    catch(std::exception& e)
    {
        std::cerr << "Unhandled Exception reached the top of main: "
                  << e.what() << ", application will now exit" << std::endl;
        return ERROR_UNHANDLED_EXCEPTION;
    }


    return SUCCESS;

}

