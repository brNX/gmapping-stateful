#gmapping-stateful

This is a modified version of the GMapping SLAM package ([http://wiki.ros.org/gmapping](http://wiki.ros.org/gmapping)) to run in a distributed environment.

##Installation

This package was developed and tested using ROS Fuerte. 

#### Dependencies: 

 * ZeroMQ ([http://zeromq.org/](http://zeromq.org/)) 
 * Google Protocol Buffers ([https://developers.google.com/protocol-buffers/](https://developers.google.com/protocol-buffers/))
 * All the dependencies from the fuerte branch of GMapping ([http://wiki.ros.org/gmapping](http://wiki.ros.org/gmapping))

To install you can follow the BuildingPackages instructions on [http://wiki.ros.org/ROS/Tutorials/BuildingPackages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) (rosbuild)
  
##Setup

#### Main Computer:
Create a *Workers.txt* text file in the same folder where you are going to launch the GMapping node with the following Syntax:

`tcp://RemoteWorkerIP:Port,percentage_of_attributed_particles`   for each remote worker  
`local` for each local Worker/Thread (notice the missing percentage parameter, the remaining particles get equally distributed to the local workers)

e.g.: for 2 remote workers with IP 192.168.1.1 (30%) and 192.168.1.2 (40%) with the RemoteWorker program  running on port 8010 and 8011 respectively  and 2 local workers the contents of the file would be:
    
    local
    local
    tcp://192.168.1.1:8010,30     
    tcp://192.168.1.2:8011,40

For the GMapping parameters see [http://wiki.ros.org/gmapping](http://wiki.ros.org/gmapping)

#### Remote Workers:

The remote workers need to run the **dp\_worker** from the command line using the following syntax : 

   `$ ./dp_worker  -a tcp://Main_PC_IP -p dp_worker_local_port` 

e.g.:    main PC with IP address 192.168.1.5  using port 50000 locally :  

`$ ./dp_worker  -a  tcp://192.168.1.5 -p  50000` 

