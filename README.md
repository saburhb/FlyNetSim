# FlyNetSim (Integrated UAV-Network Simulator)
FlyNetSim is a UAV Network Simulator created by combining ns-3 network simulator and Ardupilot based UAV simulator Software-in-the-loop (SITL) with light weight Pub/Sub based middleware. The details about the design of the simulator and some preliminary results for some use case scenarios are presented in our [paper](https://arxiv.org/pdf/1808.04967.pdf). If you find this code useful in your research, please consider citing the paper: [Baidya, S., Shaikh, Z., Levorato, M. "FlyNetSim: An Open Source Synchronized UAV Network Simulator based on ns-3 and Ardupilot". In Proceedings of the 21st ACM International Conference on Modeling, Analysis and Simulation of Wireless and Mobile Systems (ACM MSWiM).](https://dl.acm.org/doi/pdf/10.1145/3242102.3242118)

The bibtex is mentioned below:
```
  @inproceedings{baidya2018flynetsim,
      title={FlyNetSim: An Open Source Synchronized UAV Network Simulator based on ns-3 and Ardupilot},
      author={Baidya, Sabur and Shaikh, Zoheb and Levorato, Marco},
      booktitle={Proceedings of the 21st ACM International Conference on Modeling, Analysis and Simulation of Wireless and Mobile Systems},
      pages={37--45},
      year={2018},
      organization={ACM}
  }
```
This code was tested on an Ubuntu 16.04 system. For network simulator, ns-3.27 is used; so the dependent packages need to be installed as mentioned in ns-3 wiki page. For ubuntu/debian platform, the specific packages can be found at: https://www.nsnam.org/wiki/Installation#Ubuntu.2FDebian.2FMint . The basic packages needed are : Python2.7, pip, python-dev, gcc.

### Installation

1. Additional Dependencies for FlyNetSim:   
Install latest version of czmq, libzmq, libczmq and libxml
```
  sudo apt-get install libzmq5 libzmq-dev libczmq4 libczmq-dev czmq libxml2 libxml2-dev
```
If you get error for versions, install the latest available version.

2. Clone the Git repository:
```
  git clone https://github.com/saburhb/FlyNetSim.git
```
3. Go to the NetSim folder and run the initial script which downloads ns-3.27, applies patches, configures and builds. This script needs to be executed only once and it may take a while to finish.
```
  $ cd FlyNetSim/NetSim
  $ ./net_init.sh
```
4. Go to the FlySim floder and run the initial script for downloading and configuring Ardupilot, dronekit and sitl. This script also needs to be executed only once and it may take a longer time to finish.
```
  $ cd FlyNetSim/FlySim
  $ ./fly_init.sh
```
5. Install PyQt4 for GUI.

### Run Simulation

After successful installation, execute the simulation with the following command. It creates the end-to-end communication, a GUI as ground-control station to send commands and receive telemetry. The Ardupilot also creates indiviual console window for each UAV that shows the flight status. To run the simulator with default parameters: 
```
  $ cd FlyNetSim/FlySim
  $ python FlyNetSim.py
```
Specific parameters can be passed for different attributes in the simulation, e.g., number of UAVs, initial position of the UAV(s), network type (WiFi or LTE), external traffic load (number of contending nodes, data rates, packet size). To check the options that can be passed, use the help command:
```
$ python FlyNetSim.py --help
```
Once the program runs, it opens one terminal for each UAV and one terminal for the ns-3 and one GUI panel with several buttons as GCS for the UAV commands. In the GUI panel, you need to follow the orders:\
i) Click only once the "CONNECT" and wait until "ARM" button is active.\
ii) Click only once the "ARM" button to arm the vehicle (with default parameters) and wait until "TAKEOFF" button becomes active.\
iii) Click only once the "TAKEOFF" button and wait until other control buttons (UP/DOWN etc.) become active. Notice that the takeoff command takes the UAV to 10m height.\
iv) click other control buttons for movements (UP/DOWN, FORWARD/BACKWARD, LEFT/RIGHT) as many times you need.\
v) Click LAND button for landing.\

If you want automated mission, you can avoid the GUI and write your own mission in FlySim/uav_pubsub.py or in FlySim/gcs_pubsub.py file.

### EDIT:
If pressing "CONNECT" button does not activate the "ARM", the ns-3 installation/compilation needs to be verified. To confirm, if the problem is indeed from ns-3 and not from socket/ZMQ, you can run the script "FlyNetSim_direct_no_ns3.py" that directly connects the GCS and UAV over ZMQ sockets and bypasses the ns-3. Run it with the following command:
```
  $ cd FlyNetSim/
  $ python FlyNetSim_direct_no_ns3.py -v 3
```

#### Multi-UAV Scenario
For Multiple UAVs as of now, a simple positioning system with linear layout is used. It takes x-coordinate of the first UAV and places other UAVs 5 meters apart. If different layout is required, the code needs to be modified. Also, for multi-UAV scenario a single GCS gives commands to all the UAVs. The code can be extended for individual control of the vehicles.


### Fix for Possible Errors

1. While building ns-3, if you get an error for "syslog.h" in  "include/czmq_prelude.h", comment out the line in "include/czmq_prelude.h" from the include path.
```
  //#include <syslog.h>
```

2. The ZMQ is using some ports which sometimes might not be free or there are sockets open on the ports. If a port is already busy, use different port if the used one can't be freed. The port numbers used in the simulator are hardcoded as the following configuration:
 
 ![alt text](https://github.com/saburhb/FlyNetSim/blob/master/flynetsim_ports.jpg)
          
 3. If there is error for the path of "config.xml" file provide absolute path with FlyNetSim.py.
 
 4. If the ardupilot/Tools/environment_install folder does not have "install-prereqs-ubuntu.sh" , recursively update the submodules of arduplilot repository. Run the command from inside the ardupilot directory:
 ```
 $ git submodule update -init --recursive
 ```
