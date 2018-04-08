# RealSenseService Client

The RealSenseService Client focuses on receiving data from the server and publishing the received data in an intelligible ROS topic. Currently, clients must subscribe directly to the server.

## Prerequisites

Currently, only an Ubuntu client is supported. This is because the ROS interface being used in this build (ROS Indigo) is not supported by Windows.   

## Installing

First clone this repository and make sure that [CMake](https://cmake.org/install/) is installed  
Then install the following by using their tutorials:
* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)  
* [KrisLibrary](https://github.com/krishauser/KrisLibrary)  

In order to allow for compressed image data to be received by the client, you also need to make sure that libjpeg, libpng, and zlib are installed on your system. This should be accomplished by using the following commands in a standard terminal program. 

```
sudo apt-get install libjpeg-dev  
sudo apt-get install libpng-dev  
sudo apt-get install zlib1g-dev  
sudo apt-get update
```

Note that the program will be able to compile without installing libpng, libjpeg, and zlib. You just will not be able to receive compressed data.  
Afterwards, navigate to the file RealSenseService/RealSenseClient/CMakeModules/KrisLibraryDependencies.cmake. Open it up and change the value of KRISLIBRARY_ROOT to the absolute path that specifies where the KrisLibrary folder was installed.  

```
...
  SET(KRISLIBRARY_ROOT "your_path_here"
     CACHE PATH
     "Path for finding KrisLibrary external dependencies"
     FORCE)
...
```


In addition, open the Makefile located in RealSenseService/RealSenseClient folder. There, change the values of CMAKE_SOURCE_DIR and CMAKE_BIN_DIR to the full path to the RealSenseService/RealSenseClient folder.

```
...
CMAKE_SOURCE_DIR = your_other_path_here

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = your_other_path_here
...
```

Finally, from the command line, navigate to the RealSenseService/RealSenseClient folder and run the following commands:
```
cmake .
make
```

Congratulations. The RealSenseClient should be installed and compiled.

### Execution

In order to run the program, first make sure you have a ros_master running. Without a master node to subscribe to, the program will throw an error. After that is set up, run the executable from the command line:

```
./RealSense_ROS_Emitter IP_ADDRESS TOPIC_NAME 
./RealSense_ROS_Emitter IP_ADDRESS TOPIC_NAME -d
```

IP_ADDRESS refers to the ip address of the server you are trying to connect to. TOPIC_NAME can be any name. When connecting two or more cameras, you will need to have each camera to have its own TOPIC_NAME and IP_ADDRESS. The final parameter, -d, allows the user to save some information output by the program. When active, data is logged to a text file that can be consulted later. Depending on what type of data is being sent, different files will be created.  

*Note that the parameters must be input in the order shown above*

## Contributing

Please read [Contributing.md](../Contributing.md) for details on our code of conduct, and the process for submitting pull requests to us.

