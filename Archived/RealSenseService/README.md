# RealSenseService

This project is designed to allow point to point transmission of RealSense camera data. The data is then republished in ROS topics. This service can be broken down into two parts:  

The Server:  
* Interfaces directly with the camera  
* Sends over data in specified format  

The Client:  
* Subscribes to the server  
* Receives and processes all server data  
* Republishes data in ROS topics  

More specific information about running and compiling each program is located in the [RealSenseClient](./RealSenseClient) and [RealSenseServer](./RealSenseServer) directories. 

## Prerequisites

Each program currently uses [CMake](https://cmake.org/install/) to aid in compilation. However, the server and client each have certain dependencies that are specified in their corresponding directories.

## Contributing

Please read [Contributing.md](./Contributing.md) for details on our code of conduct, and the process for submitting pull requests to us.


