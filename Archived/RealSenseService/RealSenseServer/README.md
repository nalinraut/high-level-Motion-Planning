# RealSenseService Server

The RealSenseService Server focuses on connecting to the RealSense Camera, reformatting data from the camera, and sending that data across the network. Currently, each camera requires one server, and the format in which data is transmitted is determined by the user's specifications for the server. Clients cannot request data in a certain format.  

## Prerequisites

Currently, only a Windows server has been tested. This is because the RealSense SDK used in building this program was previously only supported in Windows. 

## Installing

### RealSense SDK

The server should be built using the newest version of the RealSense SDK. Download the Depth Camera Manager and SDK Essentials from [here](https://software.intel.com/en-us/intel-realsense-sdk/download). However, [here](https://software.intel.com/en-us/articles/previous-intel-realsense-install) is a link to older versions of the SDK if needed.

### RealSense Server

Currently, nothing else needs to be installed to use the server. Just make sure that you have cloned this repository.

## Execution

For ease of use, right click on the executable located in the RealSenseServer/bin/Release directory and create a shortcut on the desktop. Then, right click on the shortcut and click "Properties". From there, locate the "Target" property. It should display something like "absolute_path_to_executable/RealSenseService.exe". The IP address of the computer running the server needs to be the second parameter. For example:

```
Target: "absolute_path_to_executable/RealSenseService.exe" tcp://XXX.XXX.XXX.XXX:3457
```

Note that, we are specifically using port 3457. Other ports may work but have not been tested.  

The following parameters can be entered in any order after the IP address is defined:  
* -nocompress -> Disables compression
* -nodepth    -> Disables depth messages 
* -nocolor    -> Disables color messages
* -nopc       -> Disables point cloud messages
* -raw				-> Disables depth compression only (redundant when -nocompress is used)
* -d					-> Debug - outputs debug information into a file labeled "results.txt"  
* -v					-> Shows copies of images being streamed 


## Compilation

More detailed instructions are to follow in the future. For now, note that this project was compiled using Microsoft Visual Studio in conjunction with CMake. The project requires the following dependencies:  
* [KrisLibrary](https://github.com/krishauser/KrisLibrary)  
* [SSPP](https://github.com/iml-internal/SSPP)  
* RealSense (See above)  
* [Libjpeg](https://sourceforge.net/projects/libjpeg-turbo/)  (Grabbed from libjpeg-turbo)
* [Libpng](http://gnuwin32.sourceforge.net/packages/libpng.htm)  
* [Zlib](http://gnuwin32.sourceforge.net/packages/zlib.htm)  

We have included some Make files in this repository that may be useful, but if you want to compile this project yourself, you will need to compile KrisLibrary and may need to compile Libpng/Zlib. 

Importantly, for MSVC version 12.0 and later, the native thread library is used in compilation. For MSVC version 11.0 and earlier, you can compile this program, but you will need to add Boost thread libraries to the project. 

## Contributing

Please read [Contributing.md](../Contributing.md) for details on our code of conduct, and the process for submitting pull requests to us.

