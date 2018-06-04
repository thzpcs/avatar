#### Robotic Avatar Head Tracking and movement

Written by Skyler Morris


## Software Prerequisites

Python 2.7

Windows 10

NAOqi library

Unity 2017.4.1f1

Kinect SDK v1.8

## Hardware Prerequisites

USB 3.0 port

HDMI port

Webcam

Microsoft Kinect v1.0 (Xbox 360 version)

Computer capable of running Windows Mixed Reality 
(See: https://docs.microsoft.com/en-us/windows/mixed-reality/enthusiast-guide/windows-mixed-reality-minimum-pc-hardware-compatibility-guidelines 
for the Microsoft recommended specifications)

NAO v4 or v5 (The software should run on a v3.3, but issues were encountered and it is not recommended)


## Running

* First ensure that all required software is installed and configured including the Windows mixed reality portal first time setup. 
* Plug in all necessary hardware and ensure that the Windows mixed reality headset and webcam are functioning properly.
* Open the Python script in the text editor of your choice. 
* Connect and power on the NAO robot. 
* Run “avatar.py” to start the Python script. 
* Once done with its boot sequence, press the chest button on the NAO, and it will list its IP address
* When the NAO says its IP address enter this when requested in the Python script. A Unity window will be launched, press "OK" without changing any settings
* Change the focus of your computer back to the Unity window. The NAO should now be tracking your head movement. 

## Built With

* [Anaconda 2.7](https://anaconda.org/anaconda/python)- Python framework used
* [Spyder] - IDE used, comes with Anaconda 2.7
* [Windows 10] - Operating system used for development
* [Unity 2017.4.1f1] - Unity game development engine


## Versioning

Git was used for versioning For the versions available, see https://github.com/thzpcs/avatar

## Authors

* **Skyler Morris** - [thzpcs](https://github.com/thzpcs)

## License

This project is licensed under the MIT License 

## Acknowledgments

* Thank you to Dr. Xiaoli Zhang for assistance and inspiration for this project