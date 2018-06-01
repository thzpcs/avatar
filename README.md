### Robotic Avatar Head Tracking

Written by Skyler Morris and Mark Reifsteck for Human Centered Robotics at the Colorado School of Mines


## Software Prerequisites

Python 2.7

Windows 10

NAOqi library

Unity 2017.4.1f1

## Hardware Prerequisites

USB 3.0 port

HDMI port

Webcam

Computer capable of running Windows Mixed Reality

NAO Robot


## Running


* First ensure that all required software is installed and configured including the Windows mixed reality portal first time setup. 
* Plug in all necessary hardware and ensure that the Windows mixed reality headset and webcam are functioning properly. 
* Open the Unity portion of the project in Unity and click the play button at the top to begin the program. 
* Open the Python script in the text editor of your choice. 
* Connect and power on the NAO robot. 
* Once done with its boot sequence, press the chest button on the NAO, and it will list its IP address
* When the NAO says its IP address enter this into the IP into variable at the top of the Python script. 
* Save the script. Navigate to this directory in the console and enter “python headMove.py” to start the Python script. 
* Change the focus of your computer back to the Unity project. The NAO should now be tracking your head movement. 

## Built With

* [Anaconda 3.6.3](https://anaconda.org/anaconda/python)- Python framework used
* [Spyder] - IDE used, comes with Anaconda 3.6.3
* [Windows 10] - Operating system used for development
* [Unity 2017.4.1f1] - Unity game development engine


## Versioning

Git was used for versioning For the versions available, see https://github.com/thzpcs/MixedRealityRobotics

## Authors

* **Skyler Morris** - [thzpcs](https://github.com/thzpcs)
* **Mark Reifsteck** [mwreifsteck](https://github.com/mwreifsteck)

## License

This project is licensed under the MIT License 

## Acknowledgments

* Thank you to Dr. Hao Zhang for assistance and inspiration for this assignment
