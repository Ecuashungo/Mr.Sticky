# Mr.Sticky
Mr.Sticky is a robot that has been designed and built in the framework of the STI interdisciplinary robotics competition at EPFL. The working principle of this bottle collecting robot is a sticky conveyor belt that turns around the whole chassis which is used to grab, store and release the bottles.
The data acquisition and its processing is done on board. The bottle detection uses a classifier trained by a Haar cascade using the OpenCV library. The localization of the robot is calculated through a Kalman filter which uses data from encoders and a self-built triangulation system. The latter detects four LED beacons using a camera with a parabolic lens resulting in a panoramic 360 degree view.
This project allowed us to gain experience in time and budget management as well as practical experience such as mechanical design, PCB manufacturing and software programming. 


![alt text](https://raw.githubusercontent.com/ecuashungo/Mr.Sticky/master/Report/images/mrsticky_in_action.png)
