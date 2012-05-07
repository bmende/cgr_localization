/** @mainpage CGR (Laser Scanner, Kinect - FSPF) Localization
*
* @author Joydeep Biswas (joydeepb -AT- ri DOT cmu DOT edu)
*
* @section intro Overview
* This package provides the code for CGR localization to localize a robot using either 
* laser rangefinder readings or depth images obtained from Kinect-style sensors.
*
* @section depend Dependencies
* To install all dependencies on Ubuntu, run "./InstallPackages", or copy & run the following command:
* @verbatim sudo apt-get install g++ libqt4-dev cmake libpopt-dev libusb-1.0-0-dev liblua5.1-dev libglew1.5-dev libeigen3-dev @endverbatim
*
* @section compile Compiling
* Run "make"
*
* @section test Testing with pre-recorded data
*
* @section robot Running on your own robot
* 
*
* If using the code in this package as an example - please modify the comments
* as appropriate for your own specific code.
*
* @section cite Publications
* -# "Corrective Gradient Reﬁnement for Mobile Robot Localization", Joydeep Biswas, Brian Coltin, and Manuela Veloso, 
*     Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems, September, 2011, pp. 73 - 78. 
*     <a HREF="http://joydeepb.com/Publications/iros2011_cgr.pdf">PDF</a>
* -# "Depth Camera Based Indoor Mobile Robot Localization and Navigation", Joydeep Biswas and Manuela Veloso, 
*     Proceedings of IEEE International Conference on Robotics and Automation, May, 2012. 
*     <a HREF="http://joydeepb.com/Publications/icra2012_kinectLocalization.pdf">PDF</a>
*/