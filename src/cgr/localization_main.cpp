//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
* \file    localization_main.cpp
* \brief   Main Vector Localization test program for Cobot
* \author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>

#include "vectorparticlefilter.h"
#include "vector_map.h"
#include "popt_pp.h"
#include "terminal_utils.h"
#include "timer.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "proghelp.h"
#include "cobot_msgs/LidarDisplayMsg.h"
#include "cobot_msgs/CobotRemoteInterfaceSrv.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "configreader.h"
#include "plane_filtering.h"

using namespace std;

bool run = true;
bool usePointCloud = false;
bool noLidar = false;
int numParticles = 10;
int debugLevel = -1;

vector2f initialLoc;
float initialAngle;
float locUncertainty, angleUncertainty;

VectorLocalization2D *localization;

using namespace ros;
using namespace cobot_msgs;
Publisher guiPublisher;
Publisher localizationPublisher;
Publisher filteredPointCloudPublisher;
ServiceServer localizationServer;

LidarDisplayMsg guiMsg;

VectorLocalization2D::PointCloudParams pointCloudParams;
VectorLocalization2D::LidarParams lidarParams;
VectorLocalization2D::MotionModelParams motionParams;

vector<vector2f> pointCloud;
vector<vector2f> pointCloudNormals;

string curMapName;
vector2f curLoc;
float curAngle;
double curTime;
sensor_msgs::LaserScan lastLidarMsg;
sensor_msgs::Image lastDepthMsg;

//Point Cloud parameters
GVector::matrix3d<float> kinectToRobotTransform;
KinectRawDepthCam kinectDepthCam;
PlaneFilter::PlaneFilterParams filterParams;
PlaneFilter planeFilter;

void publishGUI();
void lidarCallback(const sensor_msgs::LaserScan& msg);
void depthCallback(const sensor_msgs::Image& msg);
void publishLocation(bool limitRate=true);

bool localizationCallback(CobotRemoteInterfaceSrv::Request& req, CobotRemoteInterfaceSrv::Response& res)
{
  static const unsigned int CmdSetLocation = 0x0002;
  
  if(debugLevel>0) printf("RemoteCommand num:%d type:0x%02X\n",req.command_num,req.command_type);
  
  if(req.command_type & CmdSetLocation){
    vector2f loc(req.loc_x, req.loc_y);
    if(debugLevel>0) printf("Setting location: %f %f %f\u00b0 on %s\n",V2COMP(loc),DEG(req.orientation),req.map.c_str());
    localization->setLocation(loc, req.orientation,req.map.c_str(),0.5,DEG(5.0));
  }
  
  return true;
}

void ClearGUI()
{
  guiMsg.lines_p1x.clear();
  guiMsg.lines_p1y.clear();
  guiMsg.lines_p2x.clear();
  guiMsg.lines_p2y.clear();
  guiMsg.points_x.clear();
  guiMsg.points_y.clear();
  guiMsg.lines_col.clear();
  guiMsg.points_col.clear();
  guiMsg.circles_x.clear();
  guiMsg.circles_y.clear();
  guiMsg.circles_col.clear();
  
  guiMsg.windowSize = 1.0;
}

void drawPointCloud()
{
  //printf("publishing %d points\n",(int) pointCloud.size());
  
  for(int i=0; i<(int) pointCloud.size(); i++){
    guiMsg.points_x.push_back(pointCloud[i].x);
    guiMsg.points_y.push_back(pointCloud[i].y);
    guiMsg.points_col.push_back(0xDE2352);
  }
}

void publishLocation(bool limitRate)
{
  static double tLast = 0;
  if(GetTimeSec()-tLast<0.03 && limitRate)
    return;
  tLast = GetTimeSec();
  CobotLocalizationMsg msg;
  localization->computeLocation(curLoc, curAngle);
  msg.timeStamp = GetTimeSec();
  msg.x = curLoc.x;
  msg.y = curLoc.y;
  msg.angle = curAngle;
  msg.map = string(localization->getCurrentMapName());
  
  localization->getUncertainty(msg.angleUncertainty, msg.locationUncertainty);
  
  VectorLocalization2D::EvalValues laserEval, pointCloudEval;
  localization->getEvalValues(laserEval,pointCloudEval);
  msg.laserNumCorrespondences = laserEval.numCorrespondences;
  msg.laserNumObservedPoints = laserEval.numObservedPoints;
  msg.laserStage0Weights = laserEval.stage0Weights;
  msg.laserStageRWeights = laserEval.stageRWeights;
  msg.laserRunTime = laserEval.runTime;
  msg.lastLaserRunTime = laserEval.lastRunTime;
  msg.laserMeanSqError = laserEval.meanSqError;
  
  msg.pointCloudNumCorrespondences = pointCloudEval.numCorrespondences;
  msg.pointCloudNumObservedPoints = pointCloudEval.numObservedPoints;
  msg.pointCloudStage0Weights = pointCloudEval.stage0Weights;
  msg.pointCloudStageRWeights = pointCloudEval.stageRWeights;
  msg.pointCloudRunTime = pointCloudEval.runTime;
  msg.lastPointCloudRunTime = pointCloudEval.lastRunTime;
  msg.pointCloudMeanSqError = pointCloudEval.meanSqError;
  
  localizationPublisher.publish(msg);
}

void publishGUI()
{ 
  static double tLast = 0;
  static double publishInterval = 0.016;
  if(debugLevel<0 || GetTimeSec()-tLast<publishInterval)
    return;
  tLast = GetTimeSec();
  ClearGUI();
  //DrawMap();
  guiMsg.robotLocX = curLoc.x;
  guiMsg.robotLocY = curLoc.y;
  guiMsg.robotAngle = curAngle;
  localization->drawDisplay(guiMsg.lines_p1x, guiMsg.lines_p1y, guiMsg.lines_p2x, guiMsg.lines_p2y, guiMsg.lines_col, guiMsg.points_x, guiMsg.points_y, guiMsg.points_col, guiMsg.circles_x, guiMsg.circles_y, guiMsg.circles_col, 1.0);
  //drawPointCloud();
  guiPublisher.publish(guiMsg);
}

void LoadParameters()
{
  WatchFiles watch_files;
  ConfigReader config(ros::package::getPath("cobot_linux").append("/").c_str());
  
  config.init(watch_files);
  
  config.addFile("config/localization_parameters.cfg");
  config.addFile("config/kinect_parameters.cfg");
  
  if(!config.readFiles()){
    printf("Failed to read config\n");
    exit(1);
  }
  
  {
    ConfigReader::SubTree c(config,"KinectParameters");
    
    unsigned int maxDepthVal;
    bool error = false;
    error = error || !c.getReal("f",kinectDepthCam.f);
    error = error || !c.getReal("fovH",kinectDepthCam.fovH);
    error = error || !c.getReal("fovV",kinectDepthCam.fovV);
    error = error || !c.getInt("width",kinectDepthCam.width);
    error = error || !c.getInt("height",kinectDepthCam.height);
    error = error || !c.getUInt("maxDepthVal",maxDepthVal);
    kinectDepthCam.maxDepthVal = maxDepthVal;    
    
    vector3f kinectLoc;
    float xRot, yRot, zRot;
    error = error || !c.getVec3f("loc",kinectLoc);
    error = error || !c.getReal("xRot",xRot);
    error = error || !c.getReal("yRot",yRot);
    error = error || !c.getReal("zRot",zRot);
    kinectToRobotTransform.xyzRotationAndTransformation(xRot,yRot,zRot,kinectLoc);
    
    if(error){
      printf("Error Loading Kinect Parameters!\n");
      exit(2);
    }
  } 
  
  {
    ConfigReader::SubTree c(config,"PlaneFilteringParameters");
    
    bool error = false;
    error = error || !c.getUInt("maxPoints",filterParams.maxPoints);
    error = error || !c.getUInt("numSamples",filterParams.numSamples);
    error = error || !c.getUInt("numLocalSamples",filterParams.numLocalSamples);
    error = error || !c.getUInt("planeSize",filterParams.planeSize);
    error = error || !c.getReal("maxError",filterParams.maxError);
    error = error || !c.getReal("maxDepthDiff",filterParams.maxDepthDiff);
    error = error || !c.getReal("minInlierFraction",filterParams.minInlierFraction);
    error = error || !c.getReal("WorldPlaneSize",filterParams.WorldPlaneSize);
    error = error || !c.getUInt("numRetries",filterParams.numRetries);
    filterParams.runPolygonization = false;
    
    if(error){
      printf("Error Loading Plane Filtering Parameters!\n");
      exit(2);
    }
  } 
  
  {
    ConfigReader::SubTree c(config,"initialConditions");
    
    bool error = false;
    curMapName = string(c.getStr("mapName"));
    error = error || curMapName.length()==0;
    error = error || !c.getVec2f("loc",initialLoc);
    error = error || !c.getReal("angle", initialAngle);
    error = error || !c.getReal("locUncertainty", locUncertainty);
    error = error || !c.getReal("angleUncertainty", angleUncertainty);
    
    if(error){
      printf("Error Loading Initial Conditions!\n");
      exit(2);
    }
  }
  
  {
    ConfigReader::SubTree c(config,"motionParams");
    
    bool error = false;
    error = error || !c.getReal("Alpha1", motionParams.Alpha1);
    error = error || !c.getReal("Alpha2", motionParams.Alpha2);
    error = error || !c.getReal("Alpha3", motionParams.Alpha3);
    error = error || !c.getReal("kernelSize", motionParams.kernelSize);
    
    
    if(error){
      printf("Error Loading Predict Parameters!\n");
      exit(2);
    }
  }
  
  {
    ConfigReader::SubTree c(config,"lidarParams");
    
    bool error = false;
    // Laser sensor properties
    error = error || !c.getReal("angleResolution", lidarParams.angleResolution);
    error = error || !c.getInt("numRays", lidarParams.numRays);
    error = error || !c.getReal("maxRange", lidarParams.maxRange);
    error = error || !c.getReal("minRange", lidarParams.minRange);
    
    // Pose of laser sensor on robot
    error = error || !c.getVec2f<vector2f>("laserLoc", lidarParams.laserLoc);
    
    // Parameters related to observation update
    error = error || !c.getReal("logObstacleProb", lidarParams.logObstacleProb);
    error = error || !c.getReal("logOutOfRangeProb", lidarParams.logOutOfRangeProb);
    error = error || !c.getReal("logShortHitProb", lidarParams.logShortHitProb);
    error = error || !c.getReal("correlationFactor", lidarParams.correlationFactor);
    error = error || !c.getReal("lidarStdDev", lidarParams.lidarStdDev);
    error = error || !c.getReal("attractorRange", lidarParams.attractorRange);
    error = error || !c.getReal("kernelSize", lidarParams.kernelSize);
    
    // Parameters related to observation refine
    error = error || !c.getInt("minPoints", lidarParams.minPoints);
    error = error || !c.getInt("numSteps", lidarParams.numSteps);
    error = error || !c.getReal("etaAngle", lidarParams.etaAngle);
    error = error || !c.getReal("etaLoc", lidarParams.etaLoc);
    error = error || !c.getReal("maxAngleGradient", lidarParams.maxAngleGradient);
    error = error || !c.getReal("maxLocGradient", lidarParams.maxLocGradient);
    error = error || !c.getReal("minCosAngleError", lidarParams.minCosAngleError);
    error = error || !c.getReal("correspondenceMargin", lidarParams.correspondenceMargin);
    error = error || !c.getReal("minRefineFraction", lidarParams.minRefineFraction);
    
    lidarParams.initialize();
    
    if(error){
      printf("Error Loading Lidar Parameters!\n");
      exit(2);
    }
  }
  
  {
    ConfigReader::SubTree c(config,"pointCloudParams");
    
    bool error = false;
    error = error || !c.getReal("logObstacleProb", pointCloudParams.logObstacleProb);
    error = error || !c.getReal("logShortHitProb", pointCloudParams.logShortHitProb);
    error = error || !c.getReal("logOutOfRangeProb", pointCloudParams.logOutOfRangeProb);
    error = error || !c.getReal("kinectCorrelationFactor", pointCloudParams.kinectCorrelationFactor);
    error = error || !c.getReal("kinectStdDev", pointCloudParams.kinectStdDev);
    error = error || !c.getReal("attractorRange", pointCloudParams.attractorRange);
    error = error || !c.getReal("maxRange", pointCloudParams.maxRange);
    error = error || !c.getReal("minRange", pointCloudParams.minRange);
    
    error = error || !c.getReal("attractorRange", pointCloudParams.attractorRange);
    error = error || !c.getReal("etaAngle", pointCloudParams.etaAngle);
    error = error || !c.getReal("etaLoc", pointCloudParams.etaLoc);
    error = error || !c.getReal("maxAngleGradient", pointCloudParams.maxAngleGradient);
    error = error || !c.getReal("maxLocGradient", pointCloudParams.maxLocGradient);
    error = error || !c.getReal("minCosAngleError", pointCloudParams.minCosAngleError);
    error = error || !c.getInt("numSteps", pointCloudParams.numSteps);
    error = error || !c.getInt("minPoints", pointCloudParams.minPoints);
    error = error || !c.getReal("minRange", pointCloudParams.minRange);
    error = error || !c.getReal("maxRange", pointCloudParams.maxRange);
    error = error || !c.getReal("correspondenceMargin", pointCloudParams.correspondenceMargin);
    error = error || !c.getReal("minRefineFraction", pointCloudParams.minRefineFraction);
    
    if(error){
      printf("Error Loading Point Cloud Parameters!\n");
      exit(2);
    }
  }
  
  planeFilter.setParameters(&kinectDepthCam,filterParams);
}

void InitModels(){
  lidarParams.laserScan = (float*) malloc(lidarParams.numRays*sizeof(float));
  return;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  static float angle = 0;
  static vector2f loc(0,0);
  static bool initialized=false;
  if(!initialized){
    angle = tf::getYaw(msg->pose.pose.orientation);
    loc = vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
    initialized = true;
    return;
  }
  
  float newAngle = tf::getYaw(msg->pose.pose.orientation);
  vector2f newLoc(msg->pose.pose.position.x, msg->pose.pose.position.y);
  
  vector2f d = (newLoc-loc).rotate(-angle);
  double dx = d.x;
  double dy = d.y;
  double dtheta = newAngle-angle;
  
  if(debugLevel>0) printf("Odometry x:%7.3f y:%7.3f a:%7.3f\u00b0\n",dx, dy, DEG(dtheta));
  
  localization->predict(dx, dy, dtheta, motionParams);
  
  angle = newAngle;
  loc = newLoc;
}

void lidarCallback(const sensor_msgs::LaserScan &msg)
{
  //FunctionTimer ft(__PRETTY_FUNCTION__);
  if(debugLevel>0){
    printf("LIDAR n:%d t:%f noLidar:%d\n",(int) msg.ranges.size(), msg.scan_time, noLidar?1:0);
  }
  
  
  if(int(msg.ranges.size()) != lidarParams.numRays){
    TerminalWarning("Incorrect number of Laser Scan rays!");
    printf("received: %d\n",int(msg.ranges.size()));
  }
  for(int i=0; i<(int)msg.ranges.size(); i++)
    lidarParams.laserScan[i] = msg.ranges[i];
  
  if(!noLidar){
    localization->refineLidar(lidarParams);
    localization->updateLidar(lidarParams, motionParams);
    localization->resample(VectorLocalization2D::LowVarianceResampling);
    localization->computeLocation(curLoc,curAngle);
  }
}


void depthCallback(const sensor_msgs::Image &msg)
{
  //Copy the depth data
  if(debugLevel>0){
    printf("Depth Message t:%f\n", msg.header.stamp.toSec());
  }
  const uint8_t* ptrSrc = msg.data.data();
  uint16_t depth[640*480];
  memcpy(depth, ptrSrc, 640*480*(sizeof(uint16_t)));
  
  if(!usePointCloud)
    return;
  //Generate filtered point cloud  
  vector<vector3f> filteredPointCloud;
  vector<vector3f> pointCloudNormals;
  vector<vector3f> outlierCloud;
  vector<vector2i> pixelLocs;
  vector<PlanePolygon> planePolygons;
  
  planeFilter.GenerateFilteredPointCloud(depth, filteredPointCloud, pixelLocs, pointCloudNormals, outlierCloud, planePolygons);
  
  //Transform from kinect coordinates to robot coordinates
  for(unsigned int i=0; i<filteredPointCloud.size(); i++){
    filteredPointCloud[i] = filteredPointCloud[i].transform(kinectToRobotTransform);
    pointCloudNormals[i] = pointCloudNormals[i].transform(kinectToRobotTransform);
  }
  
  //Call particle filter
  {
    vector<vector2f> pointCloud2D, pointCloudNormals2D;
    
    for(unsigned int i=0; i<filteredPointCloud.size(); i++){
      if(fabs(pointCloudNormals[i].z)>sin(RAD(30.0)))
        continue;
      
      vector2f curNormal(V2COMP(pointCloudNormals[i]));
      vector2f curPoint(V2COMP(filteredPointCloud[i]));
      curNormal.normalize();
      pointCloudNormals2D.push_back(curNormal);
      pointCloud2D.push_back(curPoint);
    }
    
    localization->refinePointCloud(pointCloud2D, pointCloudNormals2D, pointCloudParams);
    localization->updatePointCloud(pointCloud2D, pointCloudNormals2D, motionParams);
    localization->resample(VectorLocalization2D::LowVarianceResampling);
    localization->computeLocation(curLoc,curAngle);
  }
}

int main(int argc, char** argv)
{ 
  //========================= Load Parameters from config file ======================
  LoadParameters();
  
  //========================== Set up Command line parameters =======================
  static struct poptOption options[] = {
    { "num-particles",    'n', POPT_ARG_INT,     &numParticles,       0, "Number of Particles",   "NUM"},
    { "debug",            'd', POPT_ARG_INT,     &debugLevel,         0, "Debug Level",           "NUM"},
    { "start-x",          'x', POPT_ARG_DOUBLE,  &initialLoc.x,       0, "Starting X location",   "NUM"},
    { "start-y",          'y', POPT_ARG_DOUBLE,  &initialLoc.y,       0, "Starting Y location",   "NUM"},
    { "start-angle",      'a', POPT_ARG_DOUBLE,  &initialAngle,       0, "Starting Angle",        "NUM"},
    { "use-point-cloud",  'p', POPT_ARG_NONE,    &usePointCloud,      0, "Use Point Cloud",       "NONE"},
    { "no-lidar",         'l', POPT_ARG_NONE,    &noLidar,            0, "No LIDAR Observations", "NONE"},
    { "Alpha1-param",     'u', POPT_ARG_DOUBLE,  &motionParams.Alpha1,  0, "Alpha1 parameter",   "NUM"},
    { "Alpha2-param",     'v', POPT_ARG_DOUBLE,  &motionParams.Alpha2,  0, "Alpha2 parameter",   "NUM"},
    { "Alpha3-param",     'w', POPT_ARG_DOUBLE,  &motionParams.Alpha3,  0, "Alpha3 parameter",   "NUM"},
    
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  
  //========================= Welcome screen, Load map & log ========================
  if(debugLevel>=0)
    __attribute__ (unused) int sysReturn = system("clear\n");
  
  ColourTerminal(TerminalUtils::TERMINAL_COL_WHITE,TerminalUtils::TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nVector Localization\n\n");
  ResetTerminal();
  
  if(debugLevel>=0){
    printf("NumParticles     : %d\n",numParticles);
    printf("Alpha1           : %f\n",motionParams.Alpha1);
    printf("Alpha2           : %f\n",motionParams.Alpha2);
    printf("Alpha3           : %f\n",motionParams.Alpha3);
    printf("UsePointCloud    : %d\n",usePointCloud?1:0);
    printf("UseLIDAR         : %d\n",noLidar?0:1);
    printf("Visualizations   : %d\n",debugLevel>=0?1:0);
    printf("\n");
  }
  double seed = floor(fmod(GetTimeSec()*1000000.0,1000000.0));
  if(debugLevel>-1) printf("Seeding with %d\n",(unsigned int)seed);
  srand(seed);
  
  //Initialize particle filter, sensor model, motion model, refine model
  string mapsFolder = ros::package::getPath("cobot_linux").append("/../maps");
  localization = new VectorLocalization2D(mapsFolder.c_str());
  InitModels();
  
  //Initialize particle filter
  localization->initialize(numParticles,curMapName.c_str(),initialLoc,initialAngle,locUncertainty,angleUncertainty);
  
  //Initialize ros for sensor and odometry topics
  InitHandleStop(&run);
  ros::init(argc, argv, "CGR_Localization");
  ros::NodeHandle n;
  guiPublisher = n.advertise<LidarDisplayMsg>("localization_gui",1,true);
  localizationPublisher = n.advertise<CobotLocalizationMsg>("localization",1,true);
  Sleep(0.1);
  
  localizationServer = n.advertiseService("localization_interface", &localizationCallback);
  
  //Initialize ros for sensor and odometry topics
  ros::Subscriber odometrySubscriber = n.subscribe("odom", 20, odometryCallback);
  ros::Subscriber lidarSubscriber = n.subscribe("base_laser", 1, lidarCallback);
  
  while(ros::ok() && run){
    ros::spinOnce();
    publishGUI();
    publishLocation();
    Sleep(0.005);
  }
  
  if(debugLevel>=0) printf("closing.\n");
  return 0;
}
