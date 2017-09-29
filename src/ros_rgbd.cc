/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <thread>
//#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h> 

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <nav_msgs/Odometry.h>

#include <qapplication.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "System.h"
#include "onlinefusionviewer.hpp"
#include "qtros.hpp"
#include "KeyFrame.h"
#include "Converter.h"
#include "Map.h"
#include "MapPublisher.hpp"
//#include "OdomPublisher.hpp"
#include "orbslam2_ros/CreateMap.h"
#include "orbslam2_ros/DeleteMap.h"
#include "orbslam2_ros/LoadMap.h"
#include "orbslam2_ros/SaveMap.h"
#include "orbslam2_ros/Relocation.h"

#define BOXDELTA 0.01
//#define VOLUMERADIUS 1.5
#define VOLUMERADIUS 1.4
#define USE_ORIGINAL_VOLUME 1

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::CameraInfo> sync_pol;

class ImageGrabber
{
public:
ImageGrabber(ORB_SLAM2::System* pSLAM)
            :cloudCnt(0),keyFrameCnt(0),relocate(false),
             distance(0),mapDensity(0),keyFrameDensity(0),S(0),createMap(false),mapName("none"),mpSLAM(pSLAM),
             rgb_sub(n, "/vslam/rgb/image_raw", 1),
             depth_sub(n, "/vslam/depth_registered/image_raw", 1),
             info_sub(n, "/vslam/depth/camera_info", 1),
             sync(sync_pol(10), rgb_sub,depth_sub,info_sub)
    {
      rgb_pub = n.advertise<sensor_msgs::Image>("/vslam/rgb",1);
      depth_pub = n.advertise<sensor_msgs::Image>("/vslam/depth",1);
      odom_pub = n.advertise<nav_msgs::Odometry>("/vslam/odom", 10); 
      camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/vslam/camera_info",1);
      map_info_pub = n.advertise<std_msgs::Float32MultiArray>("/vslam/map_info", 10);

      if(!relocate && !createMap) 
      {
        rgb_sub.unsubscribe();
        depth_sub.unsubscribe();
        info_sub.unsubscribe();
      }
      sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,this,_1,_2,_3));
      
    }
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::CameraInfoConstPtr& msgInfo);
    bool CreateMapStatus(orbslam2_ros::CreateMap::Request &req,orbslam2_ros::CreateMap::Response &res);            
    bool DeleteMapStatus(orbslam2_ros::DeleteMap::Request &req,orbslam2_ros::DeleteMap::Response &res);            
    bool SaveMapStatus(orbslam2_ros::SaveMap::Request &req,orbslam2_ros::SaveMap::Response &res);
    bool RelocationStatus(orbslam2_ros::Relocation::Request &req,orbslam2_ros::Relocation::Response &res);                           
    bool LoadMapStatus(orbslam2_ros::SaveMap::Request &req,orbslam2_ros::SaveMap::Response &res); 
    void square(vector<ORB_SLAM2::MapPoint*> mapPoints,int n);
    void quarter(vector<ORB_SLAM2::MapPoint*> mapPoints,float x_min,float x_max,float y_min,float y_max,int n);
    float GetMapSquare(vector<ORB_SLAM2::MapPoint*> mapPoints);
public:
    
    int cloudCnt;                              
    int keyFrameCnt;                              
    bool relocate;
    double distance;
    double mapDensity;
    double keyFrameDensity;
    float S;
    bool createMap;
    string mapName;
    std::map<int,int> cloudSize;             
    std::map<int,cv::Mat> translation;             
    ros::NodeHandle n;                       
    cv::Mat cloud;    
    ORB_SLAM2::System* mpSLAM;               
    ORB_SLAM2::Frame* currFrame;             
    
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    message_filters::Synchronizer<sync_pol> sync;
    
    ros::Publisher depth_pub;
    ros::Publisher rgb_pub;
    ros::Publisher odom_pub; 

    ros::Publisher camera_info_pub;
    ros::Publisher map_info_pub;
    ORB_SLAM2::MapPublisher _map_pub;
    
};

bool ImageGrabber::CreateMapStatus(orbslam2_ros::CreateMap::Request  &req,  
                                   orbslam2_ros::CreateMap::Response &res)            
{                                                                          
  res.status = req.enable;                                                   
  ROS_INFO("CreateMap response:%d", (bool)res.status);               
  createMap = res.status;                               
  mpSLAM->Initialize(createMap,mapName);
  
  if(createMap) {
    rgb_sub.subscribe();
    depth_sub.subscribe();
    info_sub.subscribe();
  }else{
    rgb_sub.unsubscribe();
    depth_sub.unsubscribe();
    info_sub.unsubscribe();
  }  
  return true;                                                             
}   
bool ImageGrabber::DeleteMapStatus(orbslam2_ros::DeleteMap::Request &req, 
                                   orbslam2_ros::DeleteMap::Response &res)              
{                                                                                
  res.status = true;                                                             
  ROS_INFO("DeleteMap response:%s", req.name.c_str());                           
  mapName = req.name;                                                            
  mpSLAM->DeleteMap(mapName+".map");                                             
  return true;                                                                   
}                                                                                


bool ImageGrabber::LoadMapStatus(orbslam2_ros::SaveMap::Request  &req, 
                                 orbslam2_ros::SaveMap::Response &res) 
{                                                            
  res.status = true;                                           
  ROS_INFO("LoadMap response:%s", req.name.c_str());    
  mapName = req.name;
  mpSLAM->Initialize(createMap,mapName+".map");
  return true;                                               
}                           
bool ImageGrabber::SaveMapStatus(orbslam2_ros::SaveMap::Request  &req, 
                                 orbslam2_ros::SaveMap::Response &res) 
{                                                            
  res.status = true;                                           
  ROS_INFO("SaveMap response:%s", req.name.c_str());    
  mpSLAM->SaveMap(req.name+".map");                                

  return true;                                               
}                                                            
bool ImageGrabber::RelocationStatus(orbslam2_ros::Relocation::Request  &req,  
                                    orbslam2_ros::Relocation::Response &res)            
{                                                                          
   res.status = req.enable;                                                   
   //ROS_INFO("request: bool=%d", (bool)req.flag);                            
   ROS_INFO("Relocation response:%d", (bool)res.status);               
   relocate = res.status;                               
   if(relocate) {
     rgb_sub.subscribe();
     depth_sub.subscribe();
     info_sub.subscribe();
   }else{
     rgb_sub.unsubscribe();
     depth_sub.unsubscribe();
     info_sub.unsubscribe();
   }  
   return true;                                                             
}                                                                          

void ImageGrabber::square(vector<ORB_SLAM2::MapPoint*> mapPoints,int n)
{
    float x_min=9999,x_max=-9999;
    float y_min=9999,y_max=-9999;
    for(auto mp: mapPoints)    
    {
      cv::Mat wp = mp->GetWorldPos();  
      if(x_min > wp.at<float>(2))   
        x_min = wp.at<float>(2);
      if(x_max < wp.at<float>(2))   
        x_max = wp.at<float>(2);

      if(y_min > wp.at<float>(0))   
        y_min = wp.at<float>(0);
      if(y_max < wp.at<float>(0))   
        y_max = wp.at<float>(0);
    }
    
    if(n>1)
      quarter(mapPoints,x_min,x_max,y_min,y_max,n/4); 
    else
      S = S + (x_max-x_min)*(y_max-y_min);
}
void ImageGrabber::quarter(vector<ORB_SLAM2::MapPoint*> mapPoints,float x_min,float x_max,float y_min,float y_max,int n)
{
    float x_mid=0,y_mid=0;
    
    float x0_min=9999,x0_max=-9999,y0_min=9999,y0_max=-9999;
    float x1_min=9999,x1_max=-9999,y1_min=9999,y1_max=-9999;
    float x2_min=9999,x2_max=-9999,y2_min=9999,y2_max=-9999;
    float x3_min=9999,x3_max=-9999,y3_min=9999,y3_max=-9999;
    
    x_mid = (x_max + x_min)/2;
    y_mid = (y_max + y_min)/2;
    for(auto mp: mapPoints)    
    {
      cv::Mat wp = mp->GetWorldPos();  
      if(x_min<wp.at<float>(2) && wp.at<float>(2)<x_mid && y_mid<wp.at<float>(0) && wp.at<float>(0)<y_max )//0
      {
        if(x0_min > wp.at<float>(2)) 
          x0_min = wp.at<float>(2);  
        if(x0_max < wp.at<float>(2)) 
          x0_max = wp.at<float>(2);  
        if(y0_min > wp.at<float>(0)) 
          y0_min = wp.at<float>(0);  
        if(y0_max < wp.at<float>(0)) 
          y0_max = wp.at<float>(0); 
      }
      if(x_mid<wp.at<float>(2) && wp.at<float>(2)<x_max && y_mid<wp.at<float>(0) && wp.at<float>(0)<y_max )//1
      {
        if(x1_min > wp.at<float>(2)) 
          x1_min = wp.at<float>(2);  
        if(x1_max < wp.at<float>(2)) 
          x1_max = wp.at<float>(2);  
        if(y1_min > wp.at<float>(0)) 
          y1_min = wp.at<float>(0);  
        if(y1_max < wp.at<float>(0)) 
          y1_max = wp.at<float>(0); 
      }
      if(x_min<wp.at<float>(2) && wp.at<float>(2)<x_mid && y_min<wp.at<float>(0) && wp.at<float>(0)<y_mid )//2
      {
        if(x2_min > wp.at<float>(2)) 
          x2_min = wp.at<float>(2);  
        if(x2_max < wp.at<float>(2)) 
          x2_max = wp.at<float>(2);  
        if(y2_min > wp.at<float>(0)) 
          y2_min = wp.at<float>(0);  
        if(y2_max < wp.at<float>(0)) 
          y2_max = wp.at<float>(0); 
      }
      if(x_mid<wp.at<float>(2) && wp.at<float>(2)<x_max && y_min<wp.at<float>(0) && wp.at<float>(0)<y_mid )//3
      {
        if(x3_min > wp.at<float>(2)) 
          x3_min = wp.at<float>(2);  
        if(x3_max < wp.at<float>(2)) 
          x3_max = wp.at<float>(2);  
        if(y3_min > wp.at<float>(0)) 
          y3_min = wp.at<float>(0);  
        if(y3_max < wp.at<float>(0)) 
          y3_max = wp.at<float>(0); 
      }
    }   
    if(x0_min==9999||x0_max==-9999||y0_min==9999||y0_max==-9999)
      x0_min = x0_max = y0_min = y0_max = 0;
    if(x1_min==9999||x1_max==-9999||y1_min==9999||y1_max==-9999)
      x1_min = x1_max = y1_min = y1_max = 0;
    if(x2_min==9999||x2_max==-9999||y2_min==9999||y2_max==-9999)
      x2_min = x2_max = y2_min = y2_max = 0;
    if(x3_min==9999||x3_max==-9999||y3_min==9999||y3_max==-9999)
      x3_min = x3_max = y3_min = y3_max = 0;
    
    if(n>1)
    {
      quarter(mapPoints,x_min,x_mid,y_mid,y_max,n/4); 
      quarter(mapPoints,x_mid,x_max,y_mid,y_max,n/4); 
      quarter(mapPoints,x_min,x_mid,y_min,y_mid,n/4); 
      quarter(mapPoints,x_mid,x_max,y_min,y_mid,n/4); 
    }
    else
      S = S+ (x0_max-x0_min)*(y0_max-y0_min)+(x1_max-x1_min)*(y1_max-y1_min)+(x2_max-x2_min)*(y2_max-y2_min)+(x3_max-x3_min)*(y3_max-y3_min); 
}
int main(int argc, char **argv)
{
  bool noViewer = true;
  bool bufferImages = false;
  bool useColor = true;
  bool volumeColor = true;

  bool useLoopClosures = false;

  unsigned int startimage = 0;
  unsigned int endimage = 1000000;
  unsigned int imageStep = 1;
  //float maxCamDistance = MAXCAMDISTANCE;
  float maxCamDistance = 3.0;
  //float scale = DEFAULT_SCALE;
  float scale = 0.005;
  float threshold = DEFAULT_SCALE;
  //float imageDepthScale = DEPTHSCALE;
  float imageDepthScale = 1000;
  
  bool threadMeshing = false;
  bool threadFusion = false;
  bool threadImageReading = false;
  bool performIncrementalMeshing = true;
  int depthConstistencyChecks = 0;
  
  if(threadMeshing) fprintf(stderr,"\nMeshing will run in a separate Thread");
  else              fprintf(stderr,"\nMeshing will NOT run in a separate Thread");
  if(threadFusion)  fprintf(stderr,"\nFusion will run in a separate Thread");
  if(threadImageReading) fprintf(stderr,"\nImage Reading will run in a separate Thread");
 
  fprintf(stderr,"\nCreating Mipmapping CPU Octree");                                  
  FusionMipMapCPU *fusion = new FusionMipMapCPU(0,0,0,scale,threshold,0,volumeColor);  
                                                                                       
  fusion->setThreadMeshing(threadMeshing);                                             
  fusion->setDepthChecks(depthConstistencyChecks);                                     
  fusion->setIncrementalMeshing(performIncrementalMeshing);                            

    QtROS qtRos(argc, argv, "RGBD"); 
    QApplication application(argc,argv); 
    bool use_onlinefusion_viewer = true;                                 
                                                                
    ros::param::get("~use_pango_viewer",use_onlinefusion_viewer);         
    cerr << "## ORB ROS parameters:" << endl;                      
    cerr << "# orb use pango viewer " << use_onlinefusion_viewer << endl; 
    /*std::cout<<"ARGV[4]:"<<argv[4]<<",ARGV[5]:"<<argv[5]<<std::endl;
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    } */

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::OnlineFusionViewerManipulated *viewerpointer  = new ORB_SLAM2::OnlineFusionViewerManipulated(false);
    ORB_SLAM2::OnlineFusionViewerManipulated &viewer = *viewerpointer;

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,&viewer);
    ImageGrabber igb(&SLAM);
    
    ros::NodeHandle nh;

    ros::ServiceServer create_map_service = nh.advertiseService("/vslam/create_map", &ImageGrabber::CreateMapStatus,&igb);
    ros::ServiceServer delete_map_service = nh.advertiseService("/vslam/delete_map", &ImageGrabber::DeleteMapStatus,&igb);
    ros::ServiceServer load_map_service = nh.advertiseService("/vslam/load_map", &ImageGrabber::LoadMapStatus,&igb);          
    ros::ServiceServer save_map_service = nh.advertiseService("/vslam/save_map", &ImageGrabber::SaveMapStatus,&igb);          
    ros::ServiceServer relocate_service = nh.advertiseService("/vslam/relocation", &ImageGrabber::RelocationStatus,&igb);
	 
    fprintf(stderr,"\nSetting Viewer Parameters");    
    viewer._fusion = fusion;                          
    viewer.setWindowTitle("Fusion");           
    viewer._threadFusion = threadFusion;              
    viewer._threadImageReading = threadImageReading;  
    viewer.show();                                    
    viewer._imageDepthScale = imageDepthScale;        
    viewer._maxCamDistance = maxCamDistance;          
    viewer._firstFrame = (long int)startimage;        
    viewer._currentFrame = (long int)startimage-1;    

    QObject::connect(&application, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));    
    QObject::connect(&qtRos, SIGNAL(rosQuits()), &application, SLOT(quit()));          
                                                                                          
    qtRos.start();// Run main loop.                                                    

    application.exec(); 
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveMap("Map.map");
    //ros::shutdown();
    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::CameraInfoConstPtr& msgInfo)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cv::imwrite("/home/iopenlink/depth.jpg",cv_ptrD->image);
    currFrame = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    int mapPoints = mpSLAM->GetMap()->GetAllMapPoints().size();
    cloud = cv::Mat::zeros(mapPoints, 1, CV_32FC3 ); 
    
    int temp=0;
 
    for(auto mp: mpSLAM->GetMap()->GetAllMapPoints())    
    {
      cv::Mat wp = mp->GetWorldPos();  
      
      cloud.at<cv::Vec3f>(temp,0)[0] = wp.at<float>(0); 
      cloud.at<cv::Vec3f>(temp,0)[1] = wp.at<float>(1); 
      cloud.at<cv::Vec3f>(temp,0)[2] = wp.at<float>(2); 
      temp++;
    
    }

    cloudSize[cloudCnt] = temp;                                   
    if(cloudCnt==0) 
      _map_pub.PublishMapPoints(cloud);                                        
    else if((cloudSize[cloudCnt]-cloudSize[cloudCnt-1])!=0 && cloudCnt>=1)   
    {                                                                          
      _map_pub.PublishMapPoints(cloud);                                        
    }                                                                          
    cloudCnt++;                                       
    

    if(!currFrame->mTcw.empty()) 
    {
      
      if(!currFrame->mpRelocalizing)                   
      {  
        sensor_msgs::Image imageD = *msgD;                                                                           
        imageD.header.stamp = msgD->header.stamp;
        imageD.header.frame_id = "/camera_depth_frame";
        depth_pub.publish(imageD); 

        sensor_msgs::Image imageRGB = *msgRGB;                                                                           
        imageRGB.header.stamp = msgRGB->header.stamp;
        imageRGB.header.frame_id = "/camera_rgb_frame";
        rgb_pub.publish(imageRGB); 

        nav_msgs::Odometry odom;  
        odom.header.stamp = msgRGB->header.stamp; 
        odom.header.frame_id = "/odom";
        //cv::Mat Twc = (currFrame->mTcw).inv();                          
        cv::Mat Twc = currFrame->mTcw;                          
        cv::Mat tcwMat = Twc.rowRange(0,3).col(3);        

        Eigen::Matrix<double,3,3> eigMat =ORB_SLAM2::Converter::toMatrix3d(Twc); 
        Eigen::Quaterniond q(eigMat);                     

        odom.pose.pose.position.x = tcwMat.at<float>(0);  
        odom.pose.pose.position.y = tcwMat.at<float>(1);  
        odom.pose.pose.position.z = tcwMat.at<float>(2);  
        odom.pose.pose.orientation.x = q.x();  
        odom.pose.pose.orientation.y = q.y();  
        odom.pose.pose.orientation.z = q.z();  
        odom.pose.pose.orientation.w = q.w();  
        odom_pub.publish(odom);  

        sensor_msgs::CameraInfo depthInfo = *msgInfo;
        _map_pub.PublishCurrentCamera(currFrame->mTcw); 
        camera_info_pub.publish(depthInfo);
                                                                                                                     
        //imageD.header.stamp = msgInfo->header.stamp;                                                                 
        _map_pub.PublishCurrentCamera(currFrame->mTcw);                                                              
        camera_info_pub.publish(msgInfo);                                                                            
                                                                                                                     
       /* static tf::TransformBroadcaster laser_broadcaster;
        tf::Transform laser_transform;                    
        laser_transform.setOrigin( tf::Vector3(tcwMat.at<float>(2), -tcwMat.at<float>(0),-tcwMat.at<float>(1)) );    
        Eigen::Quaterniond qMat(eigMat);                     
        tf::Quaternion q(qMat.z(),-qMat.x(),-qMat.y(),qMat.w());        
        //q.setRPY(-ea[2],ea[0],-ea[1]);                                                                               
        laser_transform.setRotation(q);                                                                              
        laser_broadcaster.sendTransform(tf::StampedTransform(laser_transform, imageD.header.stamp, "map", "odom"));  
                                                                                                                    
        static tf::TransformBroadcaster odom_base_broadcaster;
        tf::Transform odom_base_transform;                    
        odom_base_transform.setIdentity(); 
        odom_base_broadcaster.sendTransform(tf::StampedTransform(odom_base_transform, imageD.header.stamp, "odom", "base_link"));  
        
        static tf::TransformBroadcaster base_camera_broadcaster;
        tf::Transform base_camera_transform;                    
        base_camera_transform.setIdentity(); 
        base_camera_broadcaster.sendTransform(tf::StampedTransform(base_camera_transform, imageD.header.stamp, "base_link", "camera_link"));  
        
        static tf::TransformBroadcaster camera_depth_broadcaster;
        tf::Transform camera_depth_transform;                    
        camera_depth_transform.setIdentity(); 
        camera_depth_broadcaster.sendTransform(tf::StampedTransform(camera_depth_transform, imageD.header.stamp, "camera_link", "camera_depth_frame"));  
        */
      } 
    
      if(currFrame->keyFrame)
      {
        const unsigned int data_sz = 4;
        std_msgs::Float32MultiArray m;

        m.layout.dim.push_back(std_msgs::MultiArrayDimension());
        m.layout.dim[0].size = data_sz;
        m.layout.dim[0].stride = 1;
        m.layout.dim[0].label = "map_info";
        
        square(mpSLAM->GetMap()->GetAllMapPoints(),64);
        translation[keyFrameCnt] = currFrame->mTcw; 
        if(keyFrameCnt >0)
        {
          cv::Mat Tcr = translation[keyFrameCnt] * translation[keyFrameCnt-1].inv();
          cv::Mat tcwMat = Tcr.rowRange(0,3).col(3); 
          distance += fabs(cv::norm(tcwMat));
          mapDensity = mapPoints/S;
        }
        keyFrameCnt++;
        keyFrameDensity=keyFrameCnt/S;
        
        m.data.resize(data_sz);
        m.data[0] = mapDensity;
        m.data[1] = keyFrameDensity;
        m.data[2] = S;
        m.data[3] = float(!currFrame->mpRelocalizing);//tracking 1   relocation 0

        map_info_pub.publish(m);
        S =0 ;
      } 
    }

} 
