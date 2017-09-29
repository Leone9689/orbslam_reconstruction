#include "qtros.hpp"

QtROS::QtROS(int argc, char *argv[], const char* node_name) {
  std::cout << "Initializing Node...\n";
  ros::init(argc, argv, node_name);
  ros::start();
  //n = new ros::NodeHandle(node_name); //Use node name as Ros Namespace
  ROS_INFO("Connected to roscore");
  quitfromgui = false; 
}

void QtROS::quitNow(){ 
  quitfromgui = true; 
}

void QtROS::run(){

  ros::Rate r(30); // 30 hz. Kinect has 30hz and we are far from processing every frame anyhow.
  while(ros::ok() && !quitfromgui) {
    ros::spinOnce(); 
    r.sleep();}
  if (!quitfromgui) {
    Q_EMIT rosQuits();
    ROS_INFO("ROS-Node Terminated\n"); 
    ros::shutdown();//Not sure if necessary
  }
  ros::Duration d(0.5);
  d.sleep();
  exit(0);//Try Quick Exit*/
}
