#include "ros/ros.h"
#include "orbslam2_ros/CreateMap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_map_client");
  if (argc != 2)
  {
    ROS_INFO("usage: create_map_client X ");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<orbslam2_ros::CreateMap>("/vslam/create_map");
  orbslam2_ros::CreateMap srv;
  srv.request.enable = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Create map response: %d", (bool)srv.response.status);
  }
  else
  {
    ROS_ERROR("Failed to call service create_map");
    return 1;
  }

  return 0;
}
