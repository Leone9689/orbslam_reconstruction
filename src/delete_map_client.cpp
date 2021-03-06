#include "ros/ros.h"
#include "orbslam2_ros/DeleteMap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "delete_map_client");
  if (argc != 2)
  {
    ROS_INFO("usage: delete_map_client X ");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<orbslam2_ros::DeleteMap>("/vslam/delete_map");
  orbslam2_ros::DeleteMap srv;
  srv.request.name = argv[1];
  if (client.call(srv))
  {
    ROS_INFO("Delete map response: %d", srv.response.status);
  }
  else
  {
    ROS_ERROR("Failed to call service vslam_delete_map");
    return 1;
  }

  return 0;
}
