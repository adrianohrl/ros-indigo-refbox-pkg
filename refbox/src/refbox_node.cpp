#include <stdlib.h>
#include "refbox/RefereeBoxNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "receptor");
  // Instanciating a new NodeController object.
  refbox::RefereeBoxNode node(new ros::NodeHandle("~"));
  // NodeController will spin while this ROS Node is OK.
  node.spin();
  return EXIT_SUCCESS;
}
