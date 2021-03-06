/**
 *  This header file defines the NodeController class. It is highly recommended
 *  whenever an oriented-object programming ROS Node class is created
 *  to enhance this one.
 *
 *  Version: 1.0.0
 *  Created on: 05/10/2016
 *  Modified on: 05/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _NODE_CONTROLLER_H_
#define _NODE_CONTROLLER_H_

#include <ros/ros.h>

class NodeController
{
public:
  virtual ~NodeController(); // destructor
  virtual void spin() const; // standard spin method (according to the given loop rate)

protected:
  NodeController(ros::NodeHandle *nh, float loop_rate); // protected constructor
  ros::NodeHandle* getNodeHandle() const;
  std::string getName() const;
  void shutdown() const;
  
private:
  float loop_rate_; // positive spin rate
  std::string name_; // ROS node name
  ros::NodeHandle *nh_; // protected ros node handle (has-a relationship)
};

#endif // _NODE_CONTROLLER_H_
