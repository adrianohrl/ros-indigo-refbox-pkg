/**
 *  This source file implements the NodeController class.
 *
 *  Version: 1.0.0
 *  Created on: 05/10/2016
 *  Modified on: 05/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "NodeController.h"

/**
 * @brief NodeController::NodeController builds an NodeControllerController object given a ROS NodeHandle
 * and also its desired spin rate.
 * @param nh must NOT be NULL.
 * @param loop_rate must be positive.
 */
NodeController::NodeController(ros::NodeHandle *nh, float loop_rate)
  : loop_rate_(loop_rate)
{
  if (!nh)
  {
    ROS_FATAL("ROS node handle must not be NULL!!!");
    ros::shutdown();
    return;
  }
  if (loop_rate <= 0)
  {
    ROS_FATAL("The node spin rate must be positive!!!");
    ros::shutdown();
    return;
  }
  nh_ = nh;
  name_ = ros::this_node::getName();
}

/**
 * @brief NodeController::~NodeController announces that this ROS node will shutdown and
 * destructs the ROS NodeHandle object properly.
 */
NodeController::~NodeController()
{
  if (nh_)
  {
    delete nh_;
  }
}

/**
 * @brief NodeController::spin loops while there is not another instance
 * of this node with this node name, or while the Ctrl+C buttons
 * is not pressed at the terminal. In addition, it periodicly updates
 * this node, as well as, controls the updates rate.
 */
void NodeController::spin() const
{
  ros::Rate loop_rate(loop_rate_);
  ROS_INFO("%s is ON!!!", name_.c_str());
  while (nh_->ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/**
 * @brief NodeController::shutdown
 */
void NodeController::shutdown() const
{
  ROS_WARN("%s is OFF now!!!", name_.c_str());
  nh_->shutdown();
}

/**
 * @brief NodeController::getNodeHandle encapsulates this ROS node handle.
 * @return a pointer to an internal member that handles this node.
 */
ros::NodeHandle* NodeController::getNodeHandle() const
{
  return nh_;
}

/**
 * @brief NodeController::getName encapsulates this ROS node name.
 * @return this ROS node whole name.
 */
std::string NodeController::getName() const
{
  return name_;
}
