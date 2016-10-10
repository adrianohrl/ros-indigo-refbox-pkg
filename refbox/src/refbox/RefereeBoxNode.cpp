/**
 *  This source file implements the RefereeBoxNode controller class, which is also a
 *  NodeController class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 09/10/2016
 *  Modified on: 09/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "refbox/RefereeBoxNode.h"

namespace refbox
{

RefereeBoxNode::RefereeBoxNode(ros::NodeHandle *nh)
  : NodeController(nh, 30)
{}

RefereeBoxNode::~RefereeBoxNode()
{}

}
