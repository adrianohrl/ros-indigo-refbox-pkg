/**
 *  This header file defines the RefereeBoxNode controller class, which is also a
 *  NodeController class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 09/10/2016
 *  Modified on: 09/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _REFEREE_BOX_NODE_H_
#define _REFEREE_BOX_NODE_H_

#include "NodeController.h"
#include "refbox/phases/GamePhaseConverter.h"
#include "refbox/states/GameStateConverter.h"

namespace refbox
{

class RefereeBoxNode : public NodeController
{
public:
  RefereeBoxNode(ros::NodeHandle *nh);
  virtual ~RefereeBoxNode();

private:
  GamePhase phase_;
  GameState state_;

};

}

#endif // _REFEREE_BOX_NODE_H_
