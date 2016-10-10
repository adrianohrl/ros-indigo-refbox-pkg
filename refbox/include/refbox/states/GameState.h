/**
 *  This header file defines the GameState enums, as described in the
 *  LLSF Referee Box rulebook 2013 (https://trac.fawkesrobotics.org/wiki/LLSFRefBox).
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */
#ifndef _GAME_STATE_H_
#define _GAME_STATE_H_

namespace refbox
{

namespace states
{

/**
 * @brief The GameStateEnum enums:
 * INIT: The refbox is currently being initialized. If you set this state the refbox will be completely reset as if it were just started. It automatically advances to WAIT_START once initialization has completed. 
 * WAIT_START: The refbox has finished initializing and is awaiting the game start. 
 * RUNNING: The game is active. The robots must perform their tasks according to the rules and the game time is advancing. 
 * PAUSED: The game has been interrupted. All robots must stop moving immediately. In the production phase machine processing will be suspended and in the exploration phase all signals will be turned off. 
 */
enum GameStateEnum
{
  INIT,
  WAIT_START,
  RUNNING,
  PAUSED
};

}

typedef typename states::GameStateEnum GameState;

}

#endif // _GAME_STATE_H_
