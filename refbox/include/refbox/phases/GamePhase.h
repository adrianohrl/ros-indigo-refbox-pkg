/**
 *  This header file defines the GamePhase enums, as described in the
 *  LLSF Referee Box rulebook 2013 (https://trac.fawkesrobotics.org/wiki/LLSFRefBox).
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */
#ifndef _GAME_PHASE_H_
#define _GAME_PHASE_H_

namespace refbox
{

namespace phases
{

/**
 * @brief The GamePhaseEnum enums:
 * PRE_GAME: The game is prepared but not yet started.
 *           Teams are free to perform setup preparations.
 *           When the refbox is started it defaults to this phase.
 *           Setting the state to RUNNING (see below) will automatically
 *           advance to the EXPLORATION phase.
 * EXPLORATION: exploration phase as described in the rulebook.
 *              It lasts for 3 minute after which the refbox automatically
 *              switches to the PRODUCTION phase.
 * PRODUCTION: production phase as described in the rulebook. It lasts for
 *             15 minutes after which the refbox automatically switches to
 *             the POST_GAME phase.
 * POST_GAME: The game has ended.
 */
enum GamePhaseEnum
{
  PRE_GAME,
  EXPLORATION,
  PRODUCTION,
  POST_GAME
};

}

typedef typename phases::GamePhaseEnum GamePhase;

}

#endif // _GAME_PHASE_H_
