/**
 *  This header file defines the PuckState enums, as described in the
 *  LLSF Referee Box rulebook 2013 (https://trac.fawkesrobotics.org/wiki/LLSFRefBox).
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */
#ifndef _PUCK_STATE_H_
#define _PUCK_STATE_H_

namespace refbox
{

namespace states
{

/**
 * @brief The PuckStateEnum enums:
 * S0: 
 * S1: 
 * S2: 
 * P1: 
 * P2: 
 * P3: 
 * CONSUMED: 
 */
enum PuckStateEnum
{
  S0,
  S1,
  S2,
  P1,
  P2,
  P3,
  CONSUMED
};

}

typedef typename states::PuckStateEnum PuckState;

}

#endif // _PUCK_STATE_H_
