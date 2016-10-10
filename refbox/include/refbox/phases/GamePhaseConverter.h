/**
 *  This header file defines the GamePhaseConverter class, which is
 *  also an EnumConverter of GamePhase enums.
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _GAME_PHASE_CONVERTER_H_
#define _GAME_PHASE_CONVERTER_H_

#include "EnumConverter.h"
#include "refbox/phases/GamePhase.h"

namespace refbox
{

namespace phases
{

class GamePhaseConverter : public EnumConverter<GamePhase>
{
public:
  GamePhaseConverter(int id);
  GamePhaseConverter(std::string nome);
  GamePhaseConverter(GamePhase enumerated);
  virtual ~GamePhaseConverter();
  using EnumConverter::getEnumerated;
  virtual GamePhase getEnumerated(int id) const;
  virtual GamePhase getEnumerated(std::string nome) const;
  using EnumConverter::getId;
  virtual int getId(std::string nome) const;
  virtual int getId(GamePhase enumerated) const;
  using EnumConverter::str;
  virtual std::string str(GamePhase enumerated) const;
  using EnumConverter::c_str;
  virtual const char* c_str(GamePhase enumerated) const;

  static int toId(GamePhase enumerated);
  static GamePhase toEnumerated(int id);
  static GamePhase toEnumerated(std::string nome);
  static bool isValid(std::string nome);
  static std::string toString(GamePhase enumerated);
  static const char* toCString(GamePhase enumerated);
  static GamePhase getDefault();
  static std::vector<GamePhase> getAll();
};

}

typedef typename phases::GamePhaseConverter GamePhaseConverter;

}


#endif // _GAME_PHASE_CONVERTER_H_
