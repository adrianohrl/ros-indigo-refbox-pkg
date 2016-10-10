/**
 *  This header file defines the GameStateConverter class, which is
 *  also an EnumConverter of GameState enums.
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _GAME_STATE_CONVERTER_H_
#define _GAME_STATE_CONVERTER_H_

#include "EnumConverter.h"
#include "refbox/states/GameState.h"

namespace refbox
{

namespace states
{

class GameStateConverter : public EnumConverter<GameState>
{
public:
  GameStateConverter(int id);
  GameStateConverter(std::string nome);
  GameStateConverter(GameState enumerated);
  virtual ~GameStateConverter();
  using EnumConverter::getEnumerated;
  virtual GameState getEnumerated(int id) const;
  virtual GameState getEnumerated(std::string nome) const;
  using EnumConverter::getId;
  virtual int getId(std::string nome) const;
  virtual int getId(GameState enumerated) const;
  using EnumConverter::str;
  virtual std::string str(GameState enumerated) const;
  using EnumConverter::c_str;
  virtual const char* c_str(GameState enumerated) const;

  static int toId(GameState enumerated);
  static GameState toEnumerated(int id);
  static GameState toEnumerated(std::string nome);
  static bool isValid(std::string nome);
  static std::string toString(GameState enumerated);
  static const char* toCString(GameState enumerated);
  static GameState getDefault();
  static std::vector<GameState> getAll();
};

}

typedef typename states::GameStateConverter GameStateConverter;

}


#endif // _GAME_STATE_CONVERTER_H_
