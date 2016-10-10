/**
 *  This source file implements the GamePhaseConverter class.
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "refbox/phases/GamePhaseConverter.h"

namespace refbox
{

namespace phases
{

GamePhaseConverter::GamePhaseConverter(int id)
  : EnumConverter(getEnumerated(id))
{}

GamePhaseConverter::GamePhaseConverter(std::string nome)
  : EnumConverter(getEnumerated(nome))
{}

GamePhaseConverter::GamePhaseConverter(GamePhase enumerated)
  : EnumConverter(enumerated)
{}

GamePhaseConverter::~GamePhaseConverter()
{}

GamePhase GamePhaseConverter::getEnumerated(int id) const
{
  switch (id)
  {
  case 0:
    return PRE_GAME;
  case 1:
    return EXPLORATION;
  case 2:
    return PRODUCTION;
  case 3:
    return POST_GAME;
  }
  return GamePhaseConverter::getDefault();
}

GamePhase GamePhaseConverter::getEnumerated(std::string nome) const
{
  if (nome == "PRE_GAME")
  {
    return PRE_GAME;
  }
  else if (nome == "EXPLORATION")
  {
    return EXPLORATION;
  }
  else if (nome == "PRODUCTION")
  {
    return PRODUCTION;
  }
  else if (nome == "POST_GAME")
  {
    return POST_GAME;
  }
  return getDefault();
}

int GamePhaseConverter::getId(std::string nome) const
{
  if (nome == "PRE_GAME")
  {
    return 0;
  }
  else if (nome == "EXPLORATION")
  {
    return 1;
  }
  else if (nome == "PRODUCTION")
  {
    return 2;
  }
  else if (nome == "POST_GAME")
  {
    return 3;
  }
  return -1;
}

int GamePhaseConverter::getId(GamePhase enumerated) const
{
  switch (enumerated)
  {
  case PRE_GAME:
    return 0;
  case EXPLORATION:
    return 1;
  case PRODUCTION:
    return 2;
  case POST_GAME:
    return 3;
  }
  return -1;
}

std::string GamePhaseConverter::str(GamePhase enumerated) const
{
  switch (enumerated)
  {
  case PRE_GAME:
    return "PRE_GAME";
  case EXPLORATION:
    return "EXPLORATION";
  case PRODUCTION:
    return "PRODUCTION";
  case POST_GAME:
    return "POST_GAME";
  }
  return "";
}

const char* GamePhaseConverter::c_str(GamePhase enumerated) const
{
  return str(enumerated).c_str();
}

int GamePhaseConverter::toId(GamePhase enumerated)
{
  GamePhaseConverter converter(enumerated);
  return converter.getId();
}

GamePhase GamePhaseConverter::toEnumerated(int id)
{
  GamePhaseConverter converter(id);
  return converter.getEnumerated();
}

GamePhase GamePhaseConverter::toEnumerated(std::string nome)
{
  GamePhaseConverter converter(nome);
  return converter.getEnumerated();
}

bool GamePhaseConverter::isValid(std::string nome)
{
  return nome == "PRE_GAME" || nome == "EXPLORATION" || nome == "PRODUCTION" || nome == "POST_GAME";
}

std::string GamePhaseConverter::toString(GamePhase enumerated)
{
  GamePhaseConverter converter(enumerated);
  return converter.str();
}

const char* GamePhaseConverter::toCString(GamePhase enumerated)
{
  GamePhaseConverter converter(enumerated);
  return converter.c_str();
}

GamePhase GamePhaseConverter::getDefault()
{
  return PRE_GAME;
}

std::vector<GamePhase> GamePhaseConverter::getAll()
{
  std::vector<GamePhase> enumerateds;
  enumerateds.push_back(PRE_GAME);
  enumerateds.push_back(EXPLORATION);
  enumerateds.push_back(PRODUCTION);
  enumerateds.push_back(POST_GAME);
  return enumerateds;
}

}

}

