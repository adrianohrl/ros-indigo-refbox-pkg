/**
 *  This source file implements the GameStateConverter class.
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "refbox/states/GameStateConverter.h"

namespace refbox
{

namespace states
{

GameStateConverter::GameStateConverter(int id)
  : EnumConverter(getEnumerated(id))
{}

GameStateConverter::GameStateConverter(std::string nome)
  : EnumConverter(getEnumerated(nome))
{}

GameStateConverter::GameStateConverter(GameState enumerated)
  : EnumConverter(enumerated)
{}

GameStateConverter::~GameStateConverter()
{}

GameState GameStateConverter::getEnumerated(int id) const
{
  switch (id)
  {
  case 0:
    return INIT;
  case 1:
    return WAIT_START;
  case 2:
    return RUNNING;
  case 3:
    return PAUSED;
  }
  return GameStateConverter::getDefault();
}

GameState GameStateConverter::getEnumerated(std::string nome) const
{
  if (nome == "INIT")
  {
    return INIT;
  }
  else if (nome == "WAIT_START")
  {
    return WAIT_START;
  }
  else if (nome == "RUNNING")
  {
    return RUNNING;
  }
  else if (nome == "PAUSED")
  {
    return PAUSED;
  }
  return getDefault();
}

int GameStateConverter::getId(std::string nome) const
{
  if (nome == "INIT")
  {
    return 0;
  }
  else if (nome == "WAIT_START")
  {
    return 1;
  }
  else if (nome == "RUNNING")
  {
    return 2;
  }
  else if (nome == "PAUSED")
  {
    return 3;
  }
  return -1;
}

int GameStateConverter::getId(GameState enumerated) const
{
  switch (enumerated)
  {
  case INIT:
    return 0;
  case WAIT_START:
    return 1;
  case RUNNING:
    return 2;
  case PAUSED:
    return 3;
  }
  return -1;
}

std::string GameStateConverter::str(GameState enumerated) const
{
  switch (enumerated)
  {
  case INIT:
    return "INIT";
  case WAIT_START:
    return "WAIT_START";
  case RUNNING:
    return "RUNNING";
  case PAUSED:
    return "PAUSED";
  }
  return "";
}

const char* GameStateConverter::c_str(GameState enumerated) const
{
  return str(enumerated).c_str();
}

int GameStateConverter::toId(GameState enumerated)
{
  GameStateConverter converter(enumerated);
  return converter.getId();
}

GameState GameStateConverter::toEnumerated(int id)
{
  GameStateConverter converter(id);
  return converter.getEnumerated();
}

GameState GameStateConverter::toEnumerated(std::string nome)
{
  GameStateConverter converter(nome);
  return converter.getEnumerated();
}

bool GameStateConverter::isValid(std::string nome)
{
  return nome == "INIT" || nome == "WAIT_START" || nome == "RUNNING" || nome == "PAUSED";
}

std::string GameStateConverter::toString(GameState enumerated)
{
  GameStateConverter converter(enumerated);
  return converter.str();
}

const char* GameStateConverter::toCString(GameState enumerated)
{
  GameStateConverter converter(enumerated);
  return converter.c_str();
}

GameState GameStateConverter::getDefault()
{
  return INIT;
}

std::vector<GameState> GameStateConverter::getAll()
{
  std::vector<GameState> enumerateds;
  enumerateds.push_back(INIT);
  enumerateds.push_back(WAIT_START);
  enumerateds.push_back(RUNNING);
  enumerateds.push_back(PAUSED);
  return enumerateds;
}

}

}

