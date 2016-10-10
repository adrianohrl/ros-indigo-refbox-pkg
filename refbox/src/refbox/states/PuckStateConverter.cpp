/**
 *  This source file implements the PuckStateConverter class.
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "refbox/states/PuckStateConverter.h"

namespace refbox
{

namespace states
{

PuckStateConverter::PuckStateConverter(int id)
  : EnumConverter(getEnumerated(id))
{}

PuckStateConverter::PuckStateConverter(std::string nome)
  : EnumConverter(getEnumerated(nome))
{}

PuckStateConverter::PuckStateConverter(PuckState enumerated)
  : EnumConverter(enumerated)
{}

PuckStateConverter::~PuckStateConverter()
{}

PuckState PuckStateConverter::getEnumerated(int id) const
{
  switch (id)
  {
  case 0:
    return S0;
  case 1:
    return S1;
  case 2:
    return S2;
  case 3:
    return P1;
  case 4:
    return P2;
  case 5:
    return P3;
  case 6:
    return CONSUMED;
  }
  return PuckStateConverter::getDefault();
}

PuckState PuckStateConverter::getEnumerated(std::string nome) const
{
  if (nome == "S0")
  {
    return S0;
  }
  else if (nome == "S1")
  {
    return S1;
  }
  else if (nome == "S2")
  {
    return S2;
  }
  else if (nome == "P1")
  {
    return P1;
  }
  else if (nome == "P2")
  {
    return P2;
  }
  else if (nome == "P3")
  {
    return P3;
  }
  else if (nome == "CONSUMED")
  {
    return CONSUMED;
  }
  return getDefault();
}

int PuckStateConverter::getId(std::string nome) const
{
  if (nome == "S0")
  {
    return 0;
  }
  else if (nome == "S1")
  {
    return 1;
  }
  else if (nome == "S2")
  {
    return 2;
  }
  else if (nome == "P1")
  {
    return 3;
  }
  else if (nome == "P2")
  {
    return 4;
  }
  else if (nome == "P3")
  {
    return 5;
  }
  else if (nome == "CONSUMED")
  {
    return CONSUMED;
  }
  return -1;
}

int PuckStateConverter::getId(PuckState enumerated) const
{
  switch (enumerated)
  {
  case S0:
    return 0;
  case S1:
    return 1;
  case S2:
    return 2;
  case P1:
    return 3;
  case P2:
    return 4;
  case P3:
    return 5;
  case CONSUMED:
    return 6;
  }
  return -1;
}

std::string PuckStateConverter::str(PuckState enumerated) const
{
  switch (enumerated)
  {
  case S0:
    return "S0";
  case S1:
    return "S1";
  case S2:
    return "S2";
  case P1:
    return "P1";
  case P2:
    return "P2";
  case P3:
    return "P3";
  case CONSUMED:
    return "CONSUMED";
  }
  return "";
}

const char* PuckStateConverter::c_str(PuckState enumerated) const
{
  return str(enumerated).c_str();
}

int PuckStateConverter::toId(PuckState enumerated)
{
  PuckStateConverter converter(enumerated);
  return converter.getId();
}

PuckState PuckStateConverter::toEnumerated(int id)
{
  PuckStateConverter converter(id);
  return converter.getEnumerated();
}

PuckState PuckStateConverter::toEnumerated(std::string nome)
{
  PuckStateConverter converter(nome);
  return converter.getEnumerated();
}

bool PuckStateConverter::isValid(std::string nome)
{
  return nome == "S0" || nome == "S1" || nome == "S2" ||
      nome == "P1" || nome == "P2" || nome == "P3" || nome == "CONSUMED";
}

std::string PuckStateConverter::toString(PuckState enumerated)
{
  PuckStateConverter converter(enumerated);
  return converter.str();
}

const char* PuckStateConverter::toCString(PuckState enumerated)
{
  PuckStateConverter converter(enumerated);
  return converter.c_str();
}

PuckState PuckStateConverter::getDefault()
{
  return S0;
}

std::vector<PuckState> PuckStateConverter::getAll()
{
  std::vector<PuckState> enumerateds;
  enumerateds.push_back(S0);
  enumerateds.push_back(S1);
  enumerateds.push_back(S2);
  enumerateds.push_back(P1);
  enumerateds.push_back(P2);
  enumerateds.push_back(P3);
  enumerateds.push_back(CONSUMED);
  return enumerateds;
}

}

}

