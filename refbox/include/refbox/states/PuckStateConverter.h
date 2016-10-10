/**
 *  This header file defines the PuckStateConverter class, which is
 *  also an EnumConverter of PuckState enums.
 *
 *  Version: 1.0.0
 *  Created on: 10/10/2016
 *  Modified on: 10/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _PUCK_STATE_CONVERTER_H_
#define _PUCK_STATE_CONVERTER_H_

#include "EnumConverter.h"
#include "refbox/states/PuckState.h"

namespace refbox
{

namespace states
{

class PuckStateConverter : public EnumConverter<PuckState>
{
public:
  PuckStateConverter(int id);
  PuckStateConverter(std::string nome);
  PuckStateConverter(PuckState enumerated);
  virtual ~PuckStateConverter();
  using EnumConverter::getEnumerated;
  virtual PuckState getEnumerated(int id) const;
  virtual PuckState getEnumerated(std::string nome) const;
  using EnumConverter::getId;
  virtual int getId(std::string nome) const;
  virtual int getId(PuckState enumerated) const;
  using EnumConverter::str;
  virtual std::string str(PuckState enumerated) const;
  using EnumConverter::c_str;
  virtual const char* c_str(PuckState enumerated) const;

  static int toId(PuckState enumerated);
  static PuckState toEnumerated(int id);
  static PuckState toEnumerated(std::string nome);
  static bool isValid(std::string nome);
  static std::string toString(PuckState enumerated);
  static const char* toCString(PuckState enumerated);
  static PuckState getDefault();
  static std::vector<PuckState> getAll();
};

}

typedef typename states::PuckStateConverter PuckStateConverter;

}


#endif // _PUCK_STATE_CONVERTER_H_
