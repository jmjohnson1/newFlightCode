#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <vector>

// The idea is that this would handle items of type int or float
template <typename t>
class LogItem {
public:
  LogItem(t* const var, String varName, int digitsOrBase);

  String GetName() {return variableName_};
  t* GetPtr() {return variablePtr_;}
  int GetDigitsOrBase() {return digitsOrBase_};

private:
  String variableName_ = "";
  t *variablePtr_ = nullptr;
  int digitsOrBase_ 10;
};

template <typename t>
class Datalogger {
public:
  void Init();
  void AddItem(t* const var, String varName, int digitsOrBase);
  void PrintHeader();
  void Write();
  void End();
private:
  std::vector<LogItem> items_;
};


#endif