// Demonstrate use of a singleton
#include <iostream>

class SensorMgr
{
public:
  static SensorMgr* getInstance();    // single point of access
  int ID;
private:
  static SensorMgr* instance;         // static pointer to instance
  SensorMgr(const SensorMgr&);        // not implemented so you can't copy
  void operator=(const SensorMgr&);   // not implemented so you can't assign
  SensorMgr() { }                     // private so you can't instantiate
};
SensorMgr* SensorMgr::instance = 0;   // Init to 0 when the program starts

//--Single point of access---------------------
SensorMgr* SensorMgr::getInstance()
{
  if( instance == 0 ) { // true on first call only
    instance = new SensorMgr();       // instantiation happens exactly once
  }
  return instance;
}

//---
int main()
{
  // Anywhere in the program SensorMgr::getInstance() accesses the same, unique instance
  SensorMgr::getInstance()->ID = 42;
  std::cout << SensorMgr::getInstance()->ID << std::endl;

  // SensorMgr WillFail;                    // this fails, there can be only one
  // SensorMgr* pSingle = new SensorMgr();  // this fails
}
