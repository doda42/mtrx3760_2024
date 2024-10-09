// Example of using the sensor hardware emulator class live-coded in lecture


#include "HardwareEmulator.h"

#include <iostream>


class SensorReceiver: public ISensorCallable
{
  public:
    void Callback( int Value )
    {
      std::cout << "SensorReceiver got value " << Value << std::endl;
    }
};

int main()
{
  SensorReceiver MyReceiver;
  CHardwareEmulator MyEmulator( MyReceiver );
  
  
  // wait for a bit to let some events happen
  std::this_thread::sleep_for(std::chrono::seconds(5));
  
  return 0;
}
