#include "HardwareEmulator.h"

#include <iostream>
#include <thread>
#include <chrono>

//-----------------------------------------------------------------------------
CHardwareEmulator::CHardwareEmulator( ISensorCallable& CallbackObject )
  : Running( false ), mCallbackObject( CallbackObject ) 
{
  Start();
}

//-----------------------------------------------------------------------------
CHardwareEmulator::~CHardwareEmulator() 
{
  Stop();
}

//-----------------------------------------------------------------------------
void CHardwareEmulator::Start() 
{
  Running = true;
  // Create the thread
  TimerThread = std::thread( &CHardwareEmulator::RunTimer, this );
  std::cout << "Started hardware emulator" << std::endl;
}

//-----------------------------------------------------------------------------
void CHardwareEmulator::Stop() 
{
  Running = false;
  if (TimerThread.joinable()) 
  {
    TimerThread.join();
  }
  std::cout << "Stopped hardware emulator" << std::endl;
}

//-----------------------------------------------------------------------------
// The timer that runs in its own thread
void CHardwareEmulator::RunTimer() 
{
  while( Running )
  {
    static int i=0;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    mCallbackObject.Callback( i ); // Call the registered callback
    ++i;
  }
}

