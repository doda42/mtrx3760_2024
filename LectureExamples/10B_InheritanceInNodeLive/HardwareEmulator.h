// Hardware emulator class
//
// This emulates the case of having a low-level hardware API, e.g. from a camera or 
// sensor manufacturer, that generates a callback when each sensor reading is ready
//
// To use it, make a class that derives from ISensorCallable and defines the Callback
// function. Then instantiate a CHardwareEmulator and provide the callback object to
// the constructor.
//
// This will set up a thread that generates a callback every second, passing a value
// that increments by one on each call.
//
// When compiling you must link in libthread with the -l compiler flag, e.g. 
// g++ HardwareEmulator.cpp SensorNode.cpp -lpthread
//
// Copyright (c) Donald Dansereau, 2024

#ifndef __HARDWAREEMULATOR_H
#define __HARDWAREEMULATOR_H

#include <thread>


//---Base class for classes that accept callbacks from the sensor----------------------
// Inherit from and define Callback to make a class that receives callbacks
class ISensorCallable
{
  public:
    virtual void Callback( int Value ) = 0;
};


//---Emulator for sensor hardware-------------------------------------------------------
// Generates callbacks emulating sensor message arrivals
class CHardwareEmulator 
{
  public:
    // Constructor takes a reference to an instance of the class that has the callback
    // Note this implementation automatically starts the timer-generated callbacks in the
    // CTor, and automatically stops them in the DTor
    CHardwareEmulator( ISensorCallable& CallbackObject );
    ~CHardwareEmulator(); 
    
    void Start();
    void Stop();

  private:
    bool Running;
    std::thread TimerThread;
    ISensorCallable& mCallbackObject; // Reference to the callback object
    
    // The timer that runs in the separate thread and generates callbacks
    void RunTimer();
};


#endif
