// Example of a sensor node using inheritance internally to expose a node
// with different behaviours at runtime; this is the version that was
// produced live in-lecture.
//
// In this example we'll use a stand-in CNode that knows how to "publish" by
// printing to the screen as a stand-in for ROS 2. This allows for lightweight
// development during the early design stage for this project.

#include "HardwareEmulator.h"

#include <iostream>
#include <string>
#include <vector>

//---Pretend-Node--------------------------------------------------------------
// A minimalist class to stand in for ROS in small examples
class CNode
{
  public:
    void Publish( std::string Message );
};

class FireSensor: public ISensorCallable
{
  public:
    void TrackHistory( int Val ) { /*...*/ }
  private:
    std::vector<int> mHistory;
};
class FireSensorCamera: public FireSensor
{
  public:
    void Callback( int Value ) 
    { 
      std::cout << "Fire sensor got value " << Value << std::endl; 
      TrackHistory( Value );
    }
    FireSensorCamera(){ std::cout << "Cam FS Ctor" << std::endl; }
};
class FireSensorPhotodiode: public FireSensor
{
  public:
    void Callback( int Value ) 
    { 
      std::cout << "Photdiode sensor got value " << Value << std::endl; 
      TrackHistory( Value );      
    }
    FireSensorPhotodiode(){ std::cout << "Photodiode FS Ctor" << std::endl; }
};


class FireSensorNode: public CNode
{
  public:
    FireSensorNode()
      : mSensor(0)
    {
      std:: cout << "FS Node Ctor" << std::endl;
      std::cout << "What kind of sensor? 0 for cam, 1 for photodiode" << std::endl;
      int Choice;
      std::cin >> Choice;
      switch( Choice )
      {
        case 0:
          mSensor = new FireSensorCamera;
        break;
        case 1:
          mSensor = new FireSensorPhotodiode;
        break;
        default:
          std::cout << "Error" << std::endl;
      }
    }
    ~FireSensorNode()
    {
      std:: cout << "FS Node Dtor" << std::endl;
      if( mSensor )
        delete mSensor;
    }
    
    FireSensor* GetSensor() { return mSensor; }
    
  private:
    FireSensor* mSensor; // I want to use polymorphism
};


//-----------------------------------------------------------------------------
int main()
{

  FireSensorNode MyNode;
  CHardwareEmulator MyEmulator( *MyNode.GetSensor() );
  
  // wait for a bit to let some events happen
  std::this_thread::sleep_for(std::chrono::seconds(5));

  
  return 0;
}


//-----------------------------------------------------------------------------
void CNode::Publish( std::string Message )
{
  std::cout << Message << std::endl;
}
