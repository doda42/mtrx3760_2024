// Example of a sensor node using inheritance internally to expose a node
// with different behaviours at runtime; this is the version that was
// produced ahead of time while working out the problem. For the version
// coded live in-lecture see SensorNodeLiveCoded.cpp.
//
// In this example we'll use a stand-in CNode that knows how to "publish" by
// printing to the screen as a stand-in for ROS 2. This allows for lightweight
// development during the early design stage for this project.

#include "HardwareEmulator.h"

#include <iostream>
#include <string>

//---Pretend-Node--------------------------------------------------------------
// A minimalist class to stand in for ROS in small examples
class CNode
{
  public:
    void Publish( std::string Message );
};

//---Hardware implementation of fire detector----------------------------------
// Receives a callback every time the fire detector hardware returns a value
class CFireDetector: public ISensorCallable
{
  public:
    CFireDetector() { std::cout << "CFireDetector CTOR" << std::endl; }
};

//---Hardware implementation of fire detector----------------------------------
// Receives a callback every time the fire detector hardware returns a value
class CFireDetectorCamera: public CFireDetector
{
  public:
    CFireDetectorCamera() 
    {
      // register Callback with manufacturer-
    }
    void Callback( int Value ) 
    {  
      std::cout << "Fire detector camera got value: " << Value << std::endl;
    }
};


//---Hardware implementation of fire detector----------------------------------
// Receives a callback every time the fire detector hardware returns a value
class CFireDetectorPhotodiode: public CFireDetector
{
  public:
    CFireDetectorPhotodiode() 
    {
      // register Callback with manufacturer-
    }
    void Callback( int Value ) 
    {  
      std::cout << "Fire detector photodiode got value: " << Value << std::endl;
    }
};


//-----------------------------------------------------------------------------
class FireDetectorNode: public CNode
{
  public:
    FireDetectorNode() : mpFireDetector(0) 
    {
      // detect whether a hardware-based fire detector is present
      int ModeSel;
      std::cout << "Enter 0 for camera-based fire detection, 1 for photodiode-based" << std::endl;
      std::cin >> ModeSel;
      switch( ModeSel )
      {
        case 0:
          mpFireDetector = new CFireDetectorCamera;
        break;
        case 1:
          mpFireDetector = new CFireDetectorPhotodiode;
        break;
        default:
          std::cout << "Error" << std::endl;
      }
    }
    ~FireDetectorNode()
    {
      if( mpFireDetector ) 
        delete mpFireDetector;
    }
    
    CFireDetector& GetFireDetector()
    {
      return *mpFireDetector;
    }
    
  private:
    CFireDetector* mpFireDetector;

};

//-----------------------------------------------------------------------------
int main()
{
  FireDetectorNode MyNode;
  CHardwareEmulator MyEmulator( MyNode.GetFireDetector() );

  // wait for a bit to let some events happen
  std::this_thread::sleep_for(std::chrono::seconds(5));

  return 0;
}


//-----------------------------------------------------------------------------
void CNode::Publish( std::string Message )
{
  std::cout << Message << std::endl;
}
