// A start on the node design for the fire sensor problem
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


//-----------------------------------------------------------------------------
int main()
{

  return 0;
}


//-----------------------------------------------------------------------------
void CNode::Publish( std::string Message )
{
  std::cout << Message << std::endl;
}
