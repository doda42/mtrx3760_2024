#include <iostream>

class CRobot
{
  public:
    typedef unsigned int t_message;
    
    void BroadcastMessage( const t_message& aMessage ) 
    { std::cout << "Broadcasting: " << aMessage << std::endl; }
    const t_message BuildMessage()
    { return rand(); }
};


int main()
{
  CRobot Constance;
  CRobot Neo;
  
  CRobot::t_message MyMessage = Neo.BuildMessage();
  Neo.BroadcastMessage( MyMessage );
}




