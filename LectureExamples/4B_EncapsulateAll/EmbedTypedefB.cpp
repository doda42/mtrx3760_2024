#include <iostream>
#include <string>

class CRobot
{
  public:
    typedef std::string t_message;
    
    void BroadcastMessage( const t_message& aMessage ) 
    { std::cout << "Broadcasting: " << aMessage << std::endl; }
    const t_message BuildMessage()
    { return "Hey there"; }
};


int main()
{
  CRobot Constance;
  CRobot Neo;
  
  CRobot::t_message MyMessage = Neo.BuildMessage();
  Neo.BroadcastMessage( MyMessage );
}
