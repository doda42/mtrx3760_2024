// Demonstrate the factory design pattern
// see FactoryMotivation.cpp to see the same without a factory
#include <iostream>
#include <vector>

class Platform {
  public:
    virtual void DoStuff() = 0;
    // Factory Method
    static Platform *MakePlatform( int choice );
};

class Quad: public Platform {
  public:
    void DoStuff() {
      std::cout << "Quadrotoring around" << std::endl;
    }
};
class AUV: public Platform {
  public:
    void DoStuff() {
      std::cout << "AUVing with the fishes" << std::endl;
    }
};

int main()
{
  std::vector<Platform*> RobotList;
  int choice;

  while( true ) {
    std::cout << "1: Quad 2: AUV 0: Go ";
    std::cin >> choice;
    if (choice == 0)
      break;
    RobotList.push_back( Platform::MakePlatform(choice) );
  }

  for (int i = 0; i < RobotList.size(); i++)
    RobotList[i]->DoStuff();
  for (int i = 0; i < RobotList.size(); i++)
    delete RobotList[i];
}


// now all the complexity of choosing which object to make, and actually making it,
// is hidden in the factory function
Platform *Platform::MakePlatform( int choice )
{
  switch( choice )
  {
    case 1: return new Quad; break;
    case 2: return new AUV; break;
  }
}
