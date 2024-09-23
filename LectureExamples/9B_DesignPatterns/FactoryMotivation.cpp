// Demonstrate motivation for the factory design pattern
#include <iostream>
#include <vector>

class Platform {
  public:
    virtual void DoStuff() = 0;
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
    else if (choice == 1)
      RobotList.push_back(new Quad);
    else if (choice == 2)
      RobotList.push_back(new AUV);
  }
  
  for (int i = 0; i < RobotList.size(); i++)
    RobotList[i]->DoStuff();
  for (int i = 0; i < RobotList.size(); i++)
    delete RobotList[i];
}
