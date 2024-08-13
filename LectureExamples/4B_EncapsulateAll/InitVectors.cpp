#include <iostream>
#include <vector>

class CCar
{
  public:
    /* explicit */ CCar( int aInfo ) : mInfo(aInfo) {}
  private:
    int mInfo;
};

class OldGarage
{
  public:
    static const std::vector<CCar> CarsThatDontMove;
};

const std::vector<CCar> OldGarage::CarsThatDontMove = {CCar(53), CCar(22)};


int main()
{
  std::vector<CCar> myCars = { CCar(1), CCar(3), CCar(8) };

  // ... or without the "explicit" keyword on the CCar constructor:  
  std::vector<CCar> moreCars = { 1,3,8 };
  std::vector<CCar> stillCars{ 1,3,8 };
}
