#include <iostream>

class WheelInfo
{
  public:
    explicit WheelInfo( int aInfo ) : mInfo(aInfo) {}
    
  private:
    int mInfo;
};


class CCar
{
  public:
    CCar( int aWheels ) : mNumWheels(aWheels) {}
    
     // a per-instance constant, note member init list above
    const int mNumWheels;

    // static consts, the same for all CCars
    static const double mAwesomeness; 
    static const WheelInfo mWheelInfo;
};

const double CCar::mAwesomeness = 4.2;
const WheelInfo CCar::mWheelInfo = WheelInfo( 92.1 );


int main()
{
  CCar myCar( 37 );
  CCar yourCar( 42 );
  
  const int* pW1 = &myCar.mNumWheels;
  const int* pW2 = &yourCar.mNumWheels;
  
  if( pW1 != pW2 )
    std::cout << "Per-instance const" << std::endl;
  
  const double* pA1 = &myCar.mAwesomeness;
  const double* pA2 = &yourCar.mAwesomeness;
  
  if( pA1 == pA2 )
    std::cout << "The same for all instances" << std::endl;
}
