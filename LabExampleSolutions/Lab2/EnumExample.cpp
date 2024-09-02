// Demonstrates appropriate use of enumerations for named entities

#include <assert.h>
#include <iostream>


class CCar
{
  public:
    enum eDrivingStyles
    {
      DRIVE_FAST = 0,     // doesn't rely on the compiler making the assignment you assume it will
      DRIVE_SLOW,
      DRIVE_SIDEWAYS,
      DRIVE_INVALID       // allows range checking
    };

    void Drive( eDrivingStyles WhichStyle )
    {
      assert( WhichStyle < DRIVE_INVALID && WhichStyle >= 0 );
      std::cout << WhichStyle << std::endl;
    }

};

int main()
{
  CCar MyCar;
  MyCar.Drive( CCar::DRIVE_FAST );
  MyCar.Drive( CCar::eDrivingStyles(CCar::DRIVE_FAST-1) );

  return 0;
}
