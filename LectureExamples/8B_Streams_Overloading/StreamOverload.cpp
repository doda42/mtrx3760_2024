#include <iostream>
#include <ostream>
#include <sstream>

class Odo  
{  
public:  
  Odo() : mX(0), mY(0) {}

  Odo& operator++(){  // Prefix increment operator.  
    mX++;  
    mY++;  
    return *this;  
  }  

  Odo operator++(int){  // Postfix increment operator, int is ignored
    Odo temp = *this;  
    ++*this;  
    return temp;  
  }  

  Odo operator+( const Odo& rhs ) { // binary add
    Odo temp;
    temp.mX = this->mX + rhs.mX;
    temp.mY = this->mY + rhs.mY;
    return temp;
  }
  
  Odo& operator+=(const Odo& rhs) {
    mX += rhs.mX;
    mY += rhs.mY;
    return *this; // return the result by reference
  }

  friend std::ostream& operator<<( std::ostream& os, const Odo& obj );
  friend std::istream& operator>>( std::istream& is, Odo& obj );

private:
  int mX, mY;  
};  
  

std::ostream& operator<<( std::ostream& os, const Odo& obj )
{
  os << "[" << obj.mX << ", " << obj.mY << "]";
  return os;
}

std::istream& operator>>( std::istream& is, Odo& obj )
{
  is.ignore(1); // ignore [
  is >> obj.mX;
  is.ignore(1); // ingore ,
  is >> obj.mY;
  is.ignore(1); // ignore ]
  return is;
}

int main()  
{
  Odo Leg1, Leg2, Leg3;
  std::cout << ++Leg1 << std::endl;
  std::cout << Leg2++ << std::endl;

  std::cout << (  Leg3 = ++Leg1 + Leg2++ ) << std::endl;
  
  Odo TotLeg = Leg1+Leg2+Leg3;
  std::cout << TotLeg << Leg3 << Leg2 << Leg1 << std::endl;
  
  std::stringstream SS;
  SS << TotLeg;
  
  Odo LoadedLeg;
  SS >> LoadedLeg;
  
  std::cout << LoadedLeg << std::endl;
}

