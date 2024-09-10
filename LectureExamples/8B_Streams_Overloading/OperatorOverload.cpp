#include <iostream>

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

private:
  int mX, mY;  
};  
  
  
int main()  
{
  Odo Leg1, Leg2, Leg3;
  ++Leg1;
  Leg2++;
  Leg3 = ++Leg1 + Leg2++;
  
  Odo TotLeg = Leg1+Leg2+Leg3;
}  

