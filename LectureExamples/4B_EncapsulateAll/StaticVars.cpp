#include <iostream>


void MyFunction()
{
  int NumToAdd = rand() % 10;
  
  // do work

  //--Debugging: track the mean number added per call
  static int Sum=0;
  static int Calls=0;  
  Sum += NumToAdd;
  ++Calls;
  std::cout << "Mean : " << double(Sum)/double(Calls) << std::endl;
}


int main()
{
  for( int j=0; j<200000; ++j )
    MyFunction();
}
