// demonstrates that an ordinary pointer qualifies as a random access iterator

#include <iostream>

int main()
{
  int MyArray[] = {1,1,2,3,5,8};
  int *MyIter = &MyArray[2];
  int Read1 = *MyIter; // read
  --MyIter; // decrement
  ++MyIter; // increment
  int Read2 = *MyIter; // read multiple times
  int Read3 = MyIter[3]; // random access (3 could by any offset)
  
  std::cout << Read1 << ", " << Read2 << ", " << Read3 << std::endl;
}
