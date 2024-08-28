// Demonstrates std::sort
#include <iostream>
#include <algorithm>

int main ()
{
  // make a container
  int MyArray[] = {234, 436, 123, 1, 7436, 21};

  // display
  for( int Val : MyArray )
    std::cout << Val << ' ';
  std::cout << std::endl;

  // sort the contents of the container      
  std::sort( MyArray, MyArray + 6 );

  // display
  for( int Val : MyArray )
    std::cout << Val << ' ';
  std::cout << std::endl;
}

