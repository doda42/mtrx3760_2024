// Demonstrates std::sort
#include <iostream>
#include <array>
#include <algorithm>

int main ()
{
  // make a container
  std::array<int, 6> MyArray = {234, 436, 123, 1, 7436, 21};

  // display
  for( int Val : MyArray )
    std::cout << Val << ' ';
  std::cout << std::endl;

  // sort the contents of the container      
  std::sort( MyArray.begin(), MyArray.end() );

  // display
  for( int Val : MyArray )
    std::cout << Val << ' ';
  std::cout << std::endl;
}

