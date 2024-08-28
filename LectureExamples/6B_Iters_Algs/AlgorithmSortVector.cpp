// Demonstrates std::sort
#include <iostream>
#include <vector>
#include <algorithm>

int main ()
{
  // make a container
  std::vector<int> MyVector = {234, 436, 123, 1, 7436, 21};

  // display
  for( int Val : MyVector )
    std::cout << Val << ' ';
  std::cout << std::endl;

  // sort the contents of the container      
  std::sort( MyVector.begin(), MyVector.end() );

  // display
  for( int Val : MyVector )
    std::cout << Val << ' ';
  std::cout << std::endl;
}

