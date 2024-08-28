// Demonstrates that std::sort can't deal with lists, 
// remember a list is a doubly-linked list, which lacks
// a random-access iterator
#include <iostream>
#include <list>
#include <algorithm>

int main ()
{
  // make a container
  std::list<int> MyList = {234, 436, 123, 1, 7436, 21};

  // display
  for( int Val : MyList )
    std::cout << Val << ' ';
  std::cout << std::endl;

  // sort the contents of the container      
  std::sort( MyList.begin(), MyList.end() ); // compiler error

  // display
  for( int Val : MyList )
    std::cout << Val << ' ';
  std::cout << std::endl;
}

