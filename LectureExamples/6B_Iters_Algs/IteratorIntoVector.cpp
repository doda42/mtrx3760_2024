// Demonstrates an iterator into a std::vector
#include <iostream>
#include <vector>

int main ()
{
  std::vector<int> MyVector = {1,4,9,16,25};
  std::vector<int>::iterator MyIt;

  // traverse the list in the forward direction
  for( MyIt = MyVector.begin();   // begin is always the first element
       MyIt != MyVector.end();    // end is always one past the last element
       ++MyIt )                   // all iterators can increment
  {
    std::cout << *MyIt << ' ';    // and all iterators can dereference
  }
  std::cout << std::endl;

  // now do the same in reverse
  do
  {
    --MyIt;                       // only bidirectional iterators can decrement
    std::cout << *MyIt << ' ';    // multipass reading, this is the 2nd time
  } while( MyIt != MyVector.begin() );
  std::cout << std::endl;

  // copying iterators is okay; const iterators make const-correct code possible
  std::vector<int>::const_iterator MyConstIt;
  MyConstIt = MyIt;

  // and random access
  int WhichOne;
  std::cout << "Which one? ";
  std::cin >> WhichOne;
  std::cout << MyConstIt[WhichOne] << std::endl; // should have checked that input!
}
