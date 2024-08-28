// Demonstrates an iterator into a std::map
#include <iostream>
#include <map>

int main ()
{
  std::map<int, char> MyMap = {{1, 'a'}, {3, 'b'}, {5, 'c'}, {7, 'd'}};
  std::map<int, char>::iterator MyIt;

  // traverse the map in the forward direction
  for( MyIt = MyMap.begin();   // begin is always the first element
       MyIt != MyMap.end();    // end is always one past the last element
       ++MyIt )                   // all iterators can increment
  {
    std::pair<int, char> CurVal = *MyIt; // and all iterators can dereference
    std::cout << CurVal.first << ", " << CurVal.second << "; ";
  }
  std::cout << std::endl;

  // now do the same in reverse
  do
  {
    --MyIt;                       // only bidirectional iterators can decrement
    std::pair<int, char> CurVal = *MyIt;
    std::cout << CurVal.first << ", " << CurVal.second << "; ";
  } while( MyIt != MyMap.begin() );
  std::cout << std::endl;

  // copying iterators is okay; const iterators make const-correct code possible
  std::map<int, char>::const_iterator MyConstIt;
  MyConstIt = MyIt;

  // random access throws a compilter error, this is not a random access iterator
  // std::pair<int, char> CurVal = MyConstIt[3];
}
